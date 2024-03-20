import cv2
from pathlib import Path
import incremental_bundle_adjustment as iba
import pose_estimation as pe
import numpy as np
from scipy.spatial.transform import Rotation
from pupil_apriltags import Detector
from pose_estimation import dcm_from_rpy, rpy_from_dcm
from typing import List, Tuple, Dict
import numpy as np
from scipy.optimize import least_squares, approx_fprime

PxPositionMap = Dict[int, List[Tuple[int, int]]]  # image_index -> [(x, y)]
MmPositionMap = Dict[int, List[Tuple[float, float, float]]]  # image_index -> [(x, y, z=0)]
Pose = np.ndarray  # 4x4 homogeneous transformation matrix



def check_if_tags_present(image: np.ndarray, detector: Detector) -> bool:
    """
    Check if there is an april tag in an image. Note this might not be very efficient potentially since im calling the detector each time an image 
    is passed tho idk if theres a specific function present in the Detector library that specifically does this.
    image: np.ndarray
    detector: April tag Detector object
    Returns:
    tags_present: bool, True if tags present in image, False if no tags detected
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BG2GRAY)
    tags = detector.detect(gray)
    tags_present = (len(tags) != 0)
    return tags_present

def get_T_Marker_Ais(april_tag_img: np.ndarray, detector: Detector, cam_intrinsics: np.ndarray, distortion_coeffs: np.ndarray, T_Marker_Camera: np.ndarray):
    """
    Estimates the pose of the april tag relative to the marker frame.
    april_tag_img: single np.ndarray image of april tags.
    detector: april tag detector object.
    cam_intrinsics: 3x3 numpy array of the camera intrinsics matrix
    distortion_coeffs: 5x1 camera distortion coefficients.
    Returns:
    T_C_A_dict: dict of T_C_A 4x4 numpy array of the transformation geting the April tag pose wrt camera pose, stored as: {tag_id0: [T_C_A1, ...], tag_id1: [T_C_A1, ...] ...}
    where each tag has atleast 1 and (probably) up to 2 Transformations due to noise. As of RIGHT NOW, only 1 transformation per tag_id but structure is still the same
    """

    # from pose_estimation import estimate_camera_pose
    print(april_tag_img.shape)  

    # Needs to be in grayscale for the apriltag detector
    gray = cv2.cvtColor(april_tag_img, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the image
    tags = detector.detect(gray) # NOTE: Potential issue, if this does cause an issue it will be very annoying, but image is not undistorted before passing to detector
    print(tags)
    print(type(tags))
    # list
    T_M_A_dict = {}
    for i, tag in enumerate(tags):
        # get tag pixel cords
        tag_corners_px = tag.corners
        tag_id = tag.id
        print(tag_corners_px, tag_corners_px.shape)
        # get tag mm cords
        tag_corners_mm_Ai = iba.get_corner_config(tag_sizes=[130], tag_config="single", use_center=False)

        # solves transformation using PnP
        T_C_Ai = pe.estimate_T_Ci_Ai(
                    tag_corners_px=tag_corners_px, 
                    tag_corners_mm_Ai=tag_corners_mm_Ai,
                    camera_matrix=cam_intrinsics, 
                    dist_coeffs=distortion_coeffs, 
                    )
        T_M_Ai = T_Marker_Camera @ T_C_Ai
        
        T_M_A_dict[tag_id] = [T_M_Ai]
        
    return T_M_A_dict # NOTE: double check that im estimating the right matrix and not its inverse.

def optimize_tag_pose(
    initial_tag_pose: Pose,
    tag_px_positions: PxPositionMap,
    drone_poses: Dict[int, Pose],
    tag_corners_mm_Ai: MmPositionMap,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    camera_extrinsics: Pose,
):
    """
    Optimize the estimated location of the tag in the VICON frame using the AprilTag corners and the drone poses
    Parameters:
    initial_tag_pose: The initial estimate of the tag pose in the VICON frame
    tag_px_positions: The pixel positions of the tag corners in the image
    drone_poses: The poses of the drone when the images were taken. Given as the transformation matrix T_VICON_drone, that is the pose of the drone in the VICON frame / transformation from drone to VICON frame
    tag_corners_mm_Ai: The 3D positions of the tag corners in the tag frame. The z component is always 0.
    camera_matrix: The camera intrinsics matrix
    distortion_coefficients: The camera distortion coefficients
    camera_extrinsics: The extrinsics of the camera in the Drone frame. Given as the transformation matrix T_drone_camera, that is the pose of the camera in the drone frame / transformation from camera to drone frame

    Returns:
    The optimized tag pose in the VICON frame
    """
    all_img_idxs = sorted(tag_px_positions.keys())
    params = params_from_T(initial_tag_pose)  # [x, y, z, roll, pitch, yaw]

    def err_func(params):
        tag_pose = T_from_params(params)

        expected_pixels: PxPositionMap = get_expected_pixels(tag_pose, tag_px_positions, drone_poses, tag_corners_mm_Ai, camera_matrix, distortion_coefficients, camera_extrinsics)

        all_expected_pixels = []
        all_actual_pixels = []
        for img_idx in all_img_idxs:
            expected = expected_pixels[img_idx]
            actual = tag_px_positions[img_idx]
            all_expected_pixels.extend(expected)
            all_actual_pixels.extend(actual)
        all_expected_pixels = np.array(all_expected_pixels)
        all_actual_pixels = np.array(all_actual_pixels)

        error = all_expected_pixels - all_actual_pixels

        return error

    def jac_func(params):
        tag_pose = T_from_params(params)

        jacobian = find_jacobian(tag_pose, tag_px_positions, drone_poses, tag_corners_mm_Ai, camera_matrix, distortion_coefficients, camera_extrinsics)

        return jacobian

    # result = least_squares(error_func, params, jac=jacobian_func, verbose=2)
    result = least_squares(error_func, params, jac='3-point', verbose=2)

    optimal_params = result.x
    optimal_tag_pose = T_from_params(optimal_params)

    return optimal_tag_pose

    
def get_rotation_and_translation(pose: Pose) -> Tuple[np.ndarray, np.ndarray]:
    """
    Get the rotation and translation from the pose matrix
    Parameters:
    pose: The pose matrix

    Returns:
    The rotation and translation
    """
    return pose[:3, :3], pose[:3, 3]

def get_expected_pixels(
    tag_pose: Pose,
    tag_px_positions: PxPositionMap,
    drone_poses: Dict[int, Pose],
    tag_corners_mm_Ai: MmPositionMap,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    camera_extrinsics: Pose,
) -> PxPositionMap:
    """
    Get the expected pixel positions of the tag corners in the image
    Parameters:
    tag_pose: The pose of the tag in the VICON frame
    tag_px_positions: The pixel positions of the tag corners in the image
    drone_poses: The poses of the drone when the images were taken. Given as the transformation matrix T_VICON_drone, that is the pose of the drone in the VICON frame / transformation from drone to VICON frame
    tag_corners_mm_Ai: The 3D positions of the tag corners in the tag frame. The z component is always 0.
    camera_matrix: The camera intrinsics matrix
    distortion_coefficients: The camera distortion coefficients
    camera_extrinsics: The extrinsics of the camera in the Drone frame. Given as the transformation matrix T_drone_camera, that is the pose of the camera in the drone frame / transformation from camera to drone frame

    Returns:
    The expected pixel positions of the tag corners in the image
    """
    expected_pixels = {}
    R_V_A, t_V_A = get_rotation_and_translation(tag_pose)  # Transformation to VICON from AprilTag
    tag_corner_position_mm = tag_corners_mm_Ai[img_idx]

    for img_idx, drone_pose in drone_poses.items():
        curr_corner_px_positions = tag_px_positions[img_idx]
        expected_pixels[img_idx] = []
        for corner_idx in range(len(tag_corner_position_mm)):
            # Get the position of the corner in the VICON frame
            p_V = R_V_A @ tag_corner_position_mm[corner_idx] + t_V_A

            # Get the position of the corner in the camera frame
            curr_drone_pose = drone_poses[img_idx]
            T_V_Ci = curr_drone_pose @ camera_extrinsics  # Transformation to VICON from camera frame
            R_V_Ci, t_V_Ci = get_rotation_and_translation(T_V_Ci)

            p_Ci = R_V_Ci.T @ (p_V - t_V_Ci)  # Taking T_Ci_V @ p_V = (T_V_Ci)^-1 @ p_V = R_V_Ci.T @ (p_V - t_V_Ci)

            # Project the position of the corner in the camera frame to the image plane and undistort them
            p_px = cv2.projectPoints(p_Ci, np.zeros(3), np.zeros(3), camera_matrix, distortion_coefficients)[0][0][0]

            expected_pixels[img_idx].append(p_px)

    return expected_pixels
    
def find_jacobian(
    tag_pose: Pose,
    tag_px_positions: PxPositionMap,
    drone_poses: Dict[int, Pose],
    tag_corners_mm_Ai: MmPositionMap,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    camera_extrinsics: Pose,
) -> np.ndarray:
    """
    Find the Jacobian matrix for the error function
    Parameters:
    tag_pose: The pose of the tag in the VICON frame
    tag_px_positions: The pixel positions of the tag corners in the image
    drone_poses: The poses of the drone when the images were taken. Given as the transformation matrix T_VICON_drone, that is the pose of the drone in the VICON frame / transformation from drone to VICON frame
    tag_corners_mm_Ai: The 3D positions of the tag corners in the tag frame. The z component is always 0.
    camera_matrix: The camera intrinsics matrix
    distortion_coefficients: The camera distortion coefficients
    camera_extrinsics: The extrinsics of the camera in the Drone frame. Given as the transformation matrix T_drone_camera, that is the pose of the camera in the drone frame / transformation from camera to drone frame

    Returns:
    The Jacobian matrix (2 * num_images * corners_per_tag, 6) for the error function
    Variable order: [x, y, z, roll, pitch, yaw]
    """
    all_img_idxs = sorted(tag_px_positions.keys())
    num_corners_per_tag = len(tag_corners_mm_Ai[all_img_idxs[0]])

    jacobian = np.zeros((2 * len(all_img_idxs) * num_corners_per_tag, 6))

    expected_pixels: PxPositionMap = get_expected_pixels(tag_pose, tag_px_positions, drone_poses, tag_corners_mm_Ai, camera_matrix, distortion_coefficients, camera_extrinsics)

    # Pre-baked matrices used to take derivatives of 1 axis rotation matrices
    X_bar_z = np.array([[0, -1, 0],[1, 0, 0],[0, 0, 0]])
    X_bar_y = np.array([[0, 0, 1],[0, 0, 0],[-1, 0, 0]])
    X_bar_x = np.array([[0, 0, 0],[0, 0, -1],[0, 1, 0]])

    R_V_A, t_V_A = get_rotation_and_translation(tag_pose)  # Transformation to VICON from AprilTag
    # Split the rotation matrix into the 3 axis rotations
    r, p, y = rpy_from_dcm(R_V_A).flatten()
    R_A_z = dcm_from_rpy(np.array([0, 0, y]))
    R_A_y = dcm_from_rpy(np.array([0, p, 0]))
    R_A_x = dcm_from_rpy(np.array([r, 0, 0]))

    for img_idx in all_img_idxs:
        drone_pose = drone_poses[img_idx]
        curr_corner_px_positions = tag_px_positions[img_idx]
        for corner_idx in range(len(tag_corners_mm_Ai[img_idx])):
            corner_pos_mm = tag_corners_mm_Ai[img_idx][corner_idx]

            # Compute intermediate values
            T_V_Ci = drone_pose @ camera_extrinsics
            R_V_Ci, t_V_Ci = get_rotation_and_translation(T_V_Ci)

            p_v = R_V_A @ corner_pos_mm + t_V_A  # (3,)
            p_ci = R_V_Ci.T @ (p_v - t_V_Ci)  # (3,)
            yi = camera_matrix @ p_ci  # (3,)

            # Intermediate derivatives
            d_xi_d_yi = np.array([
                [1 / yi[2], 0, -yi[0] / yi[2]**2],
                [0, 1 / yi[2], -yi[1] / yi[2]**2]
            ])  # (2, 3)

            d_yi_d_p_ci = camera_matrix  # (3, 3)

            d_p_ci_d_p_v = R_V_Ci.T  # (3, 3)

            d_xi_d_pv = d_xi_d_yi @ d_yi_d_p_ci @ d_p_ci_d_p_v  # (2, 3)

            d_pv_d_yaw   = X_bar_z @ R_A_z   @ R_A_y   @ R_A_x @ corner_pos_mm  # (3,)
            d_pv_d_pitch = R_A_z   @ X_bar_y @ R_A_y   @ R_A_x @ corner_pos_mm  # (3,)
            d_pv_d_roll  = R_A_z   @ R_A_y   @ X_bar_x @ R_A_x @ corner_pos_mm  # (3,)

            # Compute rotational derivatives
            d_xi_d_yaw = d_xi_d_pv @ d_pv_d_yaw  # (2,)
            d_xi_d_pitch = d_xi_d_pv @ d_pv_d_pitch  # (2,)
            d_xi_d_roll = d_xi_d_pv @ d_pv_d_roll  # (2,)

            # Compute translational derivatives
            d_xi_d_tA = d_xi_d_pv @ np.eye(3)  # (2, 3)

            # Arrange the final jacobian
            start_index = (img_idx * num_corners_per_tag + corner_idx) * 2
            end_index = start_index + 2
            jacobian[start_index:start_index+2, 0] = d_xi_d_tA[0]
            jacobian[start_index:start_index+2, 1] = d_xi_d_tA[1]
            jacobian[start_index:start_index+2, 2] = d_xi_d_tA[2]
            jacobian[start_index:start_index+2, 3] = d_xi_d_roll
            jacobian[start_index:start_index+2, 4] = d_xi_d_pitch
            jacobian[start_index:start_index+2, 5] = d_xi_d_yaw

    return jacobian
    