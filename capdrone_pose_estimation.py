import cv2
from pathlib import Path
import incremental_bundle_adjustment as iba
import pose_estimation as pe
import numpy as np
from scipy.spatial.transform import Rotation
from pupil_apriltags import Detector
from pose_estimation import dcm_from_rpy, rpy_from_dcm





def get_T_Marker_Ais(april_tag_img: np.ndarray, detector: Detector, cam_intrinsics: np.ndarray, distortion_coeffs: np.ndarray, T_Marker_Camera: np.ndarray):
    """
    Estimates the pose of the camera relative to the global vicon frame.
    april_tag_img_files: list of april tag image paths.
    detector: april tag detector object.
    cam_intrinsics: 3x3 numpy array of the camera intrinsics matrix
    distortion_coeffs: 5x1 camera distortion coefficients.
    T_1_Ai: 4x4 numpy array of the transformation from the tag frame to the base frame. NOTE: CURRENTLY NOT USED AT ALL.
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

