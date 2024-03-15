# Idea -> (camera to apr tag) @ [Identity](april tag to global) @ (glob vicon to loc vicon) @ (local vicon to camera) = I
# (camera to apr tag) @ [Identity](april tag to global) -> returned in get_T_C_vicon_global(...) fuction
# (glob vicon to loc vicon) -> returned somewhere -> look at comms_node or smt, will need rospy
#(glob vicon to loc vicon).T @ (camera to glob).T = (loc vicon to camera)

# tag size 130 mm
import cv2
from pathlib import Path
import incremental_bundle_adjustment as iba
import pose_estimation as pe
import numpy as np
from scipy.spatial.transform import Rotation
from pupil_apriltags import Detector
from pose_estimation import dcm_from_rpy, rpy_from_dcm
import rospy

from geometry_msgs.msg import PoseStamped, TransformStamped
# import tf2_ros
# import tf.transformations as tr
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

img_file = "add path to the april tag image at some point"

detector = Detector(
   families="tag36h11",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)
cam_params = np.load("./Extrinsic-Cam-Calibration/AprilTag-Bundle-Adjustment/cal.npz","r+")
cam_intrinsics = cam_params["mtx"]
distortion_coeffs = cam_params["dist"].T
# asser
# print(f"{cam_params},\n\n{cam_intrinsics.shape},\n\n{distortion_coeffs.shape}")

# def get_T_glob_loc_vicon(debug = False):
#     """
#     NOTE: NO LONGER USED
#     Relevant quaternion to R:
#     scipy.spatial.transform.Rotation.as_quat
#     scipy.spatial.transform.Rotation.from_quat
#     """
#     if debug: # sets T to a small random rotation and 0 translation
#         # sample small euler angles to generate a small perturbed rotation matrix
#         rpy = np.random.normal(0, 0.5, size=(3,1))
#         R = dcm_from_rpy(rpy)
#         # t = (np.array([[0,0,2]]).T + np.random.normal(loc=0, scale=0.5, size=(3,1))).flatten()

#         # print(R, t)
#         T = np.zeros((4,4))
#         T[0:3,0:3] = R
#         # T[0:3,3] = t
#         T[3,3] = 1
#         # T = np.eye(4)
#     return T # change later

# print(get_T_glob_loc_vicon(debug=True))

def get_T_C_global_vicon(april_tag_img_file: Path, detector: Detector, cam_intrinsics: np.ndarray, distortion_coeffs: np.ndarray, T_1_Ai = None):
    """
    Estimates the pose of the camera relative to the global vicon frame.
    april_tag_img_files: list of april tag image paths.
    detector: april tag detector object.
    cam_intrinsics: 3x3 numpy array of the camera intrinsics matrix
    distortion_coeffs: 5x1 camera distortion coefficients.
    T_1_Ai: 4x4 numpy array of the transformation from the tag frame to the base frame. NOTE: CURRENTLY NOT USED AT ALL.
    Returns:
    T_1_C: 4x4 numpy array of the transformation from the camera frame to the base frame.
    """

    # from pose_estimation import estimate_camera_pose
    image = cv2.imread(april_tag_img_file)#.absolute().as_posix())
    print(image.shape)  

    # Needs to be in grayscale for the apriltag detector
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the image
    tags = detector.detect(gray) 
    print(tags)
    print(type(tags))
    tag = tags[0] # NOTE: check if tags is an iterable or single instance

    # get tag pixel cords
    tag_corners_px = tag.corners
    print(tag_corners_px, tag_corners_px.shape)
    # get tag mm cords
    tag_corners_mm_Ai = iba.get_corner_config(tag_sizes=[130], tag_config="single", use_center=False)

    # solves transformation using PnP
    T_1_C = pe.estimate_T_Ci_Ai(
                tag_corners_px=tag_corners_px, 
                tag_corners_mm_Ai=tag_corners_mm_Ai,
                camera_matrix=cam_intrinsics, 
                dist_coeffs=distortion_coeffs, 
                )
    
    # print(T_1_C)
        
    return T_1_C # results seems reasonable

def get_T_local_vicon_C(april_tag_img_files, T_world_loc_vicon,  detector: Detector, cam_intrinsics: np.ndarray, distortion_coeffs: np.ndarray, T_1_Ai = None):
    """
    Estimates the pose of the camera relative to the local vicon frame.
    april_tag_img_files: list of april tag image paths.
    T_world_loc_vicon: list of world to local vicon transformation matrices corresponding to the time of the april tag img files.
    detector: april tag detector object.
    cam_intrinsics: 3x3 numpy array of the camera intrinsics matrix
    distortion_coeffs: 5x1 camera distortion coefficients.
    T_1_Ai: 4x4 numpy array of the transformation from the tag frame to the base frame. NOTE: CURRENTLY NOT USED AT ALL.
    Returns:
    T_1_C: 4x4 numpy array of the transformation from the camera frame to the base frame.
    """
    num_imgs = len(april_tag_img_files) # assumes 1 tag per image
    quats = np.empty((num_imgs, 4)) # for averaging over transforms
    translations = np.empty((num_imgs, 3))
    for i, (april_tag_img_file, T_world_loc_vicon_i) in enumerate(zip(april_tag_img_files, T_world_loc_vicon)):
        print(type(april_tag_img_file), april_tag_img_file, april_tag_img_files)
        T_C_world_i = get_T_C_global_vicon(april_tag_img_file, detector, cam_intrinsics, distortion_coeffs, T_1_Ai = None)
        # T_world_loc_vicon_i = get_T_glob_loc_vicon(debug=True)

        T_C_local_vicon_i = T_C_world_i @ T_world_loc_vicon_i # compute T_C_loc estimate for i-th image
        print(T_C_world_i, T_world_loc_vicon_i)
        R = Rotation.from_matrix(T_C_local_vicon_i[0:3, 0:3]) # scipy.spatial.transform.Rotation type
        t = T_C_local_vicon_i[0:3,3]

        quat = R.as_quat() # get quat rep of R, # NOTE dont know what the return type is
        assert type(quat) == np.ndarray, f"got {type(quat)}"
        quats[i,:] = quat
        translations[i,:] = t
    
    # averaged transformation
    t = np.average(translations, axis=0)
    quat = np.average(quats, axis=0) 
    quat /= np.sqrt(quat @ quat) # renormalization step

    R = Rotation.from_quat(quat=quat).as_matrix()
    T = np.zeros((4,4))
    T[0:3, 0:3] = R
    T[0:3,3] = t
    T[3,3] = 1
    T_loc_vicon_C = np.inv(T) 
    return T_loc_vicon_C

class ViconSubscriber:
    # NOTE: Dont think this class will be used
    def __init__(self):
        # Initialize the node
        rospy.init_node('vicon_subscriber', anonymous=True)
        
        # Member to store the current transform
        self.current_vicon_transform = None
        self.last_vicon_update_time = rospy.get_time()
        
        # Subscribe to the Vicon topic
        self.vicon_sub = rospy.Subscriber('/vicon/ROB498_Drone/ROB498_Drone', TransformStamped, self.vicon_cb)
        
    def vicon_cb(self, msg):
        """Callback function to process data received from the Vicon system."""
        self.current_vicon_transform = msg
        self.last_vicon_update_time = rospy.get_time()
        rospy.loginfo("Vicon Transform Updated")

    def run(self):
        # Keep the node running
        rospy.spin()

    
# No clue what this does, from ChatGPT
class ExtCalNode():
    def __init__(self):
        self.vicon_sub = rospy.Subscriber('/vicon/ROB498_Drone/ROB498_Drone', TransformStamped, callback=self.vicon_cb)

def transform_to_pq(msg):
    """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
    q = np.array([msg.rotation.x, msg.rotation.y,
                  msg.rotation.z, msg.rotation.w])
    return p, q

def transform_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/TransformStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return transform_to_pq(msg.transform)

if __name__ == '__main__':
    vicon_subscriber = ViconSubscriber()
    vicon_subscriber.run()
    while vicon_subscriber.current_vicon_transform:
        continue
    T_loc_glob_msg = vicon_subscriber.current_vicon_transform # NOTE: is this the correct name for this?
    p, q = transform_stamped_to_pq(T_loc_glob_msg)
    R = Rotation.from_quat(q).as_matrix()
    T_loc_glob = np.zeros((4,4))
    T_loc_glob[0:3, 0:3] = R
    T_loc_glob[0:3, 3] = p
    T_loc_glob[3, 3] = 1
    
    T = get_T_local_vicon_C(5*["./Extrinsic-Cam-Calibration/AprilTag-Bundle-Adjustment/examples/testimages/image.jpg"], detector=detector, cam_intrinsics=cam_intrinsics, distortion_coeffs=distortion_coeffs)
    print(T)
    # Extrinsic-Cam-Calibration\AprilTag-Bundle-Adjustment\examples\testimages\image.png