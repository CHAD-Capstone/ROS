"""
Subscribes to the VICON topic and transforms into the drone local frame

We have a known transformation matrix from the tracker frame to the drone local frame
We receive the pose in the tracker frame and transform it into the local frame and then publish it on the /mavros/vision_pose/pose
topic to update the drone's position
"""

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, Empty, EmptyResponse
import threading
import tf.transformations as tf_trans

dx = 0.0  # Given in VICON frame. Vector from VICON to local frame
dy = 0.0
dz = 0.0
roll = 0.0  # From VICON to local frame
pitch = 0.0
yaw = 0.0

trans_quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw)

def transform_vicon_to_local(vicon_translation, vicon_quaternion):
    """
    Transforms the VICON pose into the local frame

    Returns a PoseStamped object
    """
    point = Point()
    point.x = vicon_translation.x + dx
    point.y = vicon_translation.y + dy
    point.z = vicon_translation.z + dz

    # Mulitply the quaternions
    vicon_quaternion = [vicon_quaternion.x, vicon_quaternion.y, vicon_quaternion.z, vicon_quaternion.w]
    local_quaternion = tf_trans.quaternion_multiply(vicon_quaternion, trans_quaternion)

    pose = PoseStamped()
    pose.pose.position = point
    pose.pose.orientation = Quaternion(*local_quaternion)

    return pose
    
    

class ViconNode:
    def __init__(self, group_number=6, verbose=False):
        """
        Constructor for the ViconNode class
        """
        ### Initialize the node
        node_name = f'vicon_node_{group_number:02d}'
        rospy.init_node(node_name)

        self.vicon_sub = rospy.Subscriber('/vicon/ROB498_Drone/ROB498_Drone', TransformStamped, callback=self.vicon_cb)

        self.local_pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        """
        Is this the right topic?
        Apparently realsense publishes /mavros/odometry/out
        This person is talking about /mavros/local_position/pose (https://discuss.ardupilot.org/t/vision-position-estimate-not-appearing-in-qgroundcontrol/23978)

        It looks like a few things get shoved together into the visual odometry uORB topics https://docs.px4.io/main/en/ros/external_position_estimation.html#px4-mavlink-integration
        The EKF2 is subscribing to the vehicle_visual_odometry uORB topic so whatever mavros topic we use needs to be published to either 
        VISION_POSITION_ESTIMATE or ODOMETRY.
        """

        self.verbose = verbose

        rospy.spin()

    def vicon_cb(self, msg):
        """
        Callback for the VICON subscriber
        """
        transform = msg.transform
        translation = transform.translation
        rotation = transform.rotation

        if self.verbose:
            # Print out the translation and rotation
            rospy.log(f'VICON Translation: {translation}')
            rospy.log(f'VICON Rotation: {rotation}')

        local_pose = transform_vicon_to_local(translation, rotation)

        if self.verbose:
            rospy.log(f'Local Translation: {local_pose.pose.position}')
            rospy.log(f'Local Rotation: {local_pose.pose.orientation}')

        self.local_pose_pub.publish(local_pose)


if __name__ == '__main__':
    vicon_node = ViconNode()

