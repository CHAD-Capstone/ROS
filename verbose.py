"""
A node that configurably prints out a bunch of information about the drone's state
"""

import rospy
from mavros_msgs.msg import State, Odometry

class VerboseNode:
    def __init__(
        self,
        group_number: int,
        print_pose: bool = True,
        print_rate: float = 1.0,
    ):
        """
        Constructor for the VerboseNode class
        """
        node_name = f'rob498_verbose_{group_number:02d}'
        rospy.init_node(node_name)

        self.current_state = (-1, State())
        state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_cb)

        self.current_odometry = (-1, Odometry())
        odometry_sub = rospy.Subscriber("mavros/odometry/in", Odometry, callback=self.odometry_cb)
        """
        Is this the right topic?
        """

        self.print_pose = print_pose
        self.print_rate = print_rate

        self.start_printing()

    def state_cb(self, msg):
        """
        Callback for the state subscriber
        """
        current_time_sec = rospy.get_time().to_sec()
        self.current_state = (current_time_sec, msg)

    def odometry_cb(self, msg):
        """
        Callback for the odometry subscriber
        """
        current_time_sec = rospy.get_time().to_sec()
        self.current_odometry = (current_time_sec, msg)

    def get_pose() -> Tuple[float, Odometry]:
        """
        Uses the current odometry to get pose in the local frame in ENU coordinates
        """
        current_time_sec = rospy.get_time().to_sec()
        update_time_sec, odometry = self.current_odometry
        return (current_time_sec - update_time_sec, odometry)

    def start_printing(self):
        """
        Starts the printing thread
        """
        rate = rospy.Rate(self.print_rate)
        while not rospy.is_shutdown():
            if self.print_pose:
                update_time_sec, odometry = self.get_pose()
                print(f'Time since pose update: {update_time_sec}')
                print(f'Odometry: {odometry}')
            rate.sleep()
