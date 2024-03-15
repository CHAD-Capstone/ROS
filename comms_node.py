#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_srvs.srv import EmptyResponse, Empty
import threading


class DroneComms:
    def __init__(self, group_number: int, takeoff_altitude: float = 2.0, pose_update_rate: float = 20.0):
        node_name = f'rob498_drone_{group_number:02d}'
        rospy.init_node(node_name) 

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.current_state = State()
        state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)

        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Service topics
        self.srv_ping = rospy.Service(node_name + '/comm/ping', Empty, self.callback_ping)
        self.srv_launch = rospy.Service(node_name + '/comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service(node_name + '/comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service(node_name + '/comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service(node_name + '/comm/abort', Empty, self.callback_abort)

        # Object state
        self.takeoff_altitude = takeoff_altitude
        self.pose_update_rate = pose_update_rate

        self.current_requested_position = None
        self.should_run_pose_thread = True
        self.offboard_thread = None

        # Wait for startup
        rate = rospy.Rate(20.0)
        while(not rospy.is_shutdown() and not self.current_state.connected):
            rate.sleep()
        rospy.loginfo('Drone is connected')

        rospy.spin()

    ### Data managers
    def get_pose(self):
        """
        Uses the current state to get pose in the local frame
        """
        raise NotImplementedError('This method is not implemented yet')

    ### Callbacks
    def state_cb(self, msg):
        """
        Callback for the state subscriber
        """
        self.current_state = msg

    ### Service callbacks
    def callback_ping(self, request):
        rospy.loginfo("Got ping!")
        return EmptyResponse()

    def callback_launch(self, request):
        self.handle_launch()
        return EmptyResponse()

    def callback_test(self, request):
        self.handle_test()
        return EmptyResponse()

    def callback_land(self, request):
        self.handle_land()
        return EmptyResponse()

    def callback_abort(self, request):
        self.handle_abort()
        return EmptyResponse()
        
    ### Handlers
    def handle_launch(self):
        print('Launch Requested. Your drone should take off.')
        self.arm_and_takeoff(self.takeoff_altitude)

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

    def handle_land(self):
        print('Land Requested. Your drone should land.')
        self.land()

    def handle_abort(self):
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        self.abort()

    ### Commands
    def arm_and_takeoff(self, targetAltitude):
        """
        Sends an arm service call to the drone and then takes off to the target altitude
        """
        self.move_to(0, 0, targetAltitude)
        rospy.sleep(1)
        offboard = self.start_offboard_mode()
        if not offboard:
            rospy.logerr('Drone failed to enter offboard mode')
            self.cancel_move()
            return False
        rospy.loginfo("Set position")
        rospy.sleep(2)  # Sleep to allow for some position messages to be sent
        armed = self.arm()
        if not armed:
            rospy.logerr('Drone failed to arm')
            self.cancel_move()
            return False
        return True

    def abort(self):
        """
        Aborts the current operation and lands the drone
        """
        self.disarm()
        self.stop_offboard_mode()

    def start_offboard_mode(self):
        """

        """
        mode_set = self.set_mode("OFFBOARD")
        # mode_set = self.set_mode("POSITION")
        if not mode_set:
            rospy.logerr("Failed to start offboard control")
            return False
        self.should_run_pose_thread = True
        self.offboard_thread = threading.Thread(target=self.publish_position_loop)
        self.offboard_thread.start()
        return True

    def publish_position_loop(self):
        rate = rospy.Rate(self.pose_update_rate)
        
        while self.should_run_pose_thread and not rospy.is_shutdown():
            if self.current_requested_position is not None:
                rospy.loginfo("Publishing pose")
                pose = self.current_requested_position
                pose.header.stamp = rospy.Time.now()
                self.local_pos_pub.publish(pose)
            else:
                rospy.logdebug('No position requested')
            rate.sleep()

    def stop_offboard_mode(self):
        self.cancel_move()
        self.should_run_pose_thread = False
        if self.offboard_thread is not None:
            self.offboard_thread.join()

    def set_mode(self, custom_mode):
        """
        Changes the mode of the drone
        """
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = custom_mode
        mode_set = self.set_mode_client.call(offb_set_mode)
        if not mode_set:
            rospy.logerr(f'Failed to enter {custom_mode} mode')
            return False
        return True

    def land(self):
        """
        Lands the drone
        """
        self.stop_offboard_mode()
        landing = self.set_mode("AUTO.LAND")
        if not landing:
            rospy.logerr('Drone failed to land')
            return False
        return True

    def arm(self):
        """
        Arms the drone
        """
        # Check if the drone is in offboard mode
        rospy.loginfo(f"Current Mode: {self.current_state.mode}")
        if self.current_state.mode != "OFFBOARD":
            rospy.logerr("CANNOT ARM! NOT IN OFFBOARD MODE!")
            return False
        arm_request = CommandBoolRequest()
        arm_request.value = True
        return self.arming_client.call(arm_request).success

    def disarm(self):
        """
        Disarms the drone
        """
        arm_request = CommandBoolRequest()
        arm_request.value = False
        return self.arming_client.call(arm_request).success

    def cancel_move(self):
        """
        Cancels the current move request
        """
        self.current_requested_position = None

    def move_to(self, x, y, z, qx = 0, qy = 0, qz = 0, qw = 1):
        """
        Moves the drone to the specified position
        """
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.current_requested_position = pose


# Main communication node for ground control
def comm_node():
    # print('This is a dummy drone node to test communication with the ground control')
    # print('node_name should be rob498_drone_TeamID. Service topics should follow the name convention below')
    # print('The TAs will test these service calls prior to flight')
    # print('Your own code should be integrated into this node')

    # Your code goes below
    drone_comms = DroneComms(6, takeoff_altitude=1.5)

if __name__ == "__main__":
    comm_node()
