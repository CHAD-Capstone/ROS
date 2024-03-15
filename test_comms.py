#!/usr/bin/python3

import rospy
from std_srvs.srv import Empty

def call_service(service_name):
    rospy.wait_for_service(service_name)
    try:
        service = rospy.ServiceProxy(service_name, Empty)
        resp = service()
        print(f"Service {service_name} called successfully")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def main():
    rospy.init_node('drone_test_script')
    group_number = input("Enter the drone group number: ")
    base_service_name = f"rob498_drone_{int(group_number):02d}/comm/"

    print("Press 'l' to launch, 't' to test, 'a' to abort, 'd' to land, or 'q' to quit.")

    while not rospy.is_shutdown():
        key = input("Enter command: ")
        if key.lower() == 'l':
            call_service(base_service_name + "launch")
        elif key.lower() == 't':
            call_service(base_service_name + "test")
        elif key.lower() == 'a':
            call_service(base_service_name + "abort")
        elif key.lower() == 'd':
            call_service(base_service_name + "land")
        elif key.lower() == 'p':
            call_service(base_service_name + "ping")
        elif key.lower() == 'q':
            break
        else:
            print("Invalid input. Please try again.")

if __name__ == '__main__':
    main()