#!/usr/bin/env python3

import rospy
from sony_camera_control.msg import GoToInfo
from std_msgs.msg import String

pub = rospy.Publisher('/camera_cntl', String, queue_size=1)

def callback(msg):
    # Extract the integer part of the current_pos
    current_integer_part = int(msg.current_pos)

    # Check if the integer part has changed
    if current_integer_part != previous_integer_part:
        # Publish 'capture' message
        pub.publish("capture")
        
        # Update previous_integer_part
        previous_integer_part = current_integer_part

def main():
    rospy.init_node('sdk_crawler_link_node')

    rospy.Subscriber('/goto_status', GoToInfo, callback, queue_size=1)
    
    rospy.spin()

if __name__ == "__main__":
    main()
