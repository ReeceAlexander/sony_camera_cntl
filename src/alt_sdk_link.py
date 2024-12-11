#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import pexpect
import threading

class RemoteCliController:
    def __init__(self):
        rospy.init_node('sony_sdk_link_node')

        # Start the RemoteCli using pexpect
        self.child = pexpect.spawn('/home/reece/Downloads/CrSDK_v1.13.00_20241016a_Linux64PC/build/RemoteCli', encoding='utf-8')
        self.child.logfile = open('/tmp/RemoteCli.log', 'w')  # Log to file
        self.child.timeout = None

        # Start a thread to read output
        self.output_thread = threading.Thread(target=self.read_output)
        self.output_thread.daemon = True
        self.output_thread.start()

        # Subscribe to the topic
        rospy.Subscriber('/camera_cntl', String, self.command_callback, queue_size=1)

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

    def read_output(self):
        while True:
            try:
                line = self.child.readline()
                if line:
                    rospy.loginfo(line.strip())
            except Exception as e:
                rospy.logerr(f"Error reading output: {e}")
                break

    def send_sequence(self, sequence):
        for cmd in sequence:
            rospy.loginfo(f"Sending command: {cmd}")
            self.child.sendline(cmd)
            rospy.sleep(1)  # Adjust sleep as needed

    def command_callback(self, msg):
        command = msg.data
        rospy.loginfo(f"Received command: {command}")

        if command == "connect":
            # Send initial connection commands
            self.send_sequence(["1", "1"])

        elif command == "capture":
            # Send ["3", "y"] to trigger capture
            self.send_sequence(["1", "3", "y"])
        else:
            rospy.logwarn(f"Unknown command: {command}")

    def shutdown(self):
        rospy.loginfo("Shutting down Remote CLI controller")
        if self.child:
            self.child.terminate()

def main():
    try:
        cli_controller = RemoteCliController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
