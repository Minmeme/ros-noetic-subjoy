#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoyReceiverFromWebSocket:
    def __init__(self):
        rospy.init_node('joy_receiver', anonymous=True)
        self.subscriber = rospy.Subscriber('/websocket_joy', String, self.joy_callback)
        rospy.loginfo("Subscribed to /websocket_joy")

    def joy_callback(self, msg):
        # Process the received message here
        rospy.loginfo(f"Received message: {msg.data}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        JoyReceiverFromWebSocket().run()
    except rospy.ROSInterruptException:
        pass
