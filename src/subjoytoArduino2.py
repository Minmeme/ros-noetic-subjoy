#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32

class JoyReceiverFromWebSocket:
    def __init__(self):
        rospy.init_node('joy_receiver', anonymous=True)
        
        # Subscriber to /websocket_joy topic
        self.subscriber = rospy.Subscriber('/websocket_joy', String, self.joy_callback)
        rospy.loginfo("Subscribed to /websocket_joy")
        
        # Publisher for Int32 messages
        self.publisher = rospy.Publisher('/processed_joy', Int32, queue_size=10)
        rospy.loginfo("Publisher for /processed_joy initialized")

    def joy_callback(self, msg):
        # Process the received message here
        rospy.loginfo(f"Received message: {msg.data}")
        
        # Example: Convert the string data to an integer and publish
        try:
            processed_value = int(msg.data)
            self.publisher.publish(processed_value)
            rospy.loginfo(f"Published processed value: {processed_value}")
        except ValueError:
            rospy.logerr(f"Invalid data received: {msg.data} is not an integer")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        JoyReceiverFromWebSocket().run()
    except rospy.ROSInterruptException:
        pass
