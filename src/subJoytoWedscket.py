#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import roslibpy

# Initialize ROS and WebSocket
rospy.init_node('joy_to_websocket', anonymous=True)
client = roslibpy.Ros(host='10.10.89.165', port=9090)  # Adjust WebSocket address
client.run()

# Define a ROS topic on the WebSocket
websocket_topic = roslibpy.Topic(client, '/websocket_joy', 'std_msgs/String')

# Callback function to process joystick data
def joy_callback(data):
    if client.is_connected:
        # Convert joystick data to a string message
        message = {
            'axes': list(data.axes),
            'buttons': list(data.buttons)
        }
        # Publish the message to the WebSocket topic
        websocket_topic.publish(roslibpy.Message({'data': str(message)}))
        rospy.loginfo('Data sent to WebSocket: %s', message)
    else:
        rospy.logwarn('WebSocket is not connected')

# Subscribe to the joystick topic
rospy.Subscriber('/joy', Joy, joy_callback)

# Keep the ROS node running
try:
    rospy.spin()
finally:
    # Close the WebSocket topic and client on exit
    websocket_topic.unadvertise()
    client.terminate()
