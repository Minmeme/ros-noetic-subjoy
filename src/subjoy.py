#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import roslibpy
import time

# Initialize WebSocket client
class WebSocketClient:
    def __init__(self, host='0.0.0.0', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)
        self.websocket_topic = None
        self.connect()

    def connect(self):
        try:
            self.client.run()
            self.websocket_topic = roslibpy.Topic(self.client, '/websocket_joy', 'std_msgs/String', queue_size=10)
            rospy.loginfo("WebSocket connected.")
        except Exception as e:
            rospy.logwarn(f"Failed to connect to WebSocket: {e}")
            time.sleep(5)  # Wait before retrying
            self.connect()

    def publish(self, message):
        if self.client.is_connected:
            try:
                self.websocket_topic.publish(roslibpy.Message({'data': str(message)}))  # Convert message to string
                rospy.loginfo(f"Data sent to WebSocket: {message}")
            except Exception as e:
                rospy.logwarn(f"Failed to send message to WebSocket: {e}")
        else:
            rospy.logwarn('WebSocket is not connected, attempting to reconnect...')
            self.connect()


class JoyReceiver:
    def __init__(self):
        rospy.init_node('joy_receiver', anonymous=True)
        self.last_data = None
        self.websocket_client = WebSocketClient()
        rospy.Subscriber('/joy', Joy, self.joy_callback)

    def joy_callback(self, data):
        input_id = None
        try:
            if data.buttons[0] == 1:
                message, input_id = 'A', 0
            elif data.buttons[1] == 1:
                message, input_id = 'B', 1
            elif data.buttons[2] == 1:
                message, input_id = 'X', 2
            elif data.buttons[3] == 1:
                message, input_id = 'Y', 3
            elif data.buttons[4] == 1:
                message, input_id = 'LB', 4
            elif data.buttons[5] == 1:
                message, input_id = 'RB', 5
            elif data.buttons[6] == 1:
                message, input_id = 'Back', 6
            elif data.buttons[7] == 1:
                message, input_id = 'Start', 7
            elif data.buttons[8] == 1:
                message, input_id = 'Home', 8
            elif data.buttons[9] == 1:
                message, input_id = 'L_pop', 9
            elif data.buttons[10] == 1:
                message, input_id = 'R_pop', 10
            elif data.axes[0] > 0:
                message, input_id = 'AL-L', 11
            elif data.axes[0] < 0:
                message, input_id = 'AL-R', 12
            elif data.axes[1] > 0:
                message, input_id = 'AL-U', 13
            elif data.axes[1] < 0:
                message, input_id = 'AL-D', 14
            elif data.axes[3] > 0:
                message, input_id = 'AR-L', 15
            elif data.axes[3] < 0:
                message, input_id = 'AR-R', 16
            elif data.axes[4] > 0:
                message, input_id = 'AR-U', 17
            elif data.axes[4] < 0:
                message, input_id = 'AR-D', 18
            elif data.axes[2] < 1.0:
                message, input_id = 'LT', 19
            elif data.axes[5] < 1.0:
                message, input_id = 'RT', 20
            elif data.axes[6] == 1.0:
                message, input_id = 'Hat_L', 21
            elif data.axes[6] == -1.0:
                message, input_id = 'Hat_R', 22
            elif data.axes[7] == 1.0:
                message, input_id = 'Hat_Up', 23
            elif data.axes[7] == -1.0:
                message, input_id = 'Hat_Down', 24
            else:
                message, input_id = 'Stop', 30

            # Publish only if input ID changes
            if input_id != self.last_data:
                rospy.loginfo(f"Publishing: {message} (ID: {input_id})")
                self.last_data = input_id
                self.websocket_client.publish(input_id)  # Send as string

        except IndexError as e:
            rospy.logwarn(f"Index error in joystick input: {e}")
        except Exception as e:
            rospy.logwarn(f"Unexpected error: {e}")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        JoyReceiver().run()
    except rospy.ROSInterruptException:
        pass
