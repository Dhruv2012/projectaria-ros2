#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import aria.sdk as aria
import numpy as np
from projectaria_tools.core.sensor_data import ImageDataRecord

import sys

# def main():
#     rclpy.init(args=sys.argv)
#     node = rclpy.create_node("cam_aria")
#     node.get_logger().info('Created dummy node')
#     pub1 = node.create_publisher(String, "/node_test_messages_pub", 10)

#     while rclpy.ok():
#         msg1 = String()
#         msg1.data = "hi"
#         pub1.publish(msg1)

# if __name__ == "__main__":
#     main()

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("cam_aria")
        self.bridge = CvBridge()
        # self.cap = cv2.VideoCapture(0)
        self.pub = self.create_publisher(Image, "/aria", 10)
        # self.rgb8pub = self.create_publisher(Image, "/image/rgb", 10)
        # self.bgr8pub = self.create_publisher(Image, "/image/bgr", 10)
        # self.mono8pub = self.create_publisher(Image, "/image/mono", 10)

        # Ryan's aria streaming
        # Create DeviceClient instance
        device_client = aria.DeviceClient()
        client_config = aria.DeviceClientConfig()
        device_client.set_client_config(client_config)

        # Connect to device
        device = device_client.connect()

        # Get streaming manager
        streaming_manager = device.streaming_manager
        streaming_client = streaming_manager.streaming_client

        # Set config
        streaming_config = aria.StreamingConfig()
        streaming_config.profile_name = "profile18"
        streaming_config.streaming_interface = aria.StreamingInterface.Usb

        # get security certs
        streaming_config.security_options.use_ephemeral_certs = True
        streaming_manager.streaming_config = streaming_config

        # print("STREAM STATE", streaming_manager.streaming_state.value)
        if streaming_manager.streaming_state.value == 3:
            streaming_manager.stop_streaming()
        streaming_manager.start_streaming()


        # config to RGB camera stream
        config = streaming_client.subscription_config
        config.subscriber_data_type = aria.StreamingDataType.Rgb
        streaming_client.subscription_config = config

        class StreamingClientObserver:
            def __init__(self):
                self.rgb_image = None

            def on_image_received(self, image: np.array, record: ImageDataRecord):
                self.rgb_image = image

        # observer subscribe to RGB cam stream
        observer = StreamingClientObserver()
        streaming_client.set_streaming_client_observer(observer)
        streaming_client.subscribe()



        # timer
        self.timer = self.create_timer(0.01, self.timer_callback)

        #
        self.device_client = device_client
        self.streaming_client = streaming_client
        self.streaming_manager = streaming_manager
        self.observer = observer
        self.device = device

    def timer_callback(self):
        # while True:
        #     try:
        #         r, frame = self.cap.read()
        #         if not r:
        #             return
        #         self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        #         # BGR8
        #         self.bgr8pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        #         # RGB8
        #         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #         self.rgb8pub.publish(self.bridge.cv2_to_imgmsg(frame_rgb, "rgb8"))

        #         # MONO8
        #         frame_mono = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #         self.mono8pub.publish(self.bridge.cv2_to_imgmsg(frame_mono, "mono8"))

        #     except CvBridgeError as e:
        #         print(e)
        # self.cap.release()

        if self.observer.rgb_image is not None and rclpy.ok():
            rgb_image = cv2.cvtColor(self.observer.rgb_image, cv2.COLOR_BGR2RGB)
            # cv2.imshow(rgb_window, np.rot90(rgb_image, -1))

            self.observer.rgb_image = None
            self.pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "rgb8"))




        # # Close stream from data and stop streaming
    def destroy_node(self):
        print("Destroying Aria Node")
        self.streaming_client.unsubscribe()
        self.streaming_manager.stop_streaming()
        self.device_client.disconnect(self.device)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    ip = ImagePublisher()
    print("Publishing...")
    rclpy.spin(ip)
    
    print("reached here")
    ip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()