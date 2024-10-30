import argparse
import sys

import aria.sdk as aria

from visualizer import AriaVisualizer, AriaVisualizerStreamingClientObserver

import cv2
import numpy as np

from aria_utils import ctrl_c_handler, quit_keypress, update_iptables
from projectaria_tools.core.sensor_data import ImageDataRecord

parser = argparse.ArgumentParser()
parser.add_argument(
        "--update_iptables",
        default=False,
        action="store_true",
        help="Update iptables to enable receiving the data stream, only for Linux.",
    )
parser.add_argument(
        "--profile",
        dest="profile_name",
        type=str,
        default="profile18", #TBD
        required=False,
        help="Profile to be used for streaming.",
    )

def main(args):
    if args.update_iptables and sys.platform.startswith("linux"):
        update_iptables()

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
    streaming_config.profile_name = args.profile_name
    streaming_config.streaming_interface = aria.StreamingInterface.Usb

    # get security certs
    streaming_config.security_options.use_ephemeral_certs = True
    streaming_manager.streaming_config = streaming_config

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

    rgb_window = "Aria RGB"

    cv2.namedWindow(rgb_window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(rgb_window, 512, 512)
    cv2.setWindowProperty(rgb_window, cv2.WND_PROP_TOPMOST, 1)
    cv2.moveWindow(rgb_window, 50, 50)

    with ctrl_c_handler() as ctrl_c:
        while not (quit_keypress() or ctrl_c):
            if observer.rgb_image is not None:
                rgb_image = cv2.cvtColor(observer.rgb_image, cv2.COLOR_BGR2RGB)
                cv2.imshow(rgb_window, np.rot90(rgb_image, -1))

                observer.rgb_image = None
    
    # Close stream from data and stop streaming
    print("Stop listening to image data")
    streaming_client.unsubscribe()
    streaming_manager.stop_streaming()
    device_client.disconnect(device)

if __name__ == "__main__":
    args = parser.parse_args()
    main(args)
