#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2

TOPIC_IMAGE_COMPRESSED = "image_compressed"
TOPIC_IMG = "/camera/image_raw"

class ImageCompressorNode(LifecycleNode):
    def __init__(self):
        super().__init__('image_compressor')

        self.bridge = CvBridge()
        self.subscriber = None
        self.publisher = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        self.publisher = self.create_publisher(
            CompressedImage,
            TOPIC_IMAGE_COMPRESSED,
            10
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        # Activate publisher

        # Subscribe to original image
        self.subscriber = self.create_subscription(
            Image,
            TOPIC_IMG,
            self.image_callback,
            10
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        # Cleanup subscription
        if self.subscriber is not None:
            self.destroy_subscription(self.subscriber)
            self.subscriber = None

        # Deactivate publisher
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
            self.publisher = None
        return TransitionCallbackReturn.SUCCESS

    def image_callback(self, msg: Image):
        if self.publisher is None or not self.publisher.is_activated:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, encoded_image = cv2.imencode('.png', cv_image)
            if success:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'png'
                compressed_msg.data = encoded_image.tobytes()
                self.publisher.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to compress image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressorNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
