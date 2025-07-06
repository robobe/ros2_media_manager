#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2

TOPIC_IMAGE_COMPRESSED = "image_compressed"
TOPIC_IMG = "/camera/image_raw"

class ImageCompressorNode(Node):
    def __init__(self):
        super().__init__('image_compressor')

        self.bridge = CvBridge()
        self.subscriber = None
        self.publisher = None
        self.publisher = self.create_publisher(
            CompressedImage,
            TOPIC_IMAGE_COMPRESSED,
            10
        )
        self.subscriber = self.create_subscription(
            Image,
            TOPIC_IMG,
            self.image_callback,
            10
        )

    
        

    

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            success, encoded_image = cv2.imencode('.png', cv_image)
            if success:
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = 'png'
                compressed_msg.data = encoded_image.tobytes()
                self.publisher.publish(compressed_msg)
                print("--")
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
