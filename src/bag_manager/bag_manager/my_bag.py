#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage, Image
from rosbag2 import Rosbag2
from rclpy.duration import Duration
from queue import Queue, Empty
import threading
import sys
from rclpy.clock import Clock, ClockType

class CompressedImageRecorder(Node):
    def __init__(self):
        super().__init__('compressed_image_recorder')
        self._write_queue = Queue()
        self._paused = False
        self._stop_condition = threading.Condition()
        self._stop_flag = False
        self._bag_lock = threading.Lock()
        filename = "/tmp/data"
        self.topics = ["/camera/image_raw/compressed"]
        self._serialization_format = 'cdr'
        self._storage_id = 'sqlite3'
        self._storage_options = rosbag2_py.StorageOptions(
            uri=filename, storage_id=self._storage_id)
        self._converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=self._serialization_format,
            output_serialization_format=self._serialization_format)
        self.rosbag_writer = rosbag2_py.SequentialWriter()
        self.rosbag_writer.open(self._storage_options, self._converter_options)
        self._bag = self._create_initial_rosbag(filename, self.topics)
        self._write_thread = threading.Thread(target=self._run_write)
        self.timer = self.create_timer(5, self.stop_request)
        self.init_subscribers()
        self._write_thread.start()
        
    def stop_request(self):
        self.timer.cancel()
        self._stop_flag = True

    def _run_write(self):
        try:
            
            poll_interval = 1.0
            while not self._stop_flag:
                try:
                    item = self._write_queue.get(block=False, timeout=poll_interval)
                except Empty:
                    continue

                if item == self:
                    continue

                # Write the next message to the bag
                topic, msg, msg_type_name, t = item
                with self._bag_lock:
                    self.rosbag_writer.write(topic, serialize_message(msg), t.nanoseconds)
                    self.get_logger().info("write to bag")
                    # Update the overall duration for this bag based on the message just added
                    duration_ns = t.nanoseconds - self._bag.start_time.nanoseconds
                    self._bag.duration = Duration(nanoseconds=duration_ns)

            self.get_logger().info("-------- stop record") 
            self.rosbag_writer.close()

        except Exception as ex:
            print('Error writing to bag: %s' % str(ex), file=sys.stderr)

    def _create_initial_rosbag(self, filename, topics):
        # Create any active topics in the database. Could potentially have multiple types
        # associated with this topic name, but just use the first
        topic_type_map = {}
        for topic, msg_type_names in self.get_topic_names_and_types():
            if topic in topics:
                offered_qos_profiles = ''
                # qos_profiles = get_qos_profiles_for_topic(self._node, topic)
                # if qos_profiles:
                    # offered_qos_profiles = qos_profiles_to_yaml(qos_profiles)
                topic_metadata = rosbag2_py.TopicMetadata(
                    name=topic, type=msg_type_names[0],
                    serialization_format=self._serialization_format,
                    offered_qos_profiles=offered_qos_profiles)
                self.rosbag_writer.create_topic(topic_metadata)
                topic_type_map[topic] = topic_metadata

        return Rosbag2(filename, recording=True, topics=topic_type_map)
    
    def init_subscribers(self):
        # Subscribe to compressed image topic
        self.sub = self.create_subscription(
            CompressedImage,
            self.topics[0],
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        now = Clock(clock_type=ClockType.SYSTEM_TIME).now()
        self._write_queue.put((self.topics[0], msg, "", now))




def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
