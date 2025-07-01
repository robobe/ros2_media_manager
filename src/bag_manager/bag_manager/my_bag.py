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
from typing import Any
import time
from rclpy.clock import Clock, ClockType
from qos import gen_subscriber_qos_profile, get_qos_profiles_for_topic, qos_profiles_to_yaml
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Trigger
from media_manager_interfaces.srv import GetMediaFileList, SetMediaFile
from pathlib import Path
import shutil

INTERVAL_CHECK = 1.0

SRV_START_STOP = "start_stop"
SRV_REMOVE_MEDIA = "remove_media"
SRV_REMOVE_ALL = "remove_all_media"
SRV_GET_MEDIA_LIST = "get_media_list"
SRV_SET_MEDIA_NAME = "set_media_name"

PARAM_MEDIA_LOCATION = "media_location"


class _SubscriberHelper(object):
    """
    Helper class for topic subscription
    """
    def __init__(self, node: Node, recorder: 'Recorder', topic: str, msg_type_name: str):
        self.recorder: 'Recorder' = recorder
        self.topic = topic
        self.msg_type_name = msg_type_name

        # Get all of the QoS profiles for this topic
        qos_profiles = get_qos_profiles_for_topic(node, self.topic)
        if qos_profiles:
            # Select one of them to use for the subscription
            self.qos_profile = gen_subscriber_qos_profile(qos_profiles)
            self.subscriber = node.create_subscription(
                get_message(msg_type_name), 
                topic, 
                self.callback, 
                self.qos_profile)
            node.get_logger().info(f"subscribe topic: {topic}")

    def callback(self, msg: Any) -> None:
        self.recorder._record(self.topic, msg, self.msg_type_name)

NODE_NAME = "my_bag_record"

class Recorder(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self._write_queue = Queue()
        self._paused = False
        self._stop_condition = threading.Condition()
        self._stop_flag = True
        self._message_count = {}  # topic -> int (track number of messages recorded on each topic)
        self._bag_lock = threading.Lock()
        self._subscriber_helpers = {} # hold all subscriber that all ready register
        self._limited_topics = set()
        self._failed_topics = set()
        self._last_update = time.time()
        self._init_parameters()
        self._init_service()

        self.topics = ["/camera/image_raw/compressed", "/xxx"]
        
        
        self.get_logger().info("start bag manager")


    def _get_full_topic_name(self, topic):
        topic_name = f'/{self.get_name()}/{topic}'
        return topic_name

    def _init_parameters(self):
        self.declare_parameter(PARAM_MEDIA_LOCATION, "/tmp/data")

    def _init_service(self):
        """
        init media manage service
        start/stop record
        remove media
        remove all
        get media list
        """
        self.set_media_name = self.create_service(
            SetMediaFile,
            self._get_full_topic_name(SRV_SET_MEDIA_NAME),
            self.set_media_name_callback
        )

        self.remove_all = self.create_service(
            Trigger,
            self._get_full_topic_name(SRV_REMOVE_ALL),
            self.remove_all_callback
        )

        self.get_all_media_service = self.create_service(
            GetMediaFileList,
            self._get_full_topic_name(SRV_GET_MEDIA_LIST),
            self.get_all_media_callback
        )

        self.start_record_service = self.create_service(
            Trigger,
            self._get_full_topic_name(SRV_START_STOP),
            self.start_stop_callback
        )
    # region service
    # 
    def start_stop_callback(self, request: Trigger.Request, response: Trigger.Response):
        # Implement your start record logic here
        msg = "error start / stop video record"
        if self._stop_flag:
            self._run_master_check()
            self._topics_validation()
            self._write_thread = threading.Thread(target=self._run_write, daemon=True, name="storage_worker")
            self._write_thread.start()
            msg = "START"
            self._stop_flag = False
        else:
            # stop record
            self._stop_flag = True
            msg = "STOP"

        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    def set_media_name_callback(self, request: SetMediaFile.Request, response: SetMediaFile.Response):
        """
        TODO: create logic method
        """
        media_path = Path(self.get_parameter(PARAM_MEDIA_LOCATION).get_parameter_value().string_value)
        filename = request.name

        self._current_file_path = media_path / filename
        if self._current_file_path.exists():
            response.success = False
            response.message = f"Media {filename} exists"
            return response
        
        
        try:
            self._serialization_format = 'cdr'
            self._storage_id = 'sqlite3'
            self._storage_options = rosbag2_py.StorageOptions(
                uri=self._current_file_path.as_posix(), storage_id=self._storage_id)
            self._converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format=self._serialization_format,
                output_serialization_format=self._serialization_format)
            self.rosbag_writer = rosbag2_py.SequentialWriter()
            self.rosbag_writer.open(self._storage_options, self._converter_options)
            self._bag = self._create_initial_rosbag(self._current_file_path.as_posix(), self.topics)
            
            response.message = ""
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to create bag storage: {self._current_file_path.as_posix()}")
            self.get_logger().error(str(e))
            response.message = "Failed to create bag storage"
            response.success = False
        
        return response

    def remove_all_callback(self, request, response):
        self.get_logger().info("remove all start")
        media_path = Path(self.get_parameter(PARAM_MEDIA_LOCATION).get_parameter_value().string_value)
        removed_files = []
        failed_files = []
        self.get_logger().info(f"remove all: {media_path.as_posix()}")
        for item in media_path.iterdir():
            try:
                shutil.rmtree(item)
                removed_files.append(item.name)
            except Exception as e:
                failed_files.append(f"{item.name}: {str(e)}")

        if failed_files:
            response.success = False
            response.message = f"Failed to remove: {', '.join(failed_files)}"
        else:
            response.success = True
            response.message = f"Removed files: {', '.join(removed_files)}" if removed_files else "No bag files found"
        return response

    def get_all_media_callback(self, request:GetMediaFileList.Request, response: GetMediaFileList.Response):
        media_path = Path(self.get_parameter(PARAM_MEDIA_LOCATION).get_parameter_value().string_value)
        bag_folders = [f.name for f in media_path.glob("*") if f.is_dir()]
        response.file_list = bag_folders
        return response
    # endregion

    def _topics_validation(self):
        """
        Check if request topic has subscribers
        TODO: what to do if no subscriber at all
        """
        subscribes = len(self._subscriber_helpers)
        if  subscribes == 0:
            self.get_logger().error("Bag not subscriber to any topic")

        if subscribes != len(self.topics):
            self.get_logger().warning("Not all request topic has subscribers")
            request = set(self.topics)
            subscribe = set(self._subscriber_helpers.keys())
            not_subscribe = request - subscribe
            self.get_logger().warning(f"{not_subscribe}")



    

    def _run_master_check(self):
        """
        TODO: run by thread as a loop
        This Method check for new topic and subscribe it if exits in topics list 
        and not subscribe yet


        """
        try:
            
            for topic, msg_type_names in self.get_topic_names_and_types():
                
                # Check if:
                #    the topic is already subscribed to, or
                #    we've failed to subscribe to it already, or
                #    we've already reached the message limit, or
                #    we don't want to subscribe
                for msg_type_name in msg_type_names:
                    if topic in self._subscriber_helpers or \
                            topic in self._failed_topics or \
                            topic in self._limited_topics or \
                            not self._should_subscribe_to(topic):
                        continue
                    try:
                        self._message_count[topic] = 0
                        self._subscriber_helpers[topic] = _SubscriberHelper(
                            self, self, topic, msg_type_name)
                    except Exception as ex:
                        self.get_logger().error(f"Error subscribing to {topic}")

        except Exception as ex:
            self.get_logger().error(f"Error recording to bag: {ex} ")


    def _run_write(self):
        """
        Write data to bag
        all the message put in Queue, each message in heat slot
        the loop get item and write it to the bag
        """
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
                    # Update the overall duration for this bag based on the message just added
                    duration_ns = t.nanoseconds - self._bag.start_time.nanoseconds
                    self._bag.duration = Duration(nanoseconds=duration_ns)

            self.get_logger().info("-------- stop record") 
            self.rosbag_writer.close()

        except Exception as ex:
            self.get_logger().error('Error writing to bag: {ex}')

    def _should_subscribe_to(self, topic):
        return topic in self.topics

    
    def _create_initial_rosbag(self, filename, topics):
        # Create any active topics in the database. 
        # Could potentially have multiple types
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
    
    def _record(self, topic: str, msg: Any, msg_type_name: str)-> None:
        """
        Put message data in queue
        Other worker thread save the queue item in the bag
        """
        if self._paused:
            return
        
        now = Clock(clock_type=ClockType.SYSTEM_TIME).now()
        self._write_queue.put((topic, msg, msg_type_name, now))
        # count number of item for each topic
        self._message_count[topic] += 1




def main(args=None):
    rclpy.init(args=args)
    node = Recorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

