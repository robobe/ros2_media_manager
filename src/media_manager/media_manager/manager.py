#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from media_manager_interfaces.srv import GetMediaFileList, SetMediaFile
from pathlib import Path
from os import mkdir
import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
from enum import IntEnum
from bfid import draw_binary_on_image

TOPIC_IMG = "/camera/image_raw"
SRV_START_STOP = "start_stop"
SRV_REMOVE_MEDIA = "remove_media"
SRV_REMOVE_ALL = "remove_all_media"
SRV_GET_MEDIA_LIST = "get_media_list"
SRV_SET_MEDIA_NAME = "set_media_name"

PARAM_MEDIA_LOCATION = "media_location"
PARAM_MEDIA_FPS = "media_fps"
PARAM_MEDIA_WIDTH = "media_width"
PARAM_MEDIA_HEIGHT = "media_height"
PARAM_ON_IMAGE_TIME_STAMP = "on_image_time_stamp"

NODE_NAME = "media_manager"

class State(IntEnum):
    STOP = 0
    ASK_TO_STOP  = 1
    START = 2

class MediaManager(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.bridge = CvBridge()
        self._init_parameters()
        self._init_service()
        self._init_media()
        self._init_subscriber()
        self._video_write = None
        self._state = State.STOP
        self._on_image_time_stamp = self.get_parameter(PARAM_ON_IMAGE_TIME_STAMP).get_parameter_value().bool_value
        self.get_logger().info("Hello ROS2")


    def _init_media(self):
        try:
            path = Path(self.get_parameter(PARAM_MEDIA_LOCATION).get_parameter_value().string_value)
            if not path.exists():
                mkdir(path.as_posix())
        except IOError as e:
            self.get_logger().error(f"Failed to create {path.as_posix()} folder: {str(e)}")

    
        
    def _init_parameters(self):
        self.declare_parameter(PARAM_MEDIA_LOCATION, "/tmp/media")
        self.declare_parameter(PARAM_MEDIA_FPS, 20)
        self.declare_parameter(PARAM_MEDIA_WIDTH, 640)
        self.declare_parameter(PARAM_MEDIA_HEIGHT, 480)
        self.declare_parameter(PARAM_ON_IMAGE_TIME_STAMP, value=True)

    def _init_subscriber(self):
        self.create_subscription(
            Image,
            TOPIC_IMG,
            self.image_callback,
            10
        )



    def _init_service(self):
        """
        init media manage service
        start/stop record
        remove media
        remove all
        get media list
        """
        self.remove_all = self.create_service(
            Trigger,
            SRV_REMOVE_ALL,
            self.remove_all_callback
        )

        self.set_media_name = self.create_service(
            SetMediaFile,
            SRV_SET_MEDIA_NAME,
            self.set_media_name_callback
        )

        self.start_record_service = self.create_service(
            Trigger,
            SRV_START_STOP,
            self.start_stop_callback
        )
        self.remove_media_service = self.create_service(
            SetMediaFile,
            SRV_REMOVE_MEDIA,
            self.remove_media_callback
        )
        self.get_all_media_service = self.create_service(
            GetMediaFileList,
            SRV_GET_MEDIA_LIST,
            self.get_all_media_callback
        )

    #region services callback
    def set_media_name_callback(self, request: SetMediaFile.Request, response: SetMediaFile.Response):
        media_path = Path(self.get_parameter(PARAM_MEDIA_LOCATION).get_parameter_value().string_value)
        fps = self.get_parameter(PARAM_MEDIA_FPS).get_parameter_value().integer_value
        width = self.get_parameter(PARAM_MEDIA_WIDTH).get_parameter_value().integer_value
        height = self.get_parameter(PARAM_MEDIA_HEIGHT).get_parameter_value().integer_value

        filename = request.name
        if not filename.lower().endswith('.mp4'):
            filename += '.mp4'

        self._current_file_path = media_path / filename
        if self._current_file_path.exists():
            response.success = False
            response.message = f"Media {filename} exists"
            return response
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self._video_write = cv2.VideoWriter(str(self._current_file_path.as_posix()), fourcc, fps, (width, height))

        response.success = True
        response.message = "Recording started"
        return response
    
    def start_stop_callback(self, request: Trigger.Request, response: Trigger.Response):
        # Implement your start record logic here
        msg = "error start / stop video record"
        if self._state == State.STOP:
            self._state = State.START
            msg = "START"
        else:
            # stop record
            self._state = State.ASK_TO_STOP
            msg = "STOP"

        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    def remove_all_callback(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("remove all start")
        media_path = Path(self.get_parameter(PARAM_MEDIA_LOCATION).get_parameter_value().string_value)
        removed_files = []
        failed_files = []
        self.get_logger().info(f"remove all: {media_path.as_posix()}")
        for mp4_file in media_path.glob("*.mp4"):
            try:
                mp4_file.unlink()
                removed_files.append(mp4_file.name)
                self.get_logger().info(f"remove all: remove {mp4_file.name}")
            except Exception as e:
                failed_files.append(f"{mp4_file.name}: {str(e)}")
        if failed_files:
            response.success = False
            response.message = f"Failed to remove: {', '.join(failed_files)}"
        else:
            response.success = True
            response.message = f"Removed files: {', '.join(removed_files)}" if removed_files else "No mp4 files found"
        return response
    
    def remove_media_callback(self, request: SetMediaFile.Request, response: SetMediaFile.Response):
        media_path = Path(self.get_parameter(PARAM_MEDIA_LOCATION).get_parameter_value().string_value)
        file_to_remove = media_path / request.name
        if file_to_remove.exists() and file_to_remove.is_file():
            try:
                file_to_remove.unlink()
                response.success = True
                response.message = f"Removed {request.name}"
            except Exception as e:
                response.success = False
                response.message = f"Failed to remove {request.name}: {str(e)}"
        else:
            response.success = False
            response.message = f"File {request.name} does not exist"
        return response

    def get_all_media_callback(self, request: GetMediaFileList.Request, response: GetMediaFileList.Response):
        media_path = Path(self.get_parameter(PARAM_MEDIA_LOCATION).get_parameter_value().string_value)
        mp4_files = [f.name for f in media_path.glob("*.mp4") if f.is_file()]
        response.file_list = mp4_files
        return response
    #endregion services callback

    #region subscribers callbacks
    def image_callback(self, msg: Image):
        # Placeholder for image processing logic
        if self._state in [State.START, State.ASK_TO_STOP]:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self._on_image_time_stamp:
                frame = draw_binary_on_image(frame, msg.header.stamp.sec, msg.header.stamp.nanosec, bit_size=3)
            self._video_write.write(frame)
            if self._state == State.ASK_TO_STOP:
                self._state = State.STOP
                self._video_write.release()
                self._video_write = None
    #endregion

    #region private
    
       
    #endregion
    
def main(args=None):
    rclpy.init(args=args)
    node = MediaManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()