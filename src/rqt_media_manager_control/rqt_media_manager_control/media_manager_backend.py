import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from media_manager_interfaces.srv import GetMediaFileList, SetMediaFile
from rcl_interfaces.srv import GetParameters
import subprocess
import pathlib

SRV_START_STOP = "start_stop"
SRV_REMOVE_MEDIA = "remove_media"
SRV_REMOVE_ALL = "remove_all_media"
SRV_GET_MEDIA_LIST = "get_media_list"
SRV_SET_MEDIA_NAME = "set_media_name"
NODE_NAME = "media_manager"

SERVICE_CALL_TIMEOUT = 4

class Event:
    def __init__(self):
        self._handlers = []

    def __iadd__(self, handler):
        self._handlers.append(handler)
        return self

    def fire(self, *args, **kwargs):
        for handler in self._handlers:
            handler(*args, **kwargs)


class BackendNode(Node):
    def __init__(self):
        super().__init__('my_plugin_backend')
        self.__load_media_client = self.create_client(GetMediaFileList, SRV_GET_MEDIA_LIST)
        self.__remove_media_client = self.create_client(SetMediaFile, SRV_REMOVE_MEDIA)
        self.__set_media_client = self.create_client(SetMediaFile, SRV_SET_MEDIA_NAME)
        self.__start_stop_client = self.create_client(Trigger, SRV_START_STOP)
        self.__remove_all_client = self.create_client(Trigger, SRV_REMOVE_ALL)
        self.__get_media_location_client = self.create_client(GetParameters, f'/{NODE_NAME}/get_parameters')

        self.on_media_list_fetch = Event()
        self.on_error = Event()
        self.on_set_media = Event()
        self.on_start_record = Event()
        self.on_stop_record = Event()
        self.on_download_done = Event()
        self.on_connected = Event()


    def wait_for_service_handler(self, service):
        WAIT_TIME = 1
        MAX_RETRIES = 3
        counter = 0
        while not service.wait_for_service(WAIT_TIME):
            self.get_logger().warning("wait for service")
            counter += 1
            if counter > MAX_RETRIES:
                msg = "Service not available, run media manage node"
                self.on_error.fire(msg)
                self.get_logger().error(msg)
                raise Exception(msg)

    def run(self):
        self.wait_for_service_handler(self.__load_media_client)
        self.wait_for_service_handler(self.__remove_all_client)


        self.load_media()
        self.get_media_location_param()
        self.on_connected.fire()


    def get_media_location_param(self):
        request = GetParameters.Request()
        request.names = ["media_location"]

        future = self.__get_media_location_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: GetParameters.Response
        if future.result():
            response = future.result()
            
            if len(response.values) != 1:
                self.on_error("Failed to load remote media location")

            self.remote_media_location = response.values[0].string_value
            self.get_logger().info(f"remote location: {self.remote_media_location}")
        else:
            self.get_logger().error("Service call failed")


    def load_media(self):
        self.req = GetMediaFileList.Request()
        future = self.__load_media_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT)
        if not future.done():
            self.get_logger().error("Timeout while loading media files")
            self.on_error.fire("Timeout while loading media files")
            return
        if future.result() is not None:
            response: GetMediaFileList.Response
            response = future.result()
            self.on_media_list_fetch.fire(response.file_list)
            self.get_logger().info(f"Media files: {future.result().file_list}")
        else:
            self.get_logger().error("load media service failed")
            self.on_error.fire("load media service failed")

    def remove_media(self, file_name):
        req: SetMediaFile.Request = SetMediaFile.Request()
        req.name = file_name
        future = self.__remove_media_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT)
        if not future.done():
            msg = "Timeout while remove media files"
            self.get_logger().error(msg)
            self.on_error.fire(msg)
            return
        if future.result() is not None:
            response: SetMediaFile.Response = future.result()
            if response.success:
                self.load_media()
        else:
            self.get_logger().error(f"Failed to remove media: {file_name}")

    def set_media(self, file_name):
        req: SetMediaFile.Request = SetMediaFile.Request()
        req.name = file_name
        future = self.__set_media_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response: SetMediaFile.Response = future.result()
            if response.success:
                self.on_set_media.fire()
            else:
                self.get_logger().error("Media exists")
                self.on_error.fire(response.message)
        else:
            self.get_logger().error(f"Failed to remove media: {file_name}")

    def start_stop_media_record(self):
        req: Trigger.Request = Trigger.Request()
        future = self.__start_stop_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response: Trigger.Response = future.result()
            if response.success:
                if response.message == "START":
                    self.on_start_record.fire()
                else:
                    self.on_stop_record.fire()
        else:
            self.get_logger().error(f"Failed start / stop record media")

    def remove_all_remote_files(self):
        self.get_logger().info("remove all start")
        response: Trigger.Response
        req: Trigger.Request = Trigger.Request()
        future = self.__remove_all_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT)
        if not future.done():
            msg = "Timeout while remove all media files"
            self.get_logger().error(msg)
            self.on_error.fire(msg)
            return
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.load_media()
        else:
            self.get_logger().error(f"Failed start / stop record media")
            self.on_error(response.message)

    def download_file(self, source, target_folder):
        remote_path = pathlib.Path(self.remote_media_location).joinpath(source).as_posix()
        rsync_cmd = [
        "rsync", "-avz", "--progress",
        remote_path,
        target_folder
        ]

        try:
            subprocess.run(rsync_cmd, check=True)
            self.on_download_done.fire(source)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"failed to download {source}")
            self.on_error.fire("Failed to download check if rsync installed")