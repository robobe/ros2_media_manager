import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from media_manager_interfaces.srv import GetMediaFileList, SetMediaFile

SRV_START_STOP = "start_stop"
SRV_REMOVE_MEDIA = "remove_media"
SRV_REMOVE_ALL = "remove_all"
SRV_GET_MEDIA_LIST = "get_media_list"
SRV_SET_MEDIA_NAME = "set_media_name"

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
        self.load_media_client = self.create_client(GetMediaFileList, SRV_GET_MEDIA_LIST)
        self.remove_media_client = self.create_client(SetMediaFile, SRV_REMOVE_MEDIA)
        self.set_media_client = self.create_client(SetMediaFile, SRV_SET_MEDIA_NAME)
        self.start_stop_client = self.create_client(Trigger, SRV_START_STOP)

        self.on_media_list_fetch = Event()
        self.on_error = Event()
        self.on_set_media = Event()
        self.on_start_record = Event()
        self.on_stop_record = Event()

    def run(self):
        while not self.load_media_client.wait_for_service(1):
            self.get_logger("wait for service")

        self.load_media()



    def load_media(self):
        self.req = GetMediaFileList.Request()
        future = self.load_media_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
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
        future = self.remove_media_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response: SetMediaFile.Response = future.result()
            if response.success:
                self.load_media()
        else:
            self.get_logger().error(f"Failed to remove media: {file_name}")

    def set_media(self, file_name):
        req: SetMediaFile.Request = SetMediaFile.Request()
        req.name = file_name
        future = self.set_media_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response: SetMediaFile.Response = future.result()
            if response.success:
                self.on_set_media.fire()
        else:
            self.get_logger().error(f"Failed to remove media: {file_name}")

    def start_stop_media_record(self):
        req: Trigger.Request = Trigger.Request()
        future = self.start_stop_client.call_async(req)
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