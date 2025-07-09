import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from media_manager_interfaces.srv import GetMediaFileList, SetStringArray
from rcl_interfaces.srv import GetParameters
import subprocess
import pathlib
import yaml

SRV_START_STOP = "start_stop"
SRV_REMOVE_MEDIA = "remove_media"
SRV_REMOVE_ALL = "remove_all_media"
SRV_GET_MEDIA_LIST = "get_media_list"
SRV_SET_MEDIA_NAME = "set_media_name"
SRV_GET_PROFILE_LIST = "get_profile_list"
SRV_EXPORT_PROFILE = "export_profile"
SRV_IMPORT_PROFILE = "import_profile"


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
        self.__source = ""
        self.__load_media_client = None#self.create_client(GetMediaFileList, SRV_GET_MEDIA_LIST)
        self.__remove_media_client = None#
        self.__set_media_client = None#
        self.__start_stop_client = None # 
        self.__remove_all_client = None
        self.__export_profile_client = None
        self.__import_profile_client = None
        self.__get_media_location_client = None
        self.__load_profile_client = None
        self.__selected_profile = None
        self.remote_address = None
        self.remote_media_location = None

        self.on_profile_fetch = Event()
        self.on_media_list_fetch = Event()
        self.on_error = Event()
        self.on_set_media = Event()
        self.on_start_record = Event()
        self.on_stop_record = Event()
        self.on_download_done = Event()
        self.on_connected = Event()

    def _get_full_topic_name(self, topic):
        topic_name = f'/{self.__source}/{topic}'
        return topic_name

    def _init_service(self):
        if self.__load_profile_client is not None:
            self.__load_media_client.destroy()
        self.__load_profile_client = self.create_client(GetMediaFileList, self._get_full_topic_name(SRV_GET_PROFILE_LIST))

        if self.__load_media_client is not None:
            self.__load_media_client.destroy()
        self.__load_media_client = self.create_client(GetMediaFileList, self._get_full_topic_name(SRV_GET_MEDIA_LIST))

        if self.__remove_media_client is not None:
            self.__remove_media_client.destroy()
        self.__remove_media_client = self.create_client(SetStringArray, self._get_full_topic_name(SRV_REMOVE_MEDIA))

        if self.__set_media_client is not None:
            self.__set_media_client.destroy()
        self.__set_media_client = self.create_client(SetStringArray, self._get_full_topic_name(SRV_SET_MEDIA_NAME))

        if self.__start_stop_client is not None:
            self.__start_stop_client.destroy()
        self.__start_stop_client = self.create_client(Trigger, self._get_full_topic_name(SRV_START_STOP))

        if self.__remove_all_client is not None:
            self.__remove_all_client.destroy()
        self.__remove_all_client = self.create_client(Trigger, self._get_full_topic_name(SRV_REMOVE_ALL))

        if self.__get_media_location_client is not None:
            self.__get_media_location_client.destroy()
        self.__get_media_location_client = self.create_client(GetParameters, self._get_full_topic_name("get_parameters"))

        if self.__export_profile_client is not None:
            self.__export_profile_client.destroy()
        self.__export_profile_client = self.create_client(Trigger, self._get_full_topic_name(SRV_EXPORT_PROFILE))

        if self.__import_profile_client is not None:
            self.__import_profile_client.destroy()
        self.__import_profile_client = self.create_client(SetStringArray, self._get_full_topic_name(SRV_IMPORT_PROFILE))

    def set_profile(self, profile_name):
        self.__selected_profile = profile_name
        self.get_logger().info(f"Selected profile: {self.__selected_profile}")


    def set_source(self, source):
        #TODO move to enum and do better
        if source == "mp4":
            self.__source = "media_manager"
        else:
            self.__source = "my_bag_record"
        self.get_logger().info(f"--------- init server for source: {self.__source}")
        self._init_service()
        self.run()


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
        try:
            self.wait_for_service_handler(self.__load_media_client)
            self.wait_for_service_handler(self.__remove_all_client)


            self.load_media()
            self.load_profiles()
            self.load_parameters_from_remote()
            self.on_connected.fire()
        except Exception as e:
            self.get_logger().error(f"Failed to load remote service {e}")
            self.on_error.fire("Failed to load remote service")

    def get_all_topics(self):
        """
        Get all topics from the media manager node
        """
        topics = self.get_topic_names_and_types()
        topics = [topic for topic, _ in topics]
        self.get_logger().info(f"Topics: {topics}")
        return topics
    
    def create_profile(self, profile_name, topics):
        pass


    def load_parameters_from_remote(self):
        data = self.get_param_values([
            "node_address", 
            "media_location"])
        
        self.remote_address, self.remote_media_location = data

    def get_param_values(self, param_names):
        request = GetParameters.Request()
        request.names = param_names

        future = self.__get_media_location_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response: GetParameters.Response
        if future.result():
            response = future.result()

            return [param.string_value for param in response.values]
        else:
            self.get_logger().error("Service call failed")
            #TODO: ????

    def load_profiles(self):
        self.req = GetMediaFileList.Request()
        future = self.__load_profile_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT)
        if not future.done():
            self.get_logger().error("Timeout while loading media files")
            self.on_error.fire("Timeout while loading media files")
            return
        if future.result() is not None:
            response: GetMediaFileList.Response
            response = future.result()
            self.on_profile_fetch.fire(response.file_list)
            self.get_logger().info(f"Media files: {future.result().file_list}")
        else:
            self.get_logger().error("load media service failed")
            self.on_error.fire("load media service failed")

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
        req: SetStringArray.Request = SetStringArray.Request()
        req.data.append(file_name)
        future = self.__remove_media_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT)
        if not future.done():
            msg = "Timeout while remove media files"
            self.get_logger().error(msg)
            self.on_error.fire(msg)
            return
        if future.result() is not None:
            response: SetStringArray.Response = future.result()
            if response.success:
                self.load_media()
        else:
            self.get_logger().error(f"Failed to remove media: {file_name}")

    def set_media(self, file_name):
        """
        """
        req: SetStringArray.Request = SetStringArray.Request()
        req.data.append(file_name)
        req.data.append(self.__selected_profile)

        future = self.__set_media_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response: SetStringArray.Response = future.result()
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

    def import_profile(self, file_name):
        """
        Import profile from yaml file
        """
        with open(file_name, 'r') as f:
            yaml_obj = yaml.safe_load(f)

        req: SetStringArray.Request = SetStringArray.Request()
        req.data.append(yaml.dump(yaml_obj, default_flow_style=False, sort_keys=False))
        future = self.__import_profile_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT)

        if not future.done():
            msg = "Timeout while importing profile"
            self.get_logger().error(msg)
            self.on_error.fire(msg)
            return
        if future.result() is not None:
            response = future.result()
            if response.success:


                self.load_profiles()
                self.get_logger().info(f"Profile imported from {file_name}")
        else:
            self.get_logger().error(f"Failed export profile")
            self.on_error(response.message)




                
    def export_profile(self, target_folder):
        """
        """
        response: Trigger.Response
        req: Trigger.Request = Trigger.Request()
        future = self.__export_profile_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT)
        if not future.done():
            msg = "Timeout while exporting profile"
            self.get_logger().error(msg)
            self.on_error.fire(msg)
            return
        if future.result() is not None:
            response = future.result()
            if response.success:


                yaml_obj = yaml.safe_load(response.message)
                with open(pathlib.Path(target_folder).joinpath("exported_profile.yaml"), "w") as f:
                    yaml.dump(yaml_obj, f, default_flow_style=False, sort_keys=False)

                self.get_logger().info(f"Profile exported to {target_folder}")
        else:
            self.get_logger().error(f"Failed export profile")
            self.on_error(response.message)

    def download_file(self, source, target_folder):
        remote_path = pathlib.Path(self.remote_media_location).joinpath(source).as_posix()
        self.run_rsync(remote_path, target_folder
                       
                       )
    def run_rsync(self, source, target_folder):
        """
        rsync -avz user@<remote_ip>:/path/to/file ./local/
        """
        rsync_cmd = [
        "rsync", "-avz", "--progress",
        f"user@127.0.0.1:{source}", #TODO: move to config
        target_folder
        ]

        try:
            subprocess.run(rsync_cmd, check=True)
            self.on_download_done.fire(source)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"failed to download {source}")
            self.on_error.fire("Failed to download check if rsync installed")