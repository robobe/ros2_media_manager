import os
import tempfile
import shutil
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from pathlib import Path
from media_manager.manager import MediaManager, SRV_GET_MEDIA_LIST
from media_manager_interfaces.srv import GetMediaFileList

# Python



@pytest.fixture(scope="module")
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def temp_media_dir():
    temp_dir = tempfile.mkdtemp()
    # Create test files
    Path(os.path.join(temp_dir, "video1.mp4")).touch()
    Path(os.path.join(temp_dir, "video2.mp4")).touch()
    Path(os.path.join(temp_dir, "audio1.wav")).touch()
    yield temp_dir
    shutil.rmtree(temp_dir)

@pytest.fixture
def media_manager_node(temp_media_dir, rclpy_init_shutdown):
    node = MediaManager()
    node.set_parameters([rclpy.parameter.Parameter("media_location", rclpy.Parameter.Type.STRING, temp_media_dir)])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    yield node, executor
    executor.shutdown()
    node.destroy_node()

def test_get_all_media_service(media_manager_node, temp_media_dir):
    node: Node
    executor: SingleThreadedExecutor
    
    node, executor = media_manager_node

    # Create a client for the GetMediaFileList service
    client = node.create_client(GetMediaFileList, SRV_GET_MEDIA_LIST)
    assert client.wait_for_service(timeout_sec=2.0)

    # Prepare request
    request = GetMediaFileList.Request()
    # The request may have fields, but in your code it's not used

    # Call the service
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, executor=executor)
    response = future.result()

    # Check that only mp4 files are returned
    assert sorted(response.file_list) == sorted(["video1.mp4", "video2.mp4"])