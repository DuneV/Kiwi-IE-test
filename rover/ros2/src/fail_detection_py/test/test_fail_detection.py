import subprocess
import pytest
import rclpy
from rclpy.node import Node
from usr_msgs.msg import Fails
from sensor_msgs.msg import Imu
from fail_detection_py.nodes.fail_detector import FailDetector


@pytest.fixture(scope="function")
def fail_detector_node():
    rclpy.init()
    node = FailDetector()
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture(scope="function")
def imu_rosbag(fail_detector_node):
    # Reproduce el rosbag en segundo plano
    rosbag_process = subprocess.Popen(
        [
            "ros2",
            "bag",
            "play",
            "/path/to/your/rosbag_file.db3",
            "--topics",
            "/camera/imu",
            "/imu/data",
        ]
    )
    yield
    rosbag_process.terminate()


def test_fail_detector_with_rosbag(imu_rosbag, fail_detector_node):

    rclpy.spin_once(fail_detector_node, timeout_sec=5)

    assert len(fail_detector_node.pub_fail.get_subscription_count()) > 0
    fail_detector_node.get_logger().info("Testing Failures with the mcap files")
