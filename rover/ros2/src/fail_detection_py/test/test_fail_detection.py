import subprocess
import pytest
import rclpy
from usr_msgs.msg import Fails
import json  # Importing JSON for saving results
from pathlib import Path
from fail_detection_py.nodes.fail_detection_node import FailDetector

# Path to the current directory for easier access
current_dir = Path(__file__).resolve().parent

# Test values for the rosbag files
ROS_BAG_FILES = [
    "/workspace/rosbags/diagnostic_0.mcap",
    "/workspace/rosbags/bag0.mcap_0.mcap",
    "/workspace/rosbags/bag1.mcap_0.mcap",
    "/workspace/rosbags/bag2.mcap_0.mcap",
    "/workspace/rosbags/bag3.mcap_0.mcap",
    "/workspace/rosbags/bag4.mcap_0.mcap",
    "/workspace/rosbags/bag5.mcap_0.mcap",
    "/workspace/rosbags/bag6.mcap_0.mcap",
]
CONDITIONS = [1, 0, 1, 1, 0, 1, 1, 0]

COUNTER = 0

# Fixture to create the FailDetector node
@pytest.fixture(scope="function")
def fail_detector_node():
    """
    Fixture to initialize the FailDetector ROS 2 node.

    This fixture initializes the ROS 2 system, creates the FailDetector node,
    and yields it for use in the tests. After the test is finished, it cleans
    up by destroying the node and shutting down the ROS 2 system.

    Yields:
        FailDetector: The initialized FailDetector node.
    """
    rclpy.init()  # Initialize the ROS 2 system
    node = FailDetector()  # Create the FailDetector node
    yield node  # Yield the node to the test
    node.destroy_node()  # Clean up by destroying the node after the test
    rclpy.shutdown()  # Shut down the ROS 2 system

# Function to save the received messages in a JSON file
def save_results_to_file(messages, rosbag_file):
    """
    Function to save the received failure messages to a JSON file.

    Args:
        messages (list): List of failure messages to be saved.
        rosbag_file (str): The rosbag file being processed, used to name the output file.

    Saves the failure messages as a JSON file in the `/workspace/diagrams` directory.
    The file is named based on the rosbag file name.
    """
    rosbag_name = os.path.basename(rosbag_file).split(".")[0]
    output_dir = "/workspace/diagrams"
    os.makedirs(output_dir, exist_ok=True)

    result_path = os.path.join(output_dir, f"{rosbag_name}_failures.json")

    with open(result_path, "w") as f:
        json.dump(messages, f, indent=4)

    print(f"Results for {rosbag_name} saved to {result_path}")

# Test function to process multiple rosbags and check the received messages
def test_fail_detector_with_rosbags(fail_detector_node):
    """
    Test to check FailDetector node processing with multiple rosbags.

    This test processes a series of rosbag files, simulating a failure detection process.
    For each rosbag, the test subscribes to the `/fail_detection/fail` topic to
    receive failure messages. The test then checks whether the expected number of messages
    are received, based on predefined conditions.

    Args:
        fail_detector_node (FailDetector): The initialized FailDetector node to be tested.
    """
    global COUNTER
    messages_received = []

    def fail_callback(msg):
        """
        Callback to handle failure messages and append to the list.

        Args:
            msg (Fails): The failure message received from the /fail_detection/fail topic.
        """
        messages_received.append(msg.fails[0].event)

    # Create a subscription to listen for failure messages
    fail_detector_node.create_subscription(
        Fails,
        "/fail_detection/fail",
        fail_callback,
        10,
    )

    timeout_sec = 120
    start_time = rclpy.clock.Clock().now().seconds_nanoseconds()[0]

    # Iterate over the list of rosbag files and process each one
    for rosbag_file in ROS_BAG_FILES:
        # Start the rosbag process in the background without blocking the test
        rosbag_process = subprocess.Popen(
            [
                "ros2", "bag", "play", rosbag_file, "--topics", "/camera/imu", "/imu/data"
            ]
        )

        fail_detector_node.get_logger().info(f"Processing messages from {rosbag_file}")
        start_time = rclpy.clock.Clock().now().seconds_nanoseconds()[0]

        while rclpy.ok():
            # Calculate elapsed time
            elapsed_time = rclpy.clock.Clock().now().seconds_nanoseconds()[0] - start_time
            if elapsed_time > timeout_sec:
                fail_detector_node.get_logger().warn("Timeout reached. Ending the test.")
                break

            rclpy.spin_once(fail_detector_node, timeout_sec=0.1)

        fail_detector_node.get_logger().info(
            f"Total messages received for {rosbag_file}: {len(messages_received)}"
        )

        # Assert that we received at least the expected number of failure messages
        assert len(messages_received) >= CONDITIONS[COUNTER], f"No failure messages for {rosbag_file}"

        # Save the results (received messages) to a JSON file
        save_results_to_file(messages_received, rosbag_file)

        # Clear the received messages list for the next rosbag
        messages_received.clear()

        # Wait for the rosbag process to finish before moving on to the next one
        rosbag_process.wait()
        COUNTER += 1
