import os
import subprocess
import pytest
import rclpy
from rclpy.node import Node
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
    """Fixture to initialize the FailDetector ROS 2 node."""
    rclpy.init()  # Initialize the ROS 2 system
    node = FailDetector()  # Create the FailDetector node
    yield node  # Yield the node to the test
    node.destroy_node()  # Clean up by destroying the node after the test
    rclpy.shutdown()  # Shut down the ROS 2 system


# Function to save the received messages in a JSON file
def save_results_to_file(messages, rosbag_file):
    """Function to save the received failure messages to a JSON file."""
    # Extract the base name of the rosbag file (without path and extension)
    rosbag_name = os.path.basename(rosbag_file).split(".")[0]
    output_dir = "/workspace/diagrams"  # Directory where results will be saved
    os.makedirs(output_dir, exist_ok=True)  # Ensure the output directory exists

    # File path for the JSON file (named based on the rosbag)
    result_path = os.path.join(output_dir, f"{rosbag_name}_failures.json")

    # Saving the received messages as JSON
    with open(result_path, "w") as f:
        json.dump(messages, f, indent=4)  # Pretty-print the messages as JSON

    print(f"Results for {rosbag_name} saved to {result_path}")


# Test function to process multiple rosbags and check the received messages
def test_fail_detector_with_rosbags(fail_detector_node):

    global COUNTER

    """Test to check FailDetector node processing with multiple rosbags."""
    messages_received = []  # List to store received messages

    # Callback function to process failure messages and append them to the list
    def fail_callback(msg):
        """Callback to handle failure messages and append to the list."""
        messages_received.append(msg.fails[0].event)

    # Create a subscription to listen for failure messages
    fail_detector_node.create_subscription(
        Fails,
        "/fail_detection/fail",
        fail_callback,
        10,  # S
    )

    # Set a timeout for waiting for messages (in seconds)
    timeout_sec = 120  # Increased the timeout to 2 minutes to ensure enough time
    start_time = rclpy.clock.Clock().now().seconds_nanoseconds()[0]

    # Iterate over the list of rosbag files and process each one sequentially
    for rosbag_file in ROS_BAG_FILES:
        # Start the rosbag process in the background without blocking the test
        rosbag_process = subprocess.Popen(
            [
                "ros2",  # Command to use ros2
                "bag",  # ROS 2 bag command
                "play",  # Play the rosbag
                rosbag_file,  # The path to the rosbag file
                "--topics",  # Topics to be played from the bag
                "/camera/imu",
                "/imu/data",
            ]
        )

        # Spin the node to process the messages from this rosbag
        fail_detector_node.get_logger().info(f"Processing messages from {rosbag_file}")
        start_time = (
            rclpy.clock.Clock().now().seconds_nanoseconds()[0]
        )  # Reset start time

        # Spin the node and process messages
        while rclpy.ok():
            # Calculate the elapsed time
            elapsed_time = (
                rclpy.clock.Clock().now().seconds_nanoseconds()[0] - start_time
            )
            if elapsed_time > timeout_sec:
                fail_detector_node.get_logger().warn(
                    "Timeout reached. Ending the test."
                )
                break  # Break out of the loop if timeout is exceeded

            # Process incoming ROS messages
            rclpy.spin_once(fail_detector_node, timeout_sec=0.1)

        # After spinning, check if any messages were received
        fail_detector_node.get_logger().info(
            f"Total messages received for {rosbag_file}: {len(messages_received)}"
        )

        # Assert that we received at least one failure message
        assert (
            len(messages_received) >= CONDITIONS[COUNTER]
        ), f"No failure messages were received for {rosbag_file}"

        # Save the results (received messages) to a JSON file
        save_results_to_file(messages_received, rosbag_file)

        # Clear the received messages list for the next rosbag
        messages_received.clear()

        # Wait for the rosbag process to finish before moving on to the next one
        rosbag_process.wait()  # Wait for the rosbag to finish before continuing the test
        COUNTER += 1
