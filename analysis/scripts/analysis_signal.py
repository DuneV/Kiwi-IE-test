import argparse
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from logger_config import get_logger

logger = get_logger(__name__)

def read_imu_data(input_bag: str, output_csv: str, _logger: logger):
    """
    Extracts IMU data from a rosbag and writes it to a CSV file.

    Args:
        input_bag (str): Path to the input MCAP file.
        output_csv (str): Path to the output CSV file.
        logger (logging.Logger): Logger instance for logging.
    """
    _logger.info("Init function.")
    topic_name = "/camera/imu"
    message_type = "sensor_msgs/msg/Imu"

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    # Verify the topic exists in the bag
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if topic_name not in topic_types:
        raise ValueError(f"Topic {topic_name} not found in the bag.")
    if topic_types[topic_name] != message_type:
        raise ValueError(f"Topic {topic_name} does not match expected type {message_type}.")

    # Open CSV file for writing
    try:
        with open(output_csv, mode="w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                "timestamp_sec", "timestamp_nsec",
                "orientation_x", "orientation_y", "orientation_z", "orientation_w",
                "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
                "linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z"
            ])

            print(f"Processing topic: {topic_name}")
            while reader.has_next():
                topic, data, timestamp = reader.read_next()
                if topic == topic_name:
                    # Deserialize the message
                    msg_class = get_message(message_type)
                    msg = deserialize_message(data, msg_class)

                    # Extract relevant fields
                    writer.writerow([
                        msg.header.stamp.sec, msg.header.stamp.nanosec,
                        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                    ])
    except Exception as e:
        _logger.error(f"Error function: {e}")
    finally:
        _logger.debug("Data procesed")
        print(f"IMU data saved to {output_csv}")
        _logger.info("End function.")

def main():
    parser = argparse.ArgumentParser(description="Convert IMU data from MCAP to CSV.")
    parser.add_argument(
        "input", help="Path to the input MCAP file"
    )
    parser.add_argument(
        "output", help="Path to the output CSV file"
    )

    args = parser.parse_args()
    try:
        read_imu_data(args.input, args.output, logger)
    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()


