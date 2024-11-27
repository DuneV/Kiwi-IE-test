#!/usr/bin/env python3

import rclpy
from fail_detection_py.nodes.fail_detection_node import FailDetector


def main(args=None):
    # Init client
    rclpy.init(args=args)

    # Created the Class node
    fail_detection_node = FailDetector()

    # Config the loop
    try:
        rclpy.spin(fail_detection_node)
    except KeyboardInterrupt:
        print("Node stop by user (Ctrl+C)")
    finally:
        fail_detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
