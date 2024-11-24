#!/usr/bin/env python3

import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from usr_msgs.msg import Fails


def headers2dt(header1: Header, header2: Header):
    """Calculate time difference between two message headers in nanoseconds"""
    dt_ns = (header1.stamp.nanosec - header2.stamp.nanosec) + (
        header1.stamp.sec - header2.stamp.sec
    ) * 1e9
    return dt_ns


class FailDetector(Node):
    def __init__(self):
        super().__init__("fail_detector")

        # Environment variables
        self.collision_jerk = float(os.getenv("FAIL_DETECTION_COLLISION_JERK", 400.0))
        self.imu_no_msgs_report_time = int(
            os.getenv("FAIL_DETECTION_IMU_NO_MSGS_REPORT_TIME", 5)
        )
        self.n_samples_collision = int(os.getenv("FAIL_DETECTION_COLLISION_SAMPLES", 5))
        self.n_samples_fall = int(os.getenv("FAIL_DETECTION_COLLISION_SAMPLES", 100))
        self.n_samples = max(self.n_samples_fall, self.n_samples_collision)

        # Class variables
        self.bot_speed = 0.0
        self.motion_state = "forward"

        # Initialize deques
        self.imu_msgs_deque = deque(maxlen=self.n_samples)
        self.accel_deque = deque(maxlen=self.n_samples)

        default_sub_qos = QoSProfile(depth=1)
        transient_local_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Create subscribers
        self.subs_imu_camera = self.create_subscription(
            Imu, "/camera/imu", self.imu_cb, default_sub_qos
        )

        self.subs_imu_chassis = self.create_subscription(
            Imu, "/imu/data", self.chassis_imu_cb, default_sub_qos
        )

        self.subs_bot_speed = self.create_subscription(
            Odometry,
            "/wheel_odometry/local_odometry",
            self.bot_speed_cb,
            default_sub_qos,
        )

        # Create publishers
        self.pub_fail = self.create_publisher(
            Fails, "/fail_detection/fail", transient_local_qos
        )

        ## Timers
        # Add some timer if you need it
        # self.dummy_tmr = self.create_timer(0.3, self.timer_callback)
        # self.dummy_tmr.cancel()

        self.get_logger().info("Fail detector constructor: Success!")

    def imu_cb(self, msg: Imu) -> None:
        """Receive the imu msg and process it to detect collisions"""
        # keep the vector with the last self.n_samples samples. The most recent reading is stored at the first position
        self.imu_msgs_deque.appendleft(msg)
        self.accel_deque.appendleft(msg.linear_acceleration.z)
        
        # TODO:
        # Detect collisions, collisions are related to big jerks, have you some idea?

        # I define the jerk as the derivate of the aceleration respect time so with the expresion 
        # To enter more details about how calculate it consultate file 
        # We cacth 2 values at least of the
        if len(self.accel_deque) > 1: 
            # compute delta time example
            dt = self.headers2dt(self.imu_msgs_deque[1], self.imu_msgs_deque[0])
            # avoid division by zero 
            if dt > 0:
                # calculate the jerk
                jerk = abs((self.accel_deque[0] - self.accel_deque[1]) / dt)
                if jerk > self.collision_jerk:
                    self.get_logger().warn(f"Collision detected! Jerk: {jerk}")
                    fail_msg = Fails()
                    fail_msg.type = "collision"
                    fail_msg.severtity = "high"
                    fail_msg.message = f"Detected a collision with jerk: {jerk}"
                    self.pub_fail.publish(fail_msg)

    def chassis_imu_cb(self, msg: Imu) -> None:
        """Receive the chassis imu msg and process it to detect pitch changes"""
        quat = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        # Chassis Imu is aligned with the robot base link frame
        # TODO:
        # Detect rollovers
        # I need to change this values based on the data 
        roll, pitch, yaw = euler_from_quaternion(quat)
        
        if abs(roll) > 0.5 or abs(pitch) > 0.5:
            self.get_logger().warn(f"Rollover detected! Roll: {roll}, Pitch: {pitch}")
            fail_msg = Fails()
            fail_msg.type = "rollover"
            fail_msg.severity = "critical"
            fail_msg.message = f"Detected a rollover. Roll: {roll}, Pitch: {pitch}"
            self.pub_fail.publish(fail_msg)

    def bot_speed_cb(self, msg: Odometry) -> None:
        """Receive the speed command of the robot to know if its going forward or backwards"""
        self.bot_speed = msg.twist.twist.linear.x
        if self.bot_speed > 0.0 and self.motion_state != "forward":
            self.motion_state = "forward"
            self.get_logger().info("Robot is moving forward.")

        elif self.bot_speed < 0.0 and self.motion_state != "backwards":
            self.motion_state = "backwards"
            self.get_logger().info("Robot is moving backwards.")


def main(args=None):
    rclpy.init(args=args)

    fail_detection_node = FailDetector()

    node_period_s = int(os.getenv("FAIL_DETECTION_NODE_PERIOD", "50")) * 1e-3

    try:
        while rclpy.ok():
            rclpy.spin_once(fail_detection_node)
            time.sleep(node_period_s)

    except KeyboardInterrupt:
        pass

    fail_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
