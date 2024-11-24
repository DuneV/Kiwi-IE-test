import os
import rclpy
from usr_msgs.msg import Fails
from collections import deque
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class ImuHandler:
    def __init__(self, node):
        """
        Initialize the IMU handler.

        Args:
            node (Node): Reference to the ROS node using this handler.
        """
        self.node = node
        self.collision_jerk = float(os.getenv("FAIL_DETECTION_COLLISION_JERK", 400.0))
        self.imu_msgs_deque = deque(maxlen=node.n_samples)
        self.accel_deque = deque(maxlen=node.n_samples)

    def imu_cb(self, msg: Imu):
        """Process IMU data for collision detection."""
        self.imu_msgs_deque.appendleft(msg)
        self.accel_deque.appendleft(msg.linear_acceleration.z)

        if len(self.accel_deque) > 1:
            dt = headers2dt(
                self.imu_msgs_deque[1].header, self.imu_msgs_deque[0].header
            )
            if dt > 0:
                jerk = abs((self.accel_deque[0] - self.accel_deque[1]) / dt)
                if jerk > self.collision_jerk:
                    self.node.get_logger().warn(f"Collision detected! Jerk: {jerk}")
                    fail_msg = Fails()
                    fail_msg.type = "collision"
                    fail_msg.severity = "high"
                    fail_msg.message = f"Detected a collision with jerk: {jerk}"
                    self.node.pub_fail.publish(fail_msg)

    def chassis_imu_cb(self, msg: Imu):
        """Process chassis IMU data for rollover detection."""
        quat = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        roll, pitch, yaw = euler_from_quaternion(quat)

        if abs(roll) > 0.5 or abs(pitch) > 0.5:
            self.node.get_logger().warn(
                f"Rollover detected! Roll: {roll}, Pitch: {pitch}"
            )
            fail_msg = Fails()
            fail_msg.type = "rollover"
            fail_msg.severity = "critical"
            fail_msg.message = f"Detected a rollover. Roll: {roll}, Pitch: {pitch}"
            self.node.pub_fail.publish(fail_msg)
