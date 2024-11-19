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

class FailDetectionLogic:
    def __init__(self, collision_jerk: float):
        self.collision_jerk = collision_jerk

    def detect_collision(self, accel_deque):
        if len(accel_deque) < 2:
            return False

        jerk = abs(accel_deque[0] - accel_deque[1])
        return jerk > self.collision_jerk

    def detect_rollover(self, quat):
        roll, pitch, yaw = euler_from_quaternion(quat)
        return abs(roll) > 0.5 or abs(pitch) > 0.5
