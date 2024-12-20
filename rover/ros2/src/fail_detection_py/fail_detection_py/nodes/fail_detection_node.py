#!/usr/bin/env python3

import os

# deactivate innecesary warnings
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

import numpy as np
from rclpy.node import Node
import tensorflow as tf
from sklearn.preprocessing import OneHotEncoder
from usr_msgs.msg import Fails
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tensorflow.compat.v1 import logging

logging.set_verbosity(logging.ERROR)

from fail_detection_py.callbacks.imu_handler import ImuHandler

encoder = OneHotEncoder(sparse_output=False)
encoder.fit(np.array(["collision", "Bump", "Normal"]).reshape(-1, 1))
lstm_model = tf.lite.Interpreter(
    "/workspace/rover/ros2/src/fail_detection_py/fail_detection_py/nodes/models/model_conditions_wtime2sy.tflite"
)
lstm_model.allocate_tensors()


class FailDetector(Node):
    def __init__(self):

        super().__init__("fail_detector")

        # env variables
        self.collision_jerk = float(os.getenv("FAIL_DETECTION_COLLISION_JERK", 400.0))
        self.n_samples = int(os.getenv("FAIL_DETECTION_COLLISION_SAMPLES", 100))

        # Publisher
        self.pub_fail = self.create_publisher(
            Fails,
            "/fail_detection/fail",
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.VOLATILE),
        )
        logger = self.get_logger()

        # Handler IMU
        self.imu_handler = ImuHandler(
            _n_samples=12,
            _pub_fail=self.pub_fail,
            _logger=logger,
            _lstm_model=lstm_model,
            _encoder=encoder,
            _node=self,
        )

        # Subs
        self.subs_imu_camera = self.create_subscription(
            Imu, "/camera/imu", self.imu_handler.imu_cb, QoSProfile(depth=1)
        )
        self.subs_imu_chassis = self.create_subscription(
            Imu, "/imu/data", self.imu_handler.chassis_imu_cb, QoSProfile(depth=1)
        )

        self.get_logger().info("FailDetector node initialized successfully :)!")

    # def destroy_node(self):
    #     """
    #     Override destroy_node to stop the worker thread.
    #     """
    #     self.imu_handler.stop_worker()
    #     super().destroy_node()
