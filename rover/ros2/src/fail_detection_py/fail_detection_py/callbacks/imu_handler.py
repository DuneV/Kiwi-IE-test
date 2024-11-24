#!/usr/bin/env python3

import os
import numpy as np
from collections import deque
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from usr_msgs.msg import Fails
from fail_detection_py.utils.time_utils import headers2dt
from fail_detection_py.utils.math_utils import (
    process_both_methods,
    classify_events_with_secondary_opinion,
)

INFO_DICT = {"Collision": 0, "Bump": 1, "Normal": 2}


class ImuHandler:
    def __init__(
        self, collision_jerk, n_samples, pub_fail, logger, lstm_model, encoder
    ):
        """
        Initialize the IMU handler.

        Args:
            collision_jerk (float): Threshold for jerk to detect collisions.
            n_samples (int): Number of samples to store in deque.
            pub_fail (Publisher): ROS 2 publisher to send fail messages.
            logger (Logger): ROS 2 logger for logging messages.
            lstm_model: Pre-trained LSTM model for collision detection.
            encoder: OneHotEncoder for decoding model predictions.
        """
        self.collision_jerk = collision_jerk
        self.imu_msgs_deque = deque(maxlen=n_samples)
        self.accel_deque = deque(maxlen=n_samples)
        self.accel_deque_x = deque(maxlen=n_samples)
        self.accel_deque_y = deque(maxlen=n_samples)
        self.accel_deque_z = deque(maxlen=n_samples)
        self.jerk_deque = deque(maxlen=n_samples)
        self.pub_fail = pub_fail
        self.logger = logger
        self.lstm_model = lstm_model  # Pre-trained LSTM model
        self.encoder = encoder  # OneHotEncoder for decoding labels
        self.required_samples = n_samples  # Required number of samples for the model

    def imu_cb(self, msg: Imu):
        """
        Process IMU data for collision detection using an LSTM model.
        """
        # Append the current acceleration value to the deque
        self.imu_msgs_deque.appendleft(msg)
        self.accel_deque_x.appendleft(msg.linear_acceleration.x)
        self.accel_deque_y.appendleft(msg.linear_acceleration.y)
        self.accel_deque_z.appendleft(msg.linear_acceleration.z)

        if len(self.accel_deque_x) >= 2:  # Need at least 2 samples to compute jerk
            dt = (
                headers2dt(self.imu_msgs_deque[1].header, self.imu_msgs_deque[0].header)
                / 1e9
            )  # Convert ns to seconds
            if dt > 0:
                # Calculate jerk for each axis
                jerk_x = (self.accel_deque_x[0] - self.accel_deque_x[1]) / dt
                jerk_y = (self.accel_deque_y[0] - self.accel_deque_y[1]) / dt
                jerk_z = (self.accel_deque_z[0] - self.accel_deque_z[1]) / dt

                # Calculate magnitude of jerk
                jerk_magnitude = (jerk_x**2 + jerk_y**2 + jerk_z**2) ** 0.5

                self.jerk_deque.appendleft(jerk_magnitude)

        if len(self.jerk_deque) >= self.required_samples:

            input_data = np.array(self.jerk_deque).reshape(1, -1, 1)

            # Make prediction
            predictions = self.lstm_model.predict(input_data)
            predicted_label = self.encoder.inverse_transform(predictions)[0]

            # Make second prediction
            predicted_label2 = classify_events_with_secondary_opinion(self.jerk_deque)

            result = process_both_methods(
                INFO_DICT.get(predicted_label), INFO_DICT(predicted_label2)
            )

            if result == 0:
                self.logger.warn(
                    f"Collision detected by model! Label: {predicted_label}"
                )
                fail_msg = Fails()
                fail_msg.type = "collision"
                fail_msg.severity = "high"
                fail_msg.message = "Collision detected by LSTM model."
                self.pub_fail.publish(fail_msg)

    def chassis_imu_cb(self, msg: Imu):
        """Process chassis IMU data for rollover detection."""
        # Extract orientation from the IMU message
        quat = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        roll, pitch, yaw = euler_from_quaternion(quat)

        # Check for rollover conditions
        if abs(roll) > 0.5 or abs(pitch) > 0.5:
            # Log and publish the rollover detection
            self.logger.warn(f"Rollover detected! Roll: {roll}, Pitch: {pitch}")
            fail_msg = Fails()
            fail_msg.type = "rollover"
            fail_msg.severity = "critical"
            fail_msg.message = f"Detected a rollover. Roll: {roll}, Pitch: {pitch}"
            self.pub_fail.publish(fail_msg)
