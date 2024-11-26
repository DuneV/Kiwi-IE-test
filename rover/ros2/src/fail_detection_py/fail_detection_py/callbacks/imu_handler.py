#!/usr/bin/env python3

import os
import numpy as np
from threading import Thread
from queue import Queue
from collections import deque
from sensor_msgs.msg import Imu
import time
from threading import Thread
from tf_transformations import euler_from_quaternion
from usr_msgs.msg import Fail, Fails
from fail_detection_py.utils.time_utils import headers2dt
from fail_detection_py.utils.math_utils import (
    process_both_methods,
    classify_event_with_threshold,
)

INFO_DICT = {"Collision": 0, "Bump": 1, "Normal": 2}
ACCEPTED_MODEL = 1000


class ImuHandler:
    def __init__(self, _n_samples, _pub_fail, _logger, _lstm_model, _encoder, _node):
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
        self.imu_msgs_deque = deque(maxlen=2)
        self.jerk_deque_x = deque(maxlen=_n_samples)
        self.jerk_deque_y = deque(maxlen=_n_samples)
        self.accel_deque_x = deque(maxlen=2)
        self.accel_deque_y = deque(maxlen=2)

        self.node = _node
        self.pub_fail = _pub_fail
        self.logger = _logger

        self.newtime = np.linspace(0, 2, ACCEPTED_MODEL)
        self.lstm_model = _lstm_model  # Pre-trained LSTM model
        self.input_details = self.lstm_model.get_input_details()
        self.output_details = self.lstm_model.get_output_details()
        self.encoder = _encoder  # OneHotEncoder for decoding labels
        self.required_samples = _n_samples  # Required number of samples for the model
        self.create_timer()
        self.alert_state = None  # flag to order the priorities ->

        # Start a background thread for processing jerk calculations
        self.processing_thread = Thread(target=self.process_jerk_data)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def create_timer(self):
        """
        Created a timer to config the callback
        """
        self.timer = self.node.create_timer(1.0, self.evaluate_model)

    def imu_cb(self, msg: Imu):
        """
        Process IMU data for collision detection using an LSTM model.
        """
        # Add incoming IMU data to the deque (acceleration values)
        self.imu_msgs_deque.appendleft(msg)
        self.accel_deque_x.appendleft(msg.linear_acceleration.x)
        self.accel_deque_y.appendleft(msg.linear_acceleration.y)

        # Just store data, avoid computation here
        if len(self.accel_deque_x) >= 2:
            self.logger.debug("IMU data received, calculating jerk in background.")
            self.jerk_deque_x.appendleft(self.accel_deque_x[0])
            self.jerk_deque_y.appendleft(self.accel_deque_y[0])

    def process_jerk_data(self):
        """
        Process jerk data in the background to avoid blocking the main callback.
        """
        while True:
            time.sleep(0.1)  # Sleep to avoid constant checking

            if len(self.accel_deque_x) >= 2 and len(self.jerk_deque_x) >= 2:
                # Calculate jerk based on acceleration differences
                dt = (
                    headers2dt(
                        self.imu_msgs_deque[1].header, self.imu_msgs_deque[0].header
                    )
                    / 1e9
                )  # Convert ns to seconds

                if dt > 0:
                    dt = round(dt, 2)
                    jerk_x = (self.accel_deque_x[0] - self.accel_deque_x[1]) / dt
                    jerk_y = (self.accel_deque_y[0] - self.accel_deque_y[1]) / dt

                    # Store the jerk values
                    self.jerk_deque_x.appendleft(jerk_x)
                    self.jerk_deque_y.appendleft(jerk_y)

    def chassis_imu_cb(self, msg: Imu):
        """
        Process chassis IMU data for rollover detection based on roll and pitch.

        Args:
            msg (Imu): Incoming IMU message containing orientation data.
        """
        # Extract orientation from the IMU message
        quat = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        roll, pitch, yaw = euler_from_quaternion(quat)

        # Evaluate the state based on roll and pitch
        state = self.evaluate_rollover(roll, pitch)

        if state == "roll over":
            # Log and publish a critical fail for roll over
            self.logger.warn(f"Rollover detected! Roll: {roll}, Pitch: {pitch}")
            fail_msg = Fails()
            fail_item = Fail()
            fail_item.event = "rollover"
            fail_item.confidence = 98
            fail_item.message = f"Detected a rollover. Roll: {roll}, Pitch: {pitch}"
            fail_msg.fails.append(fail_item)
            self.pub_fail.publish(fail_msg)
        elif state == "stuck":
            # Log a warning for a pronounced event
            self.logger.warn(
                f"Pronounced movement detected. Roll: {roll}, Pitch: {pitch}"
            )
            fail_msg = Fails()
            fail_item = Fail()
            fail_item.event = "stuck"
            fail_item.confidence = 75
            fail_item.message = (
                f"Detected a stuck on a camber. Roll: {roll}, Pitch: {pitch}"
            )
            fail_msg.fails.append(fail_item)
            self.pub_fail.publish(fail_msg)
        else:
            # Log normal state
            self.logger.debug("Normal movement detected.")

    def evaluate_rollover(self, roll, pitch):
        """
        Evaluate the roll and pitch values to classify the state as:
        - 'roll over': If roll > 1 or roll < -1, or pitch > 1 or pitch < -1.
        - 'pronounced': If roll > 0.4 or roll < -0.4, or pitch > 0.4 or pitch < -0.4.
        - 'normal': Otherwise.

        Args:
            roll (float): Roll value in radians.
            pitch (float): Pitch value in radians.

        Returns:
            str: The classified state ('roll over', 'pronounced', or 'normal').
        """
        if abs(roll) > 1 or abs(pitch) > 1:
            return "roll over"
        elif abs(roll) > 0.4 or abs(pitch) > 0.4:
            return "stuck"
        else:
            return "normal"

    def evaluate_model(self):
        """
        Evaluate the model every 2 seconds using the accumulated data.
        """
        if len(self.jerk_deque_y) >= self.required_samples:
            #     # Process jerk and make prediction with the LSTM model
            total_time = (
                headers2dt(
                    self.imu_msgs_deque[-1].header, self.imu_msgs_deque[0].header
                )
                / 1e9
            )
            time_step = np.linspace(0, total_time, len(self.jerk_deque_y))
            interpolated_jerk = np.interp(self.newtime, time_step, self.jerk_deque_y)
            input_data = np.array(interpolated_jerk).reshape(1, -1, 1)
            input_data = input_data.astype(np.float32)
            self.lstm_model.set_tensor(self.input_details[0]["index"], input_data)
            self.lstm_model.invoke()
            output_data = self.lstm_model.get_tensor(self.output_details[0]["index"])[0]
            self.logger.info(f"Probabilities: {output_data}")
            if output_data[1] > 0.35:
                self.handler_message(
                    output_data[1], "collision", "Collision detected by model."
                )
            else:
                if (
                    classify_event_with_threshold(self.jerk_deque_y)
                    and output_data[1] > 0.3
                ):
                    self.handler_message(
                        output_data[1], "collision", "Collision detected by model."
                    )
                elif (
                    classify_event_with_threshold(self.jerk_deque_y) == 0
                    and output_data[2] < 0.6
                ):
                    self.handler_message(output_data[0], "bump", "Bump")
                else:
                    pass

    def handler_message(self, value, event, msg):
        """
        Handler of message to avoid repeat code
        Parameters:
            - value(uint): value of confidence
            - event (string): "collision", "bump", "roll"
            - msg(string): message of topic
        """
        fail_msg = Fails()
        fail_item = Fail()
        fail_item.event = event
        fail_item.confidence = int(value)
        fail_item.message = msg
        fail_msg.fails.append(fail_item)
        self.pub_fail.publish(fail_msg)
