/*! @package fail_detection
    Code Information:
        Maintainer: Eng. Pedro Alejandro Gonzalez B
        Mail: pedro@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <usr_msgs/msg/fails.hpp>
#include <utils/console.hpp>

/**
 * @class FailDetector
 * @brief This class implements a ROS 2 node to detect failures in the robot's behavior based on IMU and odometry data.
 *
 * The FailDetector class detects anomalies such as collisions and rollovers using IMU readings and robot speed data.
 * It uses deques to store and process the most recent samples for collision and rollover detection. 
 */
class FailDetector : public rclcpp::Node
{
   public:
    /**
     * @brief Constructor for the FailDetector class.
     *
     * Initializes the ROS 2 node, declares parameters, sets up subscribers and publishers, and configures internal variables.
     * @param options: rclcpp::NodeOptions for node initialization.
     */
    FailDetector(rclcpp::NodeOptions& options);

    /**
     * @brief Callback function for processing IMU messages from the camera sensor.
     *
     * This method analyzes IMU data to detect collisions based on jerk magnitude.
     * @param msg: Shared pointer to a sensor_msgs::msg::Imu message.
     */
    void ImuCb(sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief Callback function for processing IMU messages from the chassis sensor.
     *
     * This method detects rollovers by analyzing pitch and roll values derived from the IMU orientation data.
     * @param msg: Shared pointer to a sensor_msgs::msg::Imu message.
     */
    void ChassisImuCb(sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief Callback function for processing robot speed data from odometry messages.
     *
     * This method tracks the robot's motion state (forward or backward) and updates internal variables.
     * @param msg: Shared pointer to a nav_msgs::msg::Odometry message.
     */
    void BotSpeedCb(nav_msgs::msg::Odometry::SharedPtr msg);

   private:
    // ROS 2 Subscribers
    /** Subscriber for IMU messages from the camera sensor. */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_subs_imu_camera;

    /** Subscriber for IMU messages from the chassis sensor. */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_subs_imu_chassis;

    /** Subscriber for robot speed data from odometry messages. */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subs_bot_speed;

    // ROS 2 Publishers
    /** Publisher for failure detection messages. */
    rclcpp::Publisher<usr_msgs::msg::Fails>::SharedPtr m_pub_fail;

    // Timers
    // Add timers if needed in the future.
    //rclcpp::TimerBase::SharedPtr m_dummy_tmr;

    // Environment Variables
    /** Threshold for jerk magnitude to detect collisions. */
    float m_collision_jerk = getEnv("FAIL_DETECTION_COLLISION_JERK", 400.0f);

    /** Time in seconds to report missing IMU messages. */
    const int m_imu_no_msgs_report_time = getEnv("FAIL_DETECTION_IMU_NO_MSGS_REPORT_TIME", 5);

    /** Number of samples for collision detection. */
    const size_t m_n_samples_collision = getEnv("FAIL_DETECTION_COLLISION_SAMPLES", 5);

    /** Number of samples for rollover detection. */
    const size_t m_n_samples_fall = getEnv("FAIL_DETECTION_COLLISION_SAMPLES", 100);

    /** Number of samples for pitch analysis. */
    const size_t m_n_samples_pitch = getEnv("FAIL_DETECTION_COLLISION_SAMPLES", 15);

    /** Maximum number of samples to store for IMU analysis. */
    const size_t m_n_samples = std::max(m_n_samples_fall, m_n_samples_collision);

    // Class Variables
    /** Current speed of the robot. */
    double m_bot_speed = 0.0;

    /** Current motion state of the robot (e.g., "forward", "backwards"). */
    std::string m_motion_state = "forward";

    /** Deque to store recent IMU messages for analysis. */
    std::deque<sensor_msgs::msg::Imu> m_imu_msgs_deque = std::deque<sensor_msgs::msg::Imu>(m_n_samples);

    /** Deque to store recent acceleration values for collision detection. */
    std::deque<float> m_accel_deque = std::deque<float>(m_n_samples);

    /** Message object for failure reports. */
    usr_msgs::msg::Fails fails_msg;
};
