/*! @package fail_detection
    Code Information:
        Maintainer: Eng. Pedro Alejandro Gonzalez B
        Mail: pedro@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include <fail_detection_cpp/fail_detection.hpp>
#include <deque>
#include <cmath>

using std::placeholders::_1;

FailDetector::FailDetector(rclcpp::NodeOptions &options) : Node("fail_detector", options)
{
    // Parámetros
    this->declare_parameter<float>("collision_jerk", 400.0);
    this->declare_parameter<int>("n_samples", 100);

    this->get_parameter("collision_jerk", m_collision_jerk);
    this->get_parameter("n_samples", m_n_samples);

    // Configuración de las colas
    m_accel_x = std::deque<float>(m_n_samples, 0.0f);
    m_accel_y = std::deque<float>(m_n_samples, 0.0f);
    m_accel_z = std::deque<float>(m_n_samples, 0.0f);
    m_jerk_values = std::deque<float>(m_n_samples, 0.0f);

    // Subscriptores
    auto qos = rclcpp::QoS(1).keep_last(1);
    m_subs_imu_camera = this->create_subscription<sensor_msgs::msg::Imu>(
        "/camera/imu", qos, std::bind(&FailDetector::ImuCb, this, _1));
    m_subs_imu_chassis = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", qos, std::bind(&FailDetector::ChassisImuCb, this, _1));

    // Publicadores
    m_pub_fail = this->create_publisher<usr_msgs::msg::Fails>("/fail_detection/fail", qos);

    RCLCPP_INFO(this->get_logger(), "FailDetector node initialized successfully :)!");
}

void FailDetector::ImuCb(sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Agregar nueva aceleración
    m_accel_x.push_front(msg->linear_acceleration.x);
    m_accel_y.push_front(msg->linear_acceleration.y);
    m_accel_z.push_front(msg->linear_acceleration.z);

    // Remover datos antiguos
    m_accel_x.pop_back();
    m_accel_y.pop_back();
    m_accel_z.pop_back();

    // Calcular el jerk si hay suficientes datos
    if (m_accel_x.size() >= 2)
    {
        float dt = 0.02; // Tiempo aproximado entre muestras (20 ms)
        float jerk_x = (m_accel_x[0] - m_accel_x[1]) / dt;
        float jerk_y = (m_accel_y[0] - m_accel_y[1]) / dt;
        float jerk_z = (m_accel_z[0] - m_accel_z[1]) / dt;

        float jerk_magnitude = std::sqrt(jerk_x * jerk_x + jerk_y * jerk_y + jerk_z * jerk_z);
        m_jerk_values.push_front(jerk_magnitude);
        m_jerk_values.pop_back();

        // Detectar colisiones
        if (jerk_magnitude > m_collision_jerk)
        {
            RCLCPP_WARN(this->get_logger(), "Collision detected! Jerk magnitude: %.2f", jerk_magnitude);
            usr_msgs::msg::Fails fail_msg;
            fail_msg.type = "collision";
            fail_msg.severity = "high";
            fail_msg.message = "Collision detected based on jerk magnitude.";
            m_pub_fail->publish(fail_msg);
        }
    }
}

void FailDetector::ChassisImuCb(sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Calcular roll y pitch a partir del quaternion
    tf2::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Evaluar estados basados en roll y pitch
    std::string state = evaluateRollover(roll, pitch);

    if (state == "roll over")
    {
        RCLCPP_WARN(this->get_logger(), "Rollover detected! Roll: %.2f, Pitch: %.2f", roll, pitch);
        usr_msgs::msg::Fails fail_msg;
        fail_msg.type = "rollover";
        fail_msg.severity = "critical";
        fail_msg.message = "Detected a rollover.";
        m_pub_fail->publish(fail_msg);
    }
    else if (state == "pronounced")
    {
        RCLCPP_INFO(this->get_logger(), "Pronounced movement detected. Roll: %.2f, Pitch: %.2f", roll, pitch);
    }
}

std::string FailDetector::evaluateRollover(double roll, double pitch)
{
    if (std::abs(roll) > 1.0 || std::abs(pitch) > 1.0)
        return "roll over";
    else if (std::abs(roll) > 0.4 || std::abs(pitch) > 0.4)
        return "pronounced";
    return "normal";
}