/**
 *@file mobility_control_node_usb.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-10-01
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "serialib.h"

class ICommunicationProtocol {
public:
    virtual ~ICommunicationProtocol() = default;

    virtual bool open(const std::string& dev) = 0;

    virtual int write_bytes(void* buffer, const unsigned int n_bytes) = 0;

    virtual int read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) = 0;

    virtual int write_string(const std::string& msg) = 0;

    virtual std::string read_string(std::chrono::milliseconds timeout) = 0;
};

class UsbProtocol : public ICommunicationProtocol {
public:
    UsbProtocol() = default;

    bool open(const std::string& port) override {
        char errorOpening = serial_.openDevice(port.c_str(), 115200);
        if (errorOpening != 1) {
            return false;
        }
        return true;
    }

    int write_bytes(void* buffer, const unsigned int n_bytes) override {
        unsigned num_bytes = 0;
        int ret = serial_.writeBytes(buffer, n_bytes, &num_bytes);
        if (ret == 1) {
            return (int) num_bytes;
        } else {
            return 0;
        }
    }

    int read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) override {
        int ret = serial_.readBytes(buffer, max_length, timeout.count());
        if (ret >= 0) {
            return ret;
        } else {
            return 0;
        }
    }

    int write_string(const std::string& msg) override {
        int ret = serial_.writeString(msg.c_str());
        return ret;
    }

    std::string read_string(std::chrono::milliseconds timeout) override {
        int numChars = serial_.readString(read_buffer_, '\n', 1023, timeout.count());
        if (numChars == 0) {
            return std::string();
        } else if (numChars > 0) {
            return std::string(read_buffer_, numChars);
        } else {
            throw std::runtime_error("USB Read Error");
        }
    }


    ~UsbProtocol() override {
        serial_.closeDevice();
    }

private:
    serialib serial_ {};
    char read_buffer_[1024] {};
};

enum MobilityCommands {
    set_velocity = 0,
    set_dutycycle = 1,
    set_maximum_dutycycle = 2,
    set_lowpass_fc = 3,
    set_pid_params = 4,
    set_pid_bounds = 5,
    set_ff_params = 6,
    set_max_velocity = 7,
    set_feedback_hz = 8,
    set_control_hz = 9,
    control_enable = 10
};

#pragma pack(push, 1) // no padding
struct MobilityCommandPacket {
    int32_t command_id;   // 0: set velocity, 1: set PWM, 2: set PID, etc.
    float value1;         // depends on command
    float value2;
    float value3;
    float value4;         // optional
};
#pragma pack(pop)

#pragma pack(push, 1) // no padding
struct MobilityFeedbackPacket {
    int32_t positions[4];
    int32_t velocities[4];
    int32_t pwm_duties[4];
};
#pragma pack(pop)

struct MobilityFeedback {
    int positions[4];
    int velocities[4];
    int target_velocities[4];
    int pwm_duties[4];
    int duration_us;
};

class MobilityControlNode : public rclcpp::Node {

public:
    MobilityControlNode() : Node("mobility_control_node") {
        cmd_vel_topic_ = this->declare_parameter("cmd_vel_topic", cmd_vel_topic_);
        joint_states_topic_ = this->declare_parameter("joint_states_topic", joint_states_topic_);
        odom_topic_ = this->declare_parameter("odom_topic", odom_topic_);

        wheel_joint_names_ = this->declare_parameter("wheel_joint_names", wheel_joint_names_);

        wheel_separation_ = this->declare_parameter("wheel_separation", wheel_separation_);
        wheel_radius_ = this->declare_parameter("wheel_radius", wheel_radius_);
        wheel_reduction_ = this->declare_parameter("wheel_reduction", wheel_reduction_);
        wheel_encoder_cpr_ = this->declare_parameter("wheel_encoder_cpr", wheel_encoder_cpr_);

        max_pwm_dutycycle_ = this->declare_parameter("max_pwm_dutycycle", max_pwm_dutycycle_);
        velocity_filter_cutoff_hz_ = this->declare_parameter("velocity_filter_cutoff_hz", velocity_filter_cutoff_hz_);

        pid_kp_ = this->declare_parameter("pid_kp", pid_kp_);
        pid_ki_ = this->declare_parameter("pid_ki", pid_ki_);
        pid_kd_ = this->declare_parameter("pid_kd", pid_kd_);

        pid_p_bound_ = this->declare_parameter("pid_p_bound", pid_p_bound_);
        pid_i_bound_ = this->declare_parameter("pid_i_bound", pid_i_bound_);
        pid_d_bound_ = this->declare_parameter("pid_d_bound", pid_d_bound_);

        base_frame_id_ = this->declare_parameter("base_frame_id", base_frame_id_);
        odom_frame_id_ = this->declare_parameter("odom_frame_id", odom_frame_id_);

        is_open_loop_ = this->declare_parameter("is_open_loop", is_open_loop_);
        cmd_vel_timeout_sec_ = this->declare_parameter("cmd_vel_timeout", cmd_vel_timeout_sec_);

        pose_covariance_diagonal_ = this->declare_parameter("pose_covariance_diagonal", pose_covariance_diagonal_);
        twist_covariance_diagonal_ = this->declare_parameter("twist_covariance_diagonal", twist_covariance_diagonal_);

        linear_x_has_velocity_limits_ = this->declare_parameter("linear.x.has_velocity_limits", linear_x_has_velocity_limits_);
        linear_x_max_velocity_ = this->declare_parameter("linear.x.max_velocity", linear_x_max_velocity_);
        linear_x_min_velocity_ = this->declare_parameter("linear.x.min_velocity", linear_x_min_velocity_);

        angular_z_has_velocity_limits_ = this->declare_parameter("angular.z.has_velocity_limits", angular_z_has_velocity_limits_);
        angular_z_max_velocity_ = this->declare_parameter("angular.z.max_velocity", angular_z_max_velocity_);
        angular_z_min_velocity_ = this->declare_parameter("angular.z.min_velocity", angular_z_min_velocity_);

        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
            std::bind(&MobilityControlNode::cmd_vel_callback, this, std::placeholders::_1));

        jointStatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default)));
        size_t jointCount = wheel_joint_names_.size();
        wheel_joint_states_.name.reserve(jointCount);
        wheel_joint_states_.position.reserve(jointCount);
        wheel_joint_states_.velocity.reserve(jointCount);
        wheel_joint_states_.name.resize(jointCount);
        wheel_joint_states_.position.resize(jointCount);
        wheel_joint_states_.velocity.resize(jointCount);
        for (size_t i = 0; i < wheel_joint_names_.size(); i++) {
            wheel_joint_states_.name[i] = wheel_joint_names_[i];
        }

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default)));


        protocol_type_ = this->declare_parameter("protocol", protocol_type_);
        serial_port_ = this->declare_parameter("serial_port", serial_port_);

        if (protocol_type_ == "usb") {
            protocol_ = std::make_unique<UsbProtocol>();
            device_ = serial_port_;
        } else {
            RCLCPP_FATAL(this->get_logger(), "Unsupported protocol: %s", protocol_type_.c_str());
            throw std::runtime_error("Unsupported protocol");
        }

        while (!protocol_->open(device_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open USB device %s", device_.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        RCLCPP_INFO(this->get_logger(), "Connected to %s", device_.c_str());

        // Make shared state
        odom_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(1000.0 / feedback_rate_hz_)),
            std::bind(&MobilityControlNode::publish_odometry_and_joint_states,
                this));
        controller_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(1000.0 / cmd_vel_publish_rate_hz)),
            std::bind(&MobilityControlNode::command_controller,
                this));

        RCLCPP_INFO(this->get_logger(), "Initializing controller...");
        while (!init_motor_controller()) {
            RCLCPP_ERROR(this->get_logger(), "Could not initialize controller!");
        }
    }
private:

    enum WheelIndex {
        front_left = 0,
        front_right = 1,
        back_left = 2,
        back_right = 3
    };

    // --- USB async ---
    std::unique_ptr<ICommunicationProtocol> protocol_;
    std::thread usb_thread_;
    std::mutex usb_mutex_;
    std::condition_variable usb_cv_;
    std::queue<std::vector<uint8_t>> usb_write_queue_;
    bool usb_ready_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_ {};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_ {};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_ {};
    rclcpp::TimerBase::SharedPtr odom_timer_ {};
    rclcpp::TimerBase::SharedPtr controller_timer_ {};
    sensor_msgs::msg::JointState wheel_joint_states_ {};


    std::string protocol_type_ { "usb" };

    std::string cmd_vel_topic_ { "cmd_vel" };
    std::string odom_topic_ { "odom" };
    std::string joint_states_topic_ { "joint_states" };

    double wheel_separation_ = 1.0;
    double wheel_radius_ = 0.1;
    double wheel_reduction_ = 100;
    double wheel_encoder_cpr_ = 64;
    std::vector<double> wheel_encoder_velocities_steps_sec {
        0.0, 0.0, 0.0, 0.0
    };

    std::vector<std::string> wheel_joint_names_ {
        "front_left_wheel",
        "front_right_wheel",
        "back_right_wheel",
        "back_left_wheel"
    };
    std::string base_frame_id_ { "base_link" };
    std::string odom_frame_id_ { "odom" };

    std::vector<double> pose_covariance_diagonal_ { 0.002, 0.002, 0.0, 0.0, 0.0, 0.02 };
    std::vector<double> twist_covariance_diagonal_ { 0.002, 0.0, 0.0, 0.0, 0.0, 0.02 };

    double max_pwm_dutycycle_ { 80.0 };
    double velocity_filter_cutoff_hz_ { 100.0 };

    double pid_kp_ { 1.0 };
    double pid_ki_ { 0.01 };
    double pid_kd_ { 0.0001 };

    double pid_p_bound_ { 100.0f };
    double pid_i_bound_ { 50.0f };
    double pid_d_bound_ { 50.0f };

    double feedback_rate_hz_ = 200.0;
    double cmd_vel_publish_rate_hz = 50.0;

    double cmd_vel_timeout_sec_ = 0.5;

    bool is_open_loop_ = true;

    bool linear_x_has_velocity_limits_ = false;
    double linear_x_max_velocity_ = 1.0;
    double linear_x_min_velocity_ = -1.0;

    bool angular_z_has_velocity_limits_ = false;
    double angular_z_max_velocity_ = 1.0;
    double angular_z_min_velocity_ = -1.0;

    std::string serial_port_ { "/dev/ttyACM0" };
    std::string device_ { "/dev/ttyACM0" };

    rclcpp::Time prev_cmd_vel_time_ { this->get_clock()->now() };
    rclcpp::Time prev_odom_time_ { this->get_clock()->now() };
    geometry_msgs::msg::Twist cmd_vel_;
    double heading_ {};
    double pos_x_ {};
    double pos_y_ {};

    MobilityFeedback feedback_ {};

    // Updated USB thread loop
    void start_usb_thread() {
        usb_thread_ = std::thread([this]() {
            while (!protocol_->open(serial_port_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open USB %s", serial_port_.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            usb_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "USB Connected to %s", serial_port_.c_str());
            RCLCPP_INFO(this->get_logger(), "Initializing controller...");
            while (!init_motor_controller()) {
                RCLCPP_ERROR(this->get_logger(), "Could not initialize controller!");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            while (rclcpp::ok()) {
                std::unique_lock<std::mutex> lock(usb_mutex_);
                usb_cv_.wait(lock, [this]() { return !usb_write_queue_.empty(); });
                while (!usb_write_queue_.empty()) {
                    std::vector<uint8_t> packet = std::move(usb_write_queue_.front());
                    usb_write_queue_.pop();
                    lock.unlock();
                    try {
                        protocol_->write_bytes(packet.data(), packet.size());
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "USB write failed: %s", e.what());
                    }
                    lock.lock();
                }
            }
        });
        usb_thread_.detach();
    }

    void send_usb_command(const void* data, size_t length) {
        if (!usb_ready_) return;

        std::vector<uint8_t> packet((uint8_t*) data, (uint8_t*) data + length);
        {
            std::lock_guard<std::mutex> lock(usb_mutex_);
            usb_write_queue_.push(std::move(packet));
        }
        usb_cv_.notify_one();
    }

    // Send a MobilityCommandPacket over USB with logging
    bool send_usb_command_packet(const MobilityCommandPacket& packet) {
        int ret = protocol_->write_bytes((void*) &packet, sizeof(packet));
        if (ret == sizeof(packet)) {
            return true;
        } else {
            return false;
        }
    }

    // Send target wheel velocities
    void send_target_velocity_bytes(const std::array<float, 4>& target_velocities) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::set_velocity;
        pkt.value1 = target_velocities[0];
        pkt.value2 = target_velocities[1];
        pkt.value3 = target_velocities[2];
        pkt.value4 = target_velocities[3];
        if (!send_usb_command_packet(pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send target velocities to device");
        }
    }

    // Send PWM duty cycles
    void send_pwm_duty_bytes(const std::array<float, 4>& duties) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::set_dutycycle;
        pkt.value1 = duties[0];
        pkt.value2 = duties[1];
        pkt.value3 = duties[2];
        pkt.value4 = duties[3];
        if (!send_usb_command_packet(pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send PWM dutycycles to device");
        }
    }

    // Send maximum PWM duty
    void send_max_pwm_duty_bytes(float max_pwm_duty) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::set_maximum_dutycycle;
        pkt.value1 = max_pwm_duty;
        if (!send_usb_command_packet(pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set maximum PWM dutycycle");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted maximum PWM dutycycle: %f", max_pwm_duty);
        }
    }

    // Send velocity filter cutoff
    void send_lowpass_cutoff_bytes(float fc) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::set_lowpass_fc;
        pkt.value1 = fc;

        if (!send_usb_command_packet(pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set velocity filter cutoff");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new velocity filter cutoff: %f Hz", fc);
        }
    }

    // Send PID parameters
    void send_pid_params_bytes(float kp, float ki, float kd) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::set_pid_params;
        pkt.value1 = kp;
        pkt.value2 = ki;
        pkt.value3 = kd;

        if (!send_usb_command_packet(pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set PID parameters");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new PID parameters: kp:%f, ki:%f, kd:%f", kp, ki, kd);
        }
    }

    // Send PID bounds
    void send_pid_bounds_bytes(float p_bound, float i_bound, float d_bound) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::set_pid_bounds;
        pkt.value1 = p_bound;
        pkt.value2 = i_bound;
        pkt.value3 = d_bound;

        if (!send_usb_command_packet(pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set PID boundaries");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new PID boundaries: p:%f, i:%f, d:%f", p_bound, i_bound, d_bound);
        }
    }

    // Send feedforward parameters
    void send_feedforward_params_bytes(float kv, float voff) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::set_ff_params;
        pkt.value1 = kv;
        pkt.value2 = 0.0f; // ka
        pkt.value3 = 0.0f; // kj
        pkt.value4 = voff;

        if (!send_usb_command_packet(pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Feed Forward parameters");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new Feed Forward parameters: kv:%f, v_off:%f", kv, voff);
        }
    }

    // Send maximum velocity
    void send_max_velocity_bytes(float max_vel) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::set_max_velocity;
        pkt.value1 = max_vel;
        send_usb_command_packet(pkt);
        RCLCPP_INFO(this->get_logger(), "Setted new maximum velocity: %.3f", max_vel);
    }

    // Send feedback rate
    void send_feedback_rate_bytes(float feedback_rate_hz) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::set_feedback_hz;
        pkt.value1 = feedback_rate_hz;

        if (!send_usb_command_packet(pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to feedback rate");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new feedback rate: %f Hz", feedback_rate_hz);
        }
    }

    // Send motor control rate
    void send_motor_control_rate_bytes(float motor_control_rate_hz) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::set_control_hz;
        pkt.value1 = motor_control_rate_hz;
        if (!send_usb_command_packet(pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set motor control rate");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new motor control rate: %f Hz", motor_control_rate_hz);
        }
    }

    // Start / Stop controller
    void send_start_stop_bytes(bool start) {
        MobilityCommandPacket pkt {};
        pkt.command_id = MobilityCommands::control_enable;
        pkt.value1 = start ? 1.0f : 0.0f;
        if (!send_usb_command_packet(pkt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start controller");
        } else {
            RCLCPP_INFO(this->get_logger(), "Started controller");
        }
    }

    bool receive_motor_feedback_bytes(MobilityFeedback* feedback) {
        if (!feedback) return false;

        MobilityFeedbackPacket packet {};
        int bytes_received = protocol_->read_bytes(&packet, sizeof(packet), std::chrono::milliseconds(10));

        if (bytes_received != sizeof(packet)) {
            RCLCPP_WARN(this->get_logger(), "Incomplete motor feedback received: %d/%zu bytes", bytes_received, sizeof(packet));
            return false;
        }

        // Copy data from the received packet to the feedback structure
        for (int i = 0; i < 4; i++) {
            feedback->positions[i] = packet.positions[i];
            feedback->velocities[i] = packet.velocities[i];
            feedback->pwm_duties[i] = packet.pwm_duties[i];
        }

        // Keep logs
        RCLCPP_INFO(this->get_logger(),
            "Velocity Feedback: V1:%d, V2:%d, V3:%d, V4:%d",
            feedback->velocities[0], feedback->velocities[1], feedback->velocities[2], feedback->velocities[3]);
        RCLCPP_INFO(this->get_logger(),
            "Position Feedback: P1:%d, P2:%d, P3:%d, P4:%d",
            feedback->positions[0], feedback->positions[1], feedback->positions[2], feedback->positions[3]);
        RCLCPP_INFO(this->get_logger(),
            "PWM Duties: D1:%d, D2:%d, D3:%d, D4:%d",
            feedback->pwm_duties[0], feedback->pwm_duties[1], feedback->pwm_duties[2], feedback->pwm_duties[3]);

        return true;
    }


    void cmd_vel_callback(const geometry_msgs::msg::Twist& msg) {
        prev_cmd_vel_time_ = this->get_clock()->now();
        if (linear_x_has_velocity_limits_) {
            cmd_vel_.linear.x = std::clamp(msg.linear.x, linear_x_min_velocity_, linear_x_max_velocity_);
        } else {
            cmd_vel_.linear.x = msg.linear.x;
        }
        if (angular_z_has_velocity_limits_) {
            cmd_vel_.angular.z = std::clamp(msg.angular.z, angular_z_min_velocity_, angular_z_max_velocity_);
        } else {
            cmd_vel_.angular.z = msg.angular.z;
        }

    }

    void publish_odometry_and_joint_states() {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - prev_odom_time_).seconds();

        if (receive_motor_feedback_bytes(&feedback_)) {
            for (int i = 0; i < 4; i++) {
                wheel_encoder_velocities_steps_sec[i] = feedback_.velocities[i];
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Received Feedback: None");
        }

        if ((current_time - prev_cmd_vel_time_).seconds() > cmd_vel_timeout_sec_) {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = 0.0;
            cmd_vel_.angular.x = 0.0;
            cmd_vel_.angular.y = 0.0;
            cmd_vel_.angular.z = 0.0;
        }

        prev_odom_time_ = current_time;

        double robot_linear_velocity = 0;
        double robot_angular_velocity = 0;
        double left_side_velocity = 0;
        double right_side_velocity = 0;
        std::vector<double>wheel_angular_velocities { 0.0, 0.0, 0.0, 0.0 };

        if (is_open_loop_) {

            robot_linear_velocity = cmd_vel_.linear.x;
            robot_angular_velocity = cmd_vel_.angular.z;

            left_side_velocity = robot_linear_velocity - (wheel_separation_ / 2.0) * robot_angular_velocity;
            right_side_velocity = robot_linear_velocity + (wheel_separation_ / 2.0) * robot_angular_velocity;

            wheel_angular_velocities[front_left] = left_side_velocity / wheel_radius_;
            wheel_angular_velocities[back_left] = left_side_velocity / wheel_radius_;
            wheel_angular_velocities[front_right] = right_side_velocity / wheel_radius_;
            wheel_angular_velocities[back_right] = right_side_velocity / wheel_radius_;

        } else {
            for (size_t i = 0; i < wheel_encoder_velocities_steps_sec.size(); i++) {
                double motor_rotations_per_sec = wheel_encoder_velocities_steps_sec[i] / (wheel_encoder_cpr_ * wheel_reduction_);
                wheel_angular_velocities[i] = motor_rotations_per_sec * 2.0 * M_PI;
            }

            left_side_velocity =
                (wheel_angular_velocities[front_left] + wheel_angular_velocities[back_left]) / 2.0;
            right_side_velocity =
                (wheel_angular_velocities[front_right] + wheel_angular_velocities[back_right]) / 2.0;

            robot_linear_velocity = (left_side_velocity + right_side_velocity) / 2.0;
            robot_angular_velocity = (right_side_velocity - left_side_velocity) / wheel_separation_;
        }

        double delta_heading = robot_angular_velocity * dt;
        double delta_x = (robot_linear_velocity * cos(heading_)) * dt;
        double delta_y = (robot_linear_velocity * sin(heading_)) * dt;

        pos_x_ += delta_x;
        pos_y_ += delta_y;
        heading_ += delta_heading;

        // https://github.com/linorobot/linorobot/blob/master/src/lino_base.cpp

        tf2::Quaternion odomQuat {};
        odomQuat.setRPY(0, 0, heading_);

        nav_msgs::msg::Odometry odomMsg {};
        odomMsg.header.stamp = current_time;
        odomMsg.header.frame_id = odom_frame_id_;
        odomMsg.child_frame_id = base_frame_id_;

        odomMsg.pose.pose.position.x = pos_x_;
        odomMsg.pose.pose.position.y = pos_y_;
        odomMsg.pose.pose.position.z = 0.0;
        odomMsg.pose.pose.orientation.x = odomQuat.x();
        odomMsg.pose.pose.orientation.y = odomQuat.y();
        odomMsg.pose.pose.orientation.z = odomQuat.z();
        odomMsg.pose.pose.orientation.w = odomQuat.w();


        odomMsg.twist.twist.linear.x = robot_linear_velocity;
        odomMsg.twist.twist.linear.y = 0.0;
        odomMsg.twist.twist.linear.z = 0.0;
        odomMsg.twist.twist.angular.x = 0.0;
        odomMsg.twist.twist.angular.y = 0.0;
        odomMsg.twist.twist.angular.z = robot_angular_velocity;

        for (size_t i = 0; i < pose_covariance_diagonal_.size(); i++) {
            if (i * 7 < odomMsg.pose.covariance.size()) {
                odomMsg.pose.covariance[i * 7] = pose_covariance_diagonal_[i];
                odomMsg.twist.covariance[i * 7] = twist_covariance_diagonal_[i];
            }
        }
        odom_publisher_->publish(odomMsg);


        wheel_joint_states_.header.frame_id = base_frame_id_;
        wheel_joint_states_.header.stamp = current_time;
        for (size_t i = 0; i < wheel_joint_names_.size(); i++) {
            wheel_joint_states_.velocity[i] = wheel_angular_velocities[i];
            if (is_open_loop_) {
                wheel_joint_states_.position[i] += wheel_joint_states_.velocity[i] * dt;
            } else {
                wheel_joint_states_.position[i] = feedback_.positions[i] * 2.0 * M_PI / (wheel_encoder_cpr_ * wheel_reduction_);
            }
        }
        jointStatePublisher_->publish(wheel_joint_states_);

    }

    void command_controller() {
        std::array<float, 4> wheel_velocities { 0, 0, 0, 0 };

        double v = cmd_vel_.linear.x;     // robot linear velocity
        double w = cmd_vel_.angular.z;    // robot angular velocity

        // Differential drive kinematics
        double left_vel = v - (wheel_separation_ / 2.0) * w;
        double right_vel = v + (wheel_separation_ / 2.0) * w;

        if (is_open_loop_) {
            double max_velocity = sqrt(linear_x_max_velocity_ * linear_x_max_velocity_ + angular_z_max_velocity_ * angular_z_max_velocity_);
            wheel_velocities[front_left] = (left_vel / max_velocity) * 100.0;
            wheel_velocities[back_left] = (left_vel / max_velocity) * 100.0;
            wheel_velocities[front_right] = (right_vel / max_velocity) * 100.0;
            wheel_velocities[back_right] = (right_vel / max_velocity) * 100.0;
            send_pwm_duty_bytes(wheel_velocities);
        } else {
            wheel_velocities[front_left] = left_vel * wheel_reduction_ * wheel_encoder_cpr_ / (2.0 * M_PI * wheel_radius_);
            wheel_velocities[back_left] = left_vel * wheel_reduction_ * wheel_encoder_cpr_ / (2.0 * M_PI * wheel_radius_);
            wheel_velocities[front_right] = right_vel * wheel_reduction_ * wheel_encoder_cpr_ / (2.0 * M_PI * wheel_radius_);
            wheel_velocities[back_right] = right_vel * wheel_reduction_ * wheel_encoder_cpr_ / (2.0 * M_PI * wheel_radius_);
            send_target_velocity_bytes(wheel_velocities);
        }

    }

    bool init_motor_controller() {
        try {
            send_start_stop_bytes(true);

            send_max_pwm_duty_bytes(max_pwm_dutycycle_);
            send_pid_params_bytes(pid_kp_, pid_ki_, pid_kd_);
            send_pid_bounds_bytes(pid_p_bound_, pid_i_bound_, pid_d_bound_);
            send_lowpass_cutoff_bytes(velocity_filter_cutoff_hz_);
            send_feedback_rate_bytes(feedback_rate_hz_);
            return true;
        } catch (std::runtime_error& err) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data to device");
            return false;
        }
    }

    void send_target_velocity(std::array<float, 4> target_velocities) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "TargetVels:%.3f,%.3f,%.3f,%.3f\n",
            target_velocities[0],
            target_velocities[1],
            target_velocities[2],
            target_velocities[3]);
        int ret = protocol_->write_string(std::string(buffer, len));
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send target velocities to device");
        }
    }

    void send_pwm_duty(std::array<float, 4> duties) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "PwmDuties:%.3f,%.3f,%.3f,%.3f\n",
            duties[0],
            duties[1],
            duties[2],
            duties[3]);
        int ret = protocol_->write_string(std::string(buffer, len));
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send dutycycles to device");
        }
    }

    void send_max_pwm_duty(float max_pwm_duty) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "MaxPwmDuty:%.3f\r\n", max_pwm_duty);
        int ret = protocol_->write_string(std::string(buffer, len));
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set maximum PWM dutycycle");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted maximum PWM dutycycle: %f", max_pwm_duty);
        }
    }

    void send_lowpass_cutoff(float fc) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "fc:%.3f\r\n", fc);
        int ret = protocol_->write_string(std::string(buffer, len));
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set velocity filter cutoff");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new velocity filter cutoff: %f Hz", fc);
        }
    }

    void send_pid_params(float kp, float ki, float kd) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "kp:%.3f,ki:%.3f,kd:%.3f\r\n", kp, ki, kd);
        int ret = protocol_->write_string(std::string(buffer, len));
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set PID parameters");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new PID parameters: kp:%f, ki:%f, kd:%f", kp, ki, kd);
        }
    }

    void send_pid_bounds(float p_bound, float i_bound, float d_bound) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "pB:%.3f,iB:%.3f,dB:%.3f\r\n", p_bound, i_bound, d_bound);
        int ret = protocol_->write_string(std::string(buffer, len));
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set PID boundaries");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new PID boundaries: p:%f, i:%f, d:%f", p_bound, i_bound, d_bound);
        }
    }

    void send_feedforward_params(float kv, float voff) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "kv:%.3f,ka:%.3f,kj:%.3f,voff:%.3f\r\n", kv, 0.0f, 0.0f, voff);
        int ret = protocol_->write_string(std::string(buffer, len));
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Feed Forward parameters");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new Feed Forward parameters: kv:%f, v_off:%f", kv, voff);
        }
    }

    void send_max_velocity(float max_vel) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "MaxVel:%.3f\r\n", max_vel);
        protocol_->write_string(std::string(buffer, len));
    }

    void send_feedback_rate(float feedback_rate_hz) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "FeedbackRate:%.3f\r\n", feedback_rate_hz);
        int ret = protocol_->write_string(std::string(buffer, len));
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to feedback rate");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new feedback rate: %f Hz", feedback_rate_hz);
        }
    }

    void send_motor_control_rate(float motor_control_rate_hz) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "ControlRate:%.3f\r\n", motor_control_rate_hz);
        int ret = protocol_->write_string(std::string(buffer, len));
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set motor control rate");
        } else {
            RCLCPP_INFO(this->get_logger(), "Setted new motor control rate: %f Hz", motor_control_rate_hz);
        }
    }

    void send_start_stop(bool start) {
        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "Start:%d\r\n", start ? 1 : 0);
        int ret = protocol_->write_string(std::string(buffer, len));
        if (ret != 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start controller");
        } else {
            RCLCPP_INFO(this->get_logger(), "Started controller");
        }
    }

    bool receive_motor_feedback(MobilityFeedback* feedback) {
        try {
            std::string frame = protocol_->read_string(std::chrono::milliseconds(10));
            const char* header = "FRAME_START,";
            if (frame.rfind(header, 0) == 0) {
                if (frame.size() > strlen(header)) {
                    frame = frame.substr(strlen(header));
                } else {
                    // Empty frame after header
                    return false;
                }
            }
            std::vector<std::string> tokens;
            std::stringstream ss(frame);
            std::string item;

            while (std::getline(ss, item, ';')) {
                if (item.find("i:") == 0) {
                    int idx, pos, refVel, vel, duty;
                    if (sscanf(item.c_str(), "i:%d,Pos:%d,RefVel:%d,Vel:%d,Duty:%d",
                        &idx, &pos, &refVel, &vel, &duty) == 5) {
                        feedback->positions[idx] = pos;
                        feedback->target_velocities[idx] = refVel;
                        feedback->velocities[idx] = vel;
                        feedback->pwm_duties[idx] = duty;
                    }
                } else if (item.find("Duration:") == 0) {
                    int duration;
                    sscanf(item.c_str(), "Duration:%d", &duration);
                    feedback->duration_us = duration;
                }
            }
            return true;
        } catch (std::runtime_error& err) {
            RCLCPP_ERROR(this->get_logger(), "USB read failed: %s", err.what());
            return false;
        }
        return false;
    }

};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MobilityControlNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}