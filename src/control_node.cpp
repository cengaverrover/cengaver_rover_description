// Yigithan Yigit 2024

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rover_mobility_interfaces/msg/mobility_control.hpp"

struct RobotState {
    std::vector<std::string> left_wheel_names {};
    std::vector<std::string> right_wheel_names {};
    double left_side_velocity {};
    double right_side_velocity {};
    double linearVel_x = 0.0;
    double linearVel_y = 0.0;
    double angularVel_z = 0.0;
    double x_pos = 0.0;
    double y_pos = 0.0;
    double last_vel_time = 0.0;
    double vel_dt = 0.0;
    double heading = 0.0;

    std::mutex stateMutex;
};

class ControlNode : public rclcpp::Node {
public:
    ControlNode()
        : Node("control_node") {

        cmdVelTopic_ = this->declare_parameter("cmd_vel_topic", cmdVelTopic_);
        mobility_control_topic_ = this->declare_parameter("mobility_control_topic", mobility_control_topic_);
        jointStatesTopic_ = this->declare_parameter("joint_states_topic", jointStatesTopic_);
        odomTopic_ = this->declare_parameter("odom_topic", odomTopic_);

        robotState_.left_wheel_names = this->declare_parameter("left_wheel_names", robotState_.left_wheel_names);
        robotState_.right_wheel_names = this->declare_parameter("right_wheel_names", robotState_.right_wheel_names);

        wheel_separation_ = this->declare_parameter("wheel_separation_", 0.788);
        wheel_radius_ = this->declare_parameter("wheel_radius_", 0.11736);

        base_frame_id_ = this->declare_parameter("base_frame_id", base_frame_id_);
        odom_frame_id_ = this->declare_parameter("odom_frame_id", odom_frame_id_);

        open_loop_ = this->declare_parameter("open_loop", open_loop_);
        cmd_vel_timeout_ = this->declare_parameter("cmd_vel_timeout", cmd_vel_timeout_);

        pose_covariance_diagonal_ = this->declare_parameter("pose_covariance_diagonal_", pose_covariance_diagonal_);
        twist_covariance_diagonal_ = this->declare_parameter("twist_covariance_diagonal_", twist_covariance_diagonal_);

        linear_x_has_velocity_limits_ = this->declare_parameter("linear.x.has_velocity_limits", linear_x_has_velocity_limits_);
        linear_x_max_velocity_ = this->declare_parameter("linear.x.max_velocity", linear_x_max_velocity_);
        linear_x_min_velocity_ = this->declare_parameter("linear.x.min_velocity", linear_x_min_velocity_);

        angular_z_has_velocity_limits_ = this->declare_parameter("angular.z.has_velocity_limits", angular_z_has_velocity_limits_);
        angular_z_max_velocity_ = this->declare_parameter("angular.z.max_velocity", angular_z_max_velocity_);
        angular_z_min_velocity_ = this->declare_parameter("angular.z.min_velocity", angular_z_min_velocity_);

        cmdVelSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(cmdVelTopic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
            std::bind(&ControlNode::cmd_vel_callback, this, std::placeholders::_1));

        if (!open_loop_) {
            jointStateSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(jointStatesTopic_,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
                std::bind(&ControlNode::encoder_callback, this, std::placeholders::_1));
        } else {
            jointStatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(jointStatesTopic_,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default)));
            size_t jointCount = robotState_.left_wheel_names.size() + robotState_.right_wheel_names.size();
            jointStates_.name.reserve(jointCount);
            jointStates_.position.reserve(jointCount);
            jointStates_.velocity.reserve(jointCount);

            jointStates_.name.resize(jointCount);
            jointStates_.position.resize(jointCount);
            jointStates_.velocity.resize(jointCount);
            for (size_t i = 0; i < robotState_.left_wheel_names.size(); i++) {
                jointStates_.name[i * 2] = robotState_.left_wheel_names[i];
                jointStates_.name[i * 2 + 1] = robotState_.right_wheel_names[i];
            }
        }

        odomPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odomTopic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default)));
        
        mobilityControlPublisher_ = this->create_publisher<rover_mobility_interfaces::msg::MobilityControl>(mobility_control_topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default)));

        // Make shared state
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(1000.0 / publish_rate_)), std::bind(&ControlNode::publish_odometry, this));
    }


private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_ {};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_ {};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_ {};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher_ {};
    rclcpp::Publisher<rover_mobility_interfaces::msg::MobilityControl>::SharedPtr mobilityControlPublisher_ {};
    rclcpp::TimerBase::SharedPtr timer_{};
    sensor_msgs::msg::JointState jointStates_ {};

    rclcpp::Time lastCmdVelTime {};

    std::string cmdVelTopic_ { "cmd_vel" };
    std::string mobility_control_topic_ { "motor_control" };
    std::string odomTopic_ { "odom" };
    std::string jointStatesTopic_ { "joint_states" };

    double wheel_separation_ = 1.0;
    double wheel_radius_ = 0.1;
    std::string base_frame_id_ { "base_link" };
    std::string odom_frame_id_ { "odom" };

    std::vector<double> pose_covariance_diagonal_ { 0.002, 0.002, 0.0, 0.0, 0.0, 0.02 };
    std::vector<double> twist_covariance_diagonal_ { 0.002, 0.0, 0.0, 0.0, 0.0, 0.02 };

    double publish_rate_ = 30.0;

    double cmd_vel_timeout_ = 0.5;

    bool open_loop_ = true;

    bool linear_x_has_velocity_limits_ = false;
    double linear_x_max_velocity_ = 1.0;
    double linear_x_min_velocity_ = -1.0;

    bool angular_z_has_velocity_limits_ = false;
    double angular_z_max_velocity_ = 1.0;
    double angular_z_min_velocity_ = -1.0;
    
    RobotState robotState_ {};

    void cmd_vel_callback(const geometry_msgs::msg::Twist& msg) {
        rclcpp::Time currentTime = this->get_clock()->now();
        if (currentTime.seconds() - lastCmdVelTime.seconds() > cmd_vel_timeout_) {
            robotState_.linearVel_x = 0;
            robotState_.linearVel_y = 0;
            robotState_.angularVel_z = 0;
        } else {
            if (linear_x_has_velocity_limits_) {
                robotState_.linearVel_x = std::clamp(msg.linear.x, linear_x_min_velocity_, linear_x_max_velocity_);
            } else {
                robotState_.linearVel_x = msg.linear.x;
            }
            if (angular_z_has_velocity_limits_) {
                robotState_.angularVel_z = std::clamp(msg.angular.z, angular_z_min_velocity_, angular_z_max_velocity_);
            } else {
                robotState_.angularVel_z = msg.angular.z;
            }
        }
        lastCmdVelTime = currentTime;

        /*
        robotState_.vel_dt = (currentTime.seconds() - robotState_.last_vel_time);
        robotState_.last_vel_time = currentTime.seconds();

        double delta_heading = robotState_.angularVel_z * robotState_.vel_dt;
        double delta_x = (robotState_.linearVel_x * cos(robotState_.heading) - robotState_.linearVel_y * sin(robotState_.heading)) * robotState_.vel_dt;
        double delta_y = (robotState_.linearVel_x * sin(robotState_.heading) + robotState_.linearVel_y * cos(robotState_.heading)) * robotState_.vel_dt;

        robotState_.x_pos += delta_x;
        robotState_.y_pos += delta_y;
        robotState_.heading += delta_heading;
        */
    }

    void encoder_callback(const sensor_msgs::msg::JointState& msg) {
        double left_side_velocity_total = 0.0;
        double right_side_velocity_total = 0.0;

        for (size_t i = 0; i < msg.name.size(); i++) {
            if (std::find(robotState_.left_wheel_names.begin(), robotState_.left_wheel_names.end(), msg.name[i]) != robotState_.left_wheel_names.end()) {
                left_side_velocity_total += msg.velocity[i] * wheel_radius_;

            } else if (std::find(robotState_.right_wheel_names.begin(), robotState_.right_wheel_names.end(), msg.name[i]) != robotState_.right_wheel_names.end()) {
                right_side_velocity_total += msg.velocity[i] * wheel_radius_;
            } else {
                throw std::runtime_error("Unknown wheel joint name in encoder data!");
            }
        }
        robotState_.left_side_velocity = left_side_velocity_total / msg.velocity.size();
        robotState_.right_side_velocity = right_side_velocity_total / msg.velocity.size();

    }

    void publish_odometry() {
        rclcpp::Time currentTime = this->get_clock()->now();

        robotState_.vel_dt = (currentTime.seconds() - robotState_.last_vel_time);
        robotState_.last_vel_time = currentTime.seconds();

        double linear_velocity = 0;
        double angular_velocity = 0;
        if (open_loop_) {
            linear_velocity = robotState_.linearVel_x;
            angular_velocity = robotState_.angularVel_z;
        } else {
            linear_velocity = (robotState_.left_side_velocity + robotState_.right_side_velocity) / 2.0;
            angular_velocity = (robotState_.right_side_velocity - robotState_.left_side_velocity) / wheel_separation_;
        }

        double delta_heading = angular_velocity * robotState_.vel_dt;
        double delta_x = (linear_velocity * cos(robotState_.heading)) * robotState_.vel_dt;
        double delta_y = (linear_velocity * sin(robotState_.heading)) * robotState_.vel_dt;

        robotState_.x_pos += delta_x;
        robotState_.y_pos += delta_y;
        robotState_.heading += delta_heading;


        // https://github.com/linorobot/linorobot/blob/master/src/lino_base.cpp

        tf2::Quaternion odomQuat{};
        odomQuat.setRPY(0, 0, robotState_.heading);

        nav_msgs::msg::Odometry odomMsg{};
        odomMsg.header.stamp = currentTime;
        odomMsg.header.frame_id = odom_frame_id_;
        odomMsg.child_frame_id = base_frame_id_;

        odomMsg.pose.pose.position.x = robotState_.x_pos;
        odomMsg.pose.pose.position.y = robotState_.y_pos;
        odomMsg.pose.pose.position.z = 0.0;
        odomMsg.pose.pose.orientation.x = odomQuat.x();
        odomMsg.pose.pose.orientation.y = odomQuat.y();
        odomMsg.pose.pose.orientation.z = odomQuat.z();
        odomMsg.pose.pose.orientation.w = odomQuat.w();


        odomMsg.twist.twist.linear.x = linear_velocity;
        odomMsg.twist.twist.linear.y = 0.0;
        odomMsg.twist.twist.linear.z = 0.0;
        odomMsg.twist.twist.angular.x = 0.0;
        odomMsg.twist.twist.angular.y = 0.0;
        odomMsg.twist.twist.angular.z = angular_velocity;

        for (size_t i = 0; i < pose_covariance_diagonal_.size(); i++) {
            if (i * 7 < odomMsg.pose.covariance.size()) {
                odomMsg.pose.covariance[i * 7] = pose_covariance_diagonal_[i];
                odomMsg.twist.covariance[i * 7] = twist_covariance_diagonal_[i];
            }
        }
        odomPublisher_->publish(odomMsg);

        if (open_loop_) {
            jointStates_.header.frame_id = base_frame_id_;
            jointStates_.header.stamp = currentTime;
            for (size_t i = 0; i < robotState_.left_wheel_names.size(); i++) {
                jointStates_.velocity[i * 2] = robotState_.left_side_velocity / wheel_separation_;
                jointStates_.velocity[i * 2 + 1] = robotState_.right_side_velocity / wheel_separation_;

                jointStates_.position[i * 2] += jointStates_.velocity[i] * robotState_.vel_dt;
                jointStates_.position[i * 2 + 1] += jointStates_.velocity[i + 1] * robotState_.vel_dt;
            }
            jointStatePublisher_->publish(jointStates_);
        }

    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}