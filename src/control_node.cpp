// Yigithan Yigit 2024

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


#define TOTAL_TIME(sec, nsec) (sec + nsec)

struct RobotState
{
    std::vector<std::string> left_wheel_names;
    std::vector<std::string> right_wheel_names;
    double left_wheel_velocity = 0.0;
    double right_wheel_velocity = 0.0;
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

        this->declare_parameter("cmd_vel_topic", cmdVelTopic_);
        this->declare_parameter("output_topic", encoderTopic_);
        this->declare_parameter("odom_topic", odomTopic_);
        this->declare_parameter("wheel_separation", 0.788);
        this->declare_parameter("wheel_radius", 0.11736);
        this->declare_parameter("left_wheel_names", std::vector<std::string>{"front_left_wheel_joint", "back_left_wheel_joint"});
        this->declare_parameter("right_wheel_names", std::vector<std::string>{"front_right_wheel_joint", "back_right_wheel_joint"});

        this->get_parameter("wheel_separation", wheel_separation);
        this->get_parameter("wheel_radius", wheel_radius);
        this->get_parameter("cmd_vel_topic", cmdVelTopic_);
        this->get_parameter("output_topic", encoderTopic_);
        this->get_parameter("odom_topic", odomTopic_);
        this->get_parameter("left_wheel_names", robotState.left_wheel_names);
        this->get_parameter("right_wheel_names", robotState.right_wheel_names);

        cmdVelSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(cmdVelTopic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
            std::bind(&ControlNode::cmd_vel_callback, this, std::placeholders::_1));

        encoderSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(encoderTopic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
            std::bind(&ControlNode::encoder_callback, this, std::placeholders::_1));

        odomPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odomTopic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default)));

        // Make shared state
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ControlNode::publish_odometry, this));
    }

    RobotState robotState;

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_ {};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoderSubscriber_ {};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher_ {};
    rclcpp::TimerBase::SharedPtr timer_;

    std::string cmdVelTopic_ { "cmd_vel" };
    std::string encoderTopic_ { "encoder_data" };
    std::string odomTopic_ { "odom" };

    double wheel_separation;
    double wheel_radius;


    void cmd_vel_callback(const geometry_msgs::msg::Twist& msg) {
        rclcpp::Time currentTime = this->get_clock()->now();
        {
            std::lock_guard<std::mutex> lock(robotState.stateMutex);
            robotState.linearVel_x = msg.linear.x;
            robotState.linearVel_y = msg.linear.y;
            robotState.angularVel_z = msg.angular.z;

            /*
            robotState.vel_dt = (currentTime.seconds() - robotState.last_vel_time);
            robotState.last_vel_time = currentTime.seconds();

            double delta_heading = robotState.angularVel_z * robotState.vel_dt;
            double delta_x = (robotState.linearVel_x * cos(robotState.heading) - robotState.linearVel_y * sin(robotState.heading)) * robotState.vel_dt;
            double delta_y = (robotState.linearVel_x * sin(robotState.heading) + robotState.linearVel_y * cos(robotState.heading)) * robotState.vel_dt;

            robotState.x_pos += delta_x;
            robotState.y_pos += delta_y;
            robotState.heading += delta_heading;
            */
        }
    }

    void encoder_callback(const sensor_msgs::msg::JointState& msg) {
        double left_wheel_velocity = 0.0;
        double right_wheel_velocity = 0.0;

        for (size_t i = 0; i < msg.name.size(); ++i) {
            if (msg.name[i] == "front_left_wheel_joint" || msg.name[i] == "back_left_wheel_joint") {
                left_wheel_velocity += msg.velocity[i];
            } else if (msg.name[i] == "front_right_wheel_joint" || msg.name[i] == "back_right_wheel_joint") {
                right_wheel_velocity += msg.velocity[i];
            } else {
                throw std::runtime_error("Unknown wheel joint name in encoder data!");
            }
        }

        left_wheel_velocity /= 2.0;
        right_wheel_velocity /= 2.0;

        std::lock_guard<std::mutex> lock(robotState.stateMutex);
        robotState.left_wheel_velocity = left_wheel_velocity;
        robotState.right_wheel_velocity = right_wheel_velocity;
    }

  void publish_odometry() {
        rclcpp::Time currentTime = this->get_clock()->now();
        {
            std::lock_guard<std::mutex> lock(robotState.stateMutex);
            robotState.vel_dt = (currentTime.seconds() - robotState.last_vel_time);
            robotState.last_vel_time = currentTime.seconds();

            double linear_velocity = (robotState.left_wheel_velocity + robotState.right_wheel_velocity) * wheel_radius / 2.0;
            double angular_velocity = (robotState.right_wheel_velocity - robotState.left_wheel_velocity) * wheel_radius / wheel_separation;

            double delta_heading = angular_velocity * robotState.vel_dt;
            double delta_x = (linear_velocity * cos(robotState.heading)) * robotState.vel_dt;
            double delta_y = (linear_velocity * sin(robotState.heading)) * robotState.vel_dt;

            robotState.x_pos += delta_x;
            robotState.y_pos += delta_y;
            robotState.heading += delta_heading;

            tf2::Quaternion odomQuat;
            odomQuat.setRPY(0, 0, robotState.heading);

            // https://github.com/linorobot/linorobot/blob/master/src/lino_base.cpp
            /*
            geometry_msgs::msg::TransformStamped odomTrans;
            odomTrans.header.stamp = currentTime;
            odomTrans.header.frame_id = "odom";
            odomTrans.child_frame_id = "base_footprint";


            odomTrans.transform.translation.x = robotState.x_pos;
            odomTrans.transform.translation.y = robotState.y_pos;
            odomTrans.transform.translation.z = 0.0;
            odomTrans.transform.rotation.x = odomQuat.x();
            odomTrans.transform.rotation.y = odomQuat.y();
            odomTrans.transform.rotation.z = odomQuat.z();
            odomTrans.transform.rotation.w = odomQuat.w();

            odom_broadcaster_.sendTransform(odomTrans);
            */

            nav_msgs::msg::Odometry odomMsg;
            odomMsg.header.stamp = currentTime;
            odomMsg.header.frame_id = "odom";
            odomMsg.child_frame_id = "base_footprint";

            odomMsg.pose.pose.position.x = robotState.x_pos;
            odomMsg.pose.pose.position.y = robotState.y_pos;
            odomMsg.pose.pose.position.z = 0.0;
            odomMsg.pose.pose.orientation.x = odomQuat.x();
            odomMsg.pose.pose.orientation.y = odomQuat.y();
            odomMsg.pose.pose.orientation.z = odomQuat.z();
            odomMsg.pose.pose.orientation.w = odomQuat.w();
            odomMsg.pose.covariance[0] = 0.001;
            odomMsg.pose.covariance[7] = 0.001;
            odomMsg.pose.covariance[35] = 0.001;

            odomMsg.twist.twist.linear.x = linear_velocity;
            odomMsg.twist.twist.linear.y = 0.0;
            odomMsg.twist.twist.linear.z = 0.0;
            odomMsg.twist.twist.angular.x = 0.0;
            odomMsg.twist.twist.angular.y = 0.0;
            odomMsg.twist.twist.angular.z = angular_velocity;
            odomMsg.twist.covariance[0] = 0.0001;
            odomMsg.twist.covariance[7] = 0.0001;
            odomMsg.twist.covariance[35] = 0.0001;

            odomPublisher_->publish(odomMsg);
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}