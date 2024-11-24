#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LaserFilter : public rclcpp::Node {
public:
    LaserFilter()
        : Node("laser_filter") {

        const std::string inputTopicParamName { "input_topic" };
        inputTopic_ = declare_parameter(inputTopicParamName, inputTopic_);

        const std::string outputTopicParamName { "output_topic" };
        outputTopic_ = declare_parameter(outputTopicParamName, outputTopic_);

        const std::string filteredRangesParamName { "filtered_ranges" };
        filteredRanges_ = declare_parameter(filteredRangesParamName, filteredRanges_);

        if (filteredRanges_.size() % 2 != 0 || filteredRanges_.size() == 0) {
            throw std::runtime_error("filtered_ranges parameter must have a size multiple of 2!");
        }

        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(inputTopic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
            std::bind(&LaserFilter::laser_subscription_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(outputTopic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default)));

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_ {};
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_ {};

    std::string inputTopic_ { "scan_raw" };
    std::string outputTopic_ { "scan" };
    std::vector<double> filteredRanges_ {1.0, 2.0};

    void laser_subscription_callback(const sensor_msgs::msg::LaserScan& msg) {

        sensor_msgs::msg::LaserScan filteredMsg = msg;
        filteredMsg.header.stamp = this->get_clock()->now();
        size_t rangeCount = filteredRanges_.size() / 2;
        for (size_t i = 0; i < rangeCount; i++) {
            const size_t filterStartIndex = (filteredRanges_[i] - msg.angle_min) / msg.angle_increment;
            const size_t filterEndIndex = (filteredRanges_[i + 1] - msg.angle_min) / msg.angle_increment;
            for (size_t j = filterStartIndex; j < filterEndIndex; j++) {
                if (j < msg.ranges.size()) {
                    // Set the range reading to a value higher than the max so that they will be discarded as it is assumed the laser did not return back to the lidar.
                    filteredMsg.ranges[j] = msg.range_max * 2;
                }
            }
        }
        publisher_->publish(filteredMsg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserFilter>());
    rclcpp::shutdown();
    return 0;
}