#ifndef G1_CONTROL_STANDING_CONTROLLER_HPP_
#define G1_CONTROL_STANDING_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <unordered_map>
#include <vector>
#include <string>

class StandingController : public rclcpp::Node
{
public:
    StandingController();

private:

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void controlLoop();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, double> joint_positions_;
    std::unordered_map<std::string, double> joint_velocities_;

    std::vector<std::string> joint_order_;

    std::unordered_map<std::string, double> standing_pose_;

    std::unordered_map<std::string, double> kp_gains_;

    std::unordered_map<std::string, double> kd_gains_;
    std::unordered_map<std::string,double> effort_limits_;

    bool joints_received_logged_;
};

#endif // G1_CONTROL_STANDING_CONTROLLER_HPP_