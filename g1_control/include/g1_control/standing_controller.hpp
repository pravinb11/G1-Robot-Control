#ifndef G1_CONTROL_STANDING_CONTROLLER_HPP_
#define G1_CONTROL_STANDING_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
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
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void controlLoop();
    void quaternionToRPY(double qx, double qy, double qz, double qw,
                         double &roll, double &pitch, double &yaw);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, double> joint_positions_;
    std::unordered_map<std::string, double> joint_velocities_;

    std::vector<std::string> joint_order_;
    std::unordered_map<std::string, double> standing_pose_;
    std::unordered_map<std::string, double> kp_gains_;
    std::unordered_map<std::string, double> kd_gains_;
    std::unordered_map<std::string, double> effort_limits_;

    // IMU state
    double imu_roll_       = 0.0;
    double imu_pitch_      = 0.0;
    double imu_roll_rate_  = 0.0;
    double imu_pitch_rate_ = 0.0;
    bool   imu_received_   = false;

    // Balance PD gains — pitch drives hip_pitch + ankle_pitch
    double kp_balance_pitch_ = 80.0; //70.0;
    double kd_balance_pitch_ = 10.0; //8.0;
    // Balance PD gains — roll drives hip_roll + ankle_roll
    double kp_balance_roll_  = 40.0;
    double kd_balance_roll_  = 6.0;

    bool joints_received_logged_ = false;
    int settle_counter_ = 0;
};

#endif // G1_CONTROL_STANDING_CONTROLLER_HPP_