#include "g1_control/standing_controller.hpp"
#include <cmath>

// How many control cycles to run pure PD before enabling IMU balance correction.
// At 500Hz, 500 cycles = 1 second of settling time.
static constexpr int SETTLE_CYCLES = 500;

// Hard cap on velocity feedback to prevent physics explosions from
// corrupting torque commands during spawn transients.
static constexpr double MAX_JOINT_VELOCITY = 5.0;  // rad/s

StandingController::StandingController()
: Node("g1_standing_controller")
{
    joint_order_ = {
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint"
    };

    standing_pose_ = {
        {"left_hip_pitch_joint",   -0.2},
        {"left_hip_roll_joint",    0.0},
        {"left_hip_yaw_joint",     0.0},
        {"left_knee_joint",        0.4},
        {"left_ankle_pitch_joint", -0.2},
        {"left_ankle_roll_joint",  0.0},
        {"right_hip_pitch_joint",  -0.2},
        {"right_hip_roll_joint",   0.0},
        {"right_hip_yaw_joint",    0.0},
        {"right_knee_joint",       0.4},
        {"right_ankle_pitch_joint",-0.2},
        {"right_ankle_roll_joint", 0.0},
        {"waist_yaw_joint",        0.0}
    };

    kp_gains_ = {
        {"left_hip_pitch_joint",  200}, {"left_hip_roll_joint",   200},
        {"left_hip_yaw_joint",    120}, {"left_knee_joint",        300},
        {"left_ankle_pitch_joint",100}, {"left_ankle_roll_joint",  150},
        {"right_hip_pitch_joint", 200}, {"right_hip_roll_joint",   200},
        {"right_hip_yaw_joint",    120}, {"right_knee_joint",        300},
        {"right_ankle_pitch_joint",100},{"right_ankle_roll_joint",  150},
        {"waist_yaw_joint",        40}
    };

    kd_gains_ = {
        {"left_hip_pitch_joint",   5}, {"left_hip_roll_joint",    5},
        {"left_hip_yaw_joint",     3}, {"left_knee_joint",         6},
        {"left_ankle_pitch_joint", 3}, {"left_ankle_roll_joint",   3},
        {"right_hip_pitch_joint",  5}, {"right_hip_roll_joint",    5},
        {"right_hip_yaw_joint",    3}, {"right_knee_joint",         6},
        {"right_ankle_pitch_joint",3}, {"right_ankle_roll_joint",   3},
        {"waist_yaw_joint",         2}
    };

    effort_limits_ = {
        {"left_hip_pitch_joint",  139}, {"left_hip_roll_joint",   139},
        {"left_hip_yaw_joint",     88}, {"left_knee_joint",        139},
        {"left_ankle_pitch_joint", 35}, {"left_ankle_roll_joint",   35},
        {"right_hip_pitch_joint", 139}, {"right_hip_roll_joint",   139},
        {"right_hip_yaw_joint",    88}, {"right_knee_joint",        139},
        {"right_ankle_pitch_joint",35}, {"right_ankle_roll_joint",  35},
        {"waist_yaw_joint",        88}
    };

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&StandingController::jointCallback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10,
        std::bind(&StandingController::imuCallback, this, std::placeholders::_1));

    effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/effort_controller/commands", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&StandingController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "G1 Standing Controller Started (balance mode)");
}

void StandingController::quaternionToRPY(
    double qx, double qy, double qz, double qw,
    double &roll, double &pitch, double &yaw)
{
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void StandingController::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg)
{
    double yaw;
    quaternionToRPY(
        msg->orientation.x, msg->orientation.y,
        msg->orientation.z, msg->orientation.w,
        imu_roll_, imu_pitch_, yaw);

    imu_roll_rate_  = msg->angular_velocity.x;
    imu_pitch_rate_ = msg->angular_velocity.y;
    imu_received_   = true;
}

void StandingController::jointCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (size_t i = 0; i < msg->name.size(); i++) {
        const auto &name = msg->name[i];
        if (standing_pose_.count(name)) {
            double pos = msg->position[i];
            double vel = (msg->velocity.size() > i) ? msg->velocity[i] : 0.0;

            // Reject NaN/Inf from physics transients
            if (!std::isfinite(pos) || !std::isfinite(vel)) continue;

            joint_positions_[name] = pos;
            joint_velocities_[name] = vel;
        }
    }
}

void StandingController::controlLoop()
{
    if (joint_positions_.size() < joint_order_.size()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Waiting for joint states: %ld/%ld",
            joint_positions_.size(), joint_order_.size());
        return;
    }

    if (!imu_received_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Waiting for IMU...");
        return;
    }

    if (!joints_received_logged_) {
        RCLCPP_INFO(get_logger(), "All sensors ready — balance control active");
        joints_received_logged_ = true;
    }

    settle_counter_++;
    bool balance_active = (settle_counter_ > SETTLE_CYCLES);

    if (settle_counter_ == SETTLE_CYCLES + 1)
        RCLCPP_INFO(get_logger(), "Settle complete — IMU balance correction enabled");

    // IMU balance corrections (only after settling)
    double pitch_correction = 0.0;
    double roll_correction  = 0.0;
    if (balance_active) {
        pitch_correction = kp_balance_pitch_ * imu_pitch_
                         + kd_balance_pitch_ * imu_pitch_rate_;
        roll_correction  = kp_balance_roll_  * imu_roll_
                         + kd_balance_roll_  * imu_roll_rate_;

        // Clamp balance corrections to avoid destabilizing
        pitch_correction = std::clamp(pitch_correction, -15.0, 15.0);
        roll_correction  = std::clamp(roll_correction,  -10.0, 10.0);
    }

    std_msgs::msg::Float64MultiArray out_msg;
    std::vector<double> efforts;

    for (const auto &joint : joint_order_) {
        double q   = joint_positions_[joint];
        double dq  = joint_velocities_[joint];

        // Clamp velocity to prevent explosion transients from corrupting torque
        dq = std::clamp(dq, -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY);

        double q_des = standing_pose_[joint];
        double kp    = kp_gains_[joint];
        double kd    = kd_gains_[joint];

        double tau = kp * (q_des - q) - kd * dq;

        if (balance_active) {
            if (joint == "left_hip_pitch_joint" || joint == "right_hip_pitch_joint")
                tau -= pitch_correction;
            if (joint == "left_ankle_pitch_joint" || joint == "right_ankle_pitch_joint")
                tau += pitch_correction;
            if (joint == "left_hip_roll_joint" || joint == "right_hip_roll_joint")
                tau -= roll_correction;
            if (joint == "left_ankle_roll_joint" || joint == "right_ankle_roll_joint")
                tau += roll_correction;
        }

        double limit = effort_limits_[joint];
        tau = std::clamp(tau, -limit, limit);
        efforts.push_back(tau);
    }

    out_msg.data = efforts;
    effort_pub_->publish(out_msg);
}