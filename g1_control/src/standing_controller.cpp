#include "g1_control/standing_controller.hpp"
#include <cmath>

// Control timeline (all at 200 Hz / 5 ms timer; physics at 500 Hz / 2 ms step):
//   0 → RAMP_CYCLES        : slowly interpolate joints from spawn pose → standing pose
//   RAMP_CYCLES → RAMP_CYCLES+SETTLE_CYCLES : pure joint PD, no IMU corrections
//   RAMP_CYCLES+SETTLE_CYCLES onwards        : joint PD + IMU balance corrections
static constexpr int RAMP_CYCLES   = 600;    // 3 s smooth ramp   @ 200 Hz
static constexpr int SETTLE_CYCLES = 300;    // 1.5 s settle phase @ 200 Hz

static constexpr double MAX_JOINT_VELOCITY = 5.0;  // rad/s clamp

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
        {"left_hip_pitch_joint",   -0.20},
        {"left_hip_roll_joint",     0.00},
        {"left_hip_yaw_joint",      0.00},
        {"left_knee_joint",         0.40},
        {"left_ankle_pitch_joint", -0.20},
        {"left_ankle_roll_joint",   0.00},
        {"right_hip_pitch_joint",  -0.20},
        {"right_hip_roll_joint",    0.00},
        {"right_hip_yaw_joint",     0.00},
        {"right_knee_joint",        0.40},
        {"right_ankle_pitch_joint",-0.20},
        {"right_ankle_roll_joint",  0.00},
        {"waist_yaw_joint",         0.00}
    };

    // Kp tuned for Gazebo's 1 ms physics step — very high Kp causes contact-model
    // explosions. These are 40-50 % of the effort limits which gives enough
    // torque to hold the standing pose while remaining numerically stable.
    kp_gains_ = {
        {"left_hip_pitch_joint",   80}, {"left_hip_roll_joint",    80},
        {"left_hip_yaw_joint",     50}, {"left_knee_joint",         100},
        {"left_ankle_pitch_joint", 40}, {"left_ankle_roll_joint",    50},
        {"right_hip_pitch_joint",  80}, {"right_hip_roll_joint",    80},
        {"right_hip_yaw_joint",    50}, {"right_knee_joint",         100},
        {"right_ankle_pitch_joint",40}, {"right_ankle_roll_joint",   50},
        {"waist_yaw_joint",        20}
    };

    // Kd = ~10 % of Kp — enough to damp oscillations without exciting the
    // contact-model resonance that appears at higher Kd in Gazebo Classic.
    kd_gains_ = {
        {"left_hip_pitch_joint",   8}, {"left_hip_roll_joint",    8},
        {"left_hip_yaw_joint",     5}, {"left_knee_joint",        10},
        {"left_ankle_pitch_joint", 4}, {"left_ankle_roll_joint",   5},
        {"right_hip_pitch_joint",  8}, {"right_hip_roll_joint",    8},
        {"right_hip_yaw_joint",    5}, {"right_knee_joint",        10},
        {"right_ankle_pitch_joint",4}, {"right_ankle_roll_joint",   5},
        {"waist_yaw_joint",         3}
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
        std::chrono::milliseconds(5),
        std::bind(&StandingController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "G1 Standing Controller started");
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
    // Guard against NaN quaternion from physics divergence
    if (!std::isfinite(msg->orientation.x) || !std::isfinite(msg->orientation.y) ||
        !std::isfinite(msg->orientation.z) || !std::isfinite(msg->orientation.w)) {
        return;
    }
    quaternionToRPY(
        msg->orientation.x, msg->orientation.y,
        msg->orientation.z, msg->orientation.w,
        imu_roll_, imu_pitch_, yaw);

    // Angular velocity from Gazebo's IMU plugin can become NaN when the physics
    // solver diverges.  Guard here so NaN never reaches the balance correction.
    imu_roll_rate_  = std::isfinite(msg->angular_velocity.x) ? msg->angular_velocity.x : 0.0;
    imu_pitch_rate_ = std::isfinite(msg->angular_velocity.y) ? msg->angular_velocity.y : 0.0;
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

    // Snapshot initial joint positions once (used as ramp start point)
    if (!ramp_start_captured_) {
        for (const auto &joint : joint_order_)
            ramp_start_[joint] = joint_positions_[joint];
        ramp_start_captured_ = true;
        RCLCPP_INFO(get_logger(), "Sensors ready — starting 3 s position ramp to standing pose");
    }

    control_cycle_++;

    // ── Phase 1: ramp ────────────────────────────────────────────────────────
    bool ramp_done    = (control_cycle_ >= RAMP_CYCLES);
    bool balance_active = (control_cycle_ >= RAMP_CYCLES + SETTLE_CYCLES);

    double alpha = std::min(1.0, static_cast<double>(control_cycle_) / RAMP_CYCLES);
    // Smooth step (ease-in / ease-out) to avoid sharp torque spikes
    double alpha_smooth = alpha * alpha * (3.0 - 2.0 * alpha);

    if (ramp_done && !ramp_logged_) {
        RCLCPP_INFO(get_logger(), "Ramp complete — holding standing pose");
        ramp_logged_ = true;
    }
    if (balance_active && !balance_logged_) {
        RCLCPP_INFO(get_logger(), "Settle complete — IMU balance correction enabled");
        balance_logged_ = true;
    }

    // ── IMU balance corrections ───────────────────────────────────────────────
    // Deadband: only correct when tilt is meaningful (> 0.05 rad ≈ 2.9°).
    // This prevents the balance loop from fighting the joint PD at steady state,
    // which would otherwise cause slow drift toward joint limits.
    static constexpr double BALANCE_DEADBAND = 0.05;  // rad

    double pitch_correction = 0.0;
    double roll_correction  = 0.0;
    if (balance_active) {
        double eff_pitch = std::abs(imu_pitch_) > BALANCE_DEADBAND ? imu_pitch_ : 0.0;
        double eff_roll  = std::abs(imu_roll_)  > BALANCE_DEADBAND ? imu_roll_  : 0.0;

        pitch_correction = kp_balance_pitch_ * eff_pitch
                         + kd_balance_pitch_ * imu_pitch_rate_;
        roll_correction  = kp_balance_roll_  * eff_roll
                         + kd_balance_roll_  * imu_roll_rate_;

        pitch_correction = std::clamp(pitch_correction, -20.0, 20.0);
        roll_correction  = std::clamp(roll_correction,  -12.0, 12.0);
    }

    // ── Joint PD loop ─────────────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray out_msg;
    std::vector<double> efforts;

    for (const auto &joint : joint_order_) {
        double q   = joint_positions_[joint];
        double dq  = joint_velocities_[joint];

        // Skip cycle if physics has diverged; publish zeros to avoid torque bursts.
        if (!std::isfinite(q) || !std::isfinite(dq)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "NaN in joint state for %s — holding last torque", joint.c_str());
            out_msg.data = std::vector<double>(joint_order_.size(), 0.0);
            effort_pub_->publish(out_msg);
            return;
        }

        dq = std::clamp(dq, -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY);

        // Interpolate setpoint: spawn position → standing pose
        double q_start = ramp_start_.count(joint) ? ramp_start_[joint] : 0.0;
        double q_des   = q_start + alpha_smooth * (standing_pose_[joint] - q_start);

        double kp = kp_gains_[joint];
        double kd = kd_gains_[joint];
        double tau = kp * (q_des - q) - kd * dq;

        if (balance_active) {
            // Forward pitch → extend hips AND plantarflex ankles (both need -=)
            if (joint == "left_hip_pitch_joint" || joint == "right_hip_pitch_joint")
                tau -= pitch_correction;
            if (joint == "left_ankle_pitch_joint" || joint == "right_ankle_pitch_joint")
                tau -= pitch_correction;
            // Roll corrections
            if (joint == "left_hip_roll_joint" || joint == "right_hip_roll_joint")
                tau -= roll_correction;
            if (joint == "left_ankle_roll_joint" || joint == "right_ankle_roll_joint")
                tau -= roll_correction;
        }

        tau = std::clamp(tau, -effort_limits_[joint], effort_limits_[joint]);
        efforts.push_back(tau);
    }

    out_msg.data = efforts;
    effort_pub_->publish(out_msg);
}
