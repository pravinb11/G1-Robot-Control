#include "g1_control/standing_controller.hpp"


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
        {"left_hip_pitch_joint",-0.2},
        {"left_hip_roll_joint",0.0},
        {"left_hip_yaw_joint",0.0},
        {"left_knee_joint",0.4},
        {"left_ankle_pitch_joint",-0.2},
        {"left_ankle_roll_joint",0.0},

        {"right_hip_pitch_joint",-0.2},
        {"right_hip_roll_joint",0.0},
        {"right_hip_yaw_joint",0.0},
        {"right_knee_joint",0.4},
        {"right_ankle_pitch_joint",-0.2},
        {"right_ankle_roll_joint",0.0},
        {"waist_yaw_joint",0.0}
    };

    kp_gains_ = {
        {"left_hip_pitch_joint",120},
        {"left_hip_roll_joint",120},
        {"left_hip_yaw_joint",60},
        {"left_knee_joint",160},
        {"left_ankle_pitch_joint",80},
        {"left_ankle_roll_joint",80},

        {"right_hip_pitch_joint",120},
        {"right_hip_roll_joint",120},
        {"right_hip_yaw_joint",60},
        {"right_knee_joint",160},
        {"right_ankle_pitch_joint",80},
        {"right_ankle_roll_joint",80},

        {"waist_yaw_joint",20}
    };

    kd_gains_ = {
        {"left_hip_pitch_joint",10},
        {"left_hip_roll_joint",10},
        {"left_hip_yaw_joint",5},
        {"left_knee_joint",15},
        {"left_ankle_pitch_joint",5},
        {"left_ankle_roll_joint",5},

        {"right_hip_pitch_joint",10},
        {"right_hip_roll_joint",10},
        {"right_hip_yaw_joint",5},
        {"right_knee_joint",15},
        {"right_ankle_pitch_joint",5},
        {"right_ankle_roll_joint",5},
        {"waist_yaw_joint",2}
    };

    effort_limits_ = {
        {"left_hip_pitch_joint",139},
        {"left_hip_roll_joint",139},
        {"left_hip_yaw_joint",88},
        {"left_knee_joint",139},
        {"left_ankle_pitch_joint",35},
        {"left_ankle_roll_joint",35},
        {"right_hip_pitch_joint",139},
        {"right_hip_roll_joint",139},
        {"right_hip_yaw_joint",88},
        {"right_knee_joint",139},
        {"right_ankle_pitch_joint",35},
        {"right_ankle_roll_joint",35},
        {"waist_yaw_joint",88}
    };

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&StandingController::jointCallback, this, std::placeholders::_1));

    effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/effort_controller/commands", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&StandingController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "G1 Standing Controller Started");
}


void StandingController::jointCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for(size_t i=0;i<msg->name.size();i++)
    {
        const auto &name = msg->name[i];

        if(standing_pose_.count(name))
        {
            joint_positions_[name] = msg->position[i];

            if(msg->velocity.size()>i)
                joint_velocities_[name] = msg->velocity[i];
        }
    }
}

void StandingController::controlLoop()
{
    size_t received = joint_positions_.size();
    size_t total = joint_order_.size();

    if(received < total)
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Waiting for joint states: %ld/%ld",
            received,total);
        return;
    }

    if(!joints_received_logged_)
    {
        RCLCPP_INFO(get_logger(),"Received all joints — starting control loop");
        joints_received_logged_ = true;
    }

    std_msgs::msg::Float64MultiArray msg;
    std::vector<double> efforts;

    for(const auto &joint : joint_order_)
    {
        double q = joint_positions_[joint];
        double dq = joint_velocities_[joint];

        double q_des = standing_pose_[joint];
        double kp = kp_gains_[joint];
        double kd = kd_gains_[joint];

        double tau = kp*(q_des-q) - kd*dq;

        double limit = effort_limits_[joint];
        tau = std::clamp(tau,-limit,limit);

        efforts.push_back(tau);
    }

    msg.data = efforts;
    effort_pub_->publish(msg);
}



