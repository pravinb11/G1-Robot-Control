#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <Eigen/Dense>

#include "g1_zmp_walking/walking_fsm.hpp"
#include "g1_zmp_walking/footstep_planner.hpp"   // for yaw_to_quaternion

namespace g1_zmp_walking {

// ─────────────────────────────────────────────────────────────────────────────

class WalkingFSMNode : public rclcpp::Node
{
public:
  explicit WalkingFSMNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("walking_fsm", opts)
  {
    // ── parameters ────────────────────────────────────────────────────────────
    declare_parameter("g1.t_single",   0.70);
    declare_parameter("g1.t_double",   0.10);
    declare_parameter("control.dt",    0.005);
    declare_parameter("world_frame",   std::string("odom"));

    FSMParams p;
    p.t_single   = get_parameter("g1.t_single").as_double();
    p.t_double   = get_parameter("g1.t_double").as_double();
    p.dt         = get_parameter("control.dt").as_double();
    fsm_         = std::make_unique<WalkingFSM>(p);
    dt_          = p.dt;
    world_frame_ = get_parameter("world_frame").as_string();

    // ── FSM callbacks ─────────────────────────────────────────────────────────
    fsm_->on_lift_off = [this](const FSMFootstep& step) {
      RCLCPP_INFO(get_logger(), "LIFT-OFF  step %d  %s  target=[%.3f, %.3f, %.3f]",
        fsm_->step_idx(),
        step.foot == SwingFoot::LEFT ? "LEFT" : "RIGHT",
        step.position.x(), step.position.y(), step.position.z());
    };

    fsm_->on_touch_down = [this](const FSMFootstep& step) {
      RCLCPP_INFO(get_logger(), "TOUCH-DOWN step %d  %s",
        fsm_->step_idx(),
        step.foot == SwingFoot::LEFT ? "LEFT" : "RIGHT");
    };

    fsm_->on_walk_complete = [this]() {
      RCLCPP_INFO(get_logger(), "Walk complete — FSM entered STOP_DOUBLE.");
      // Stop the control timer
      if (control_timer_) control_timer_->cancel();
    };

    // ── publishers ────────────────────────────────────────────────────────────
    const auto qos = rclcpp::QoS(10);

    // Current FSM state as string (for logging / dashboards)
    pub_state_str_   = create_publisher<std_msgs::msg::String>(
                         "~/state", qos);
    // Step index
    pub_step_idx_    = create_publisher<std_msgs::msg::Int32>(
                         "~/step_idx", qos);
    // Swing phase [0..1]
    pub_swing_phase_ = create_publisher<std_msgs::msg::Float64>(
                         "~/swing_phase", qos);
    // Current swing foot target as PoseStamped (for IK / swing trajectory)
    pub_swing_target_= create_publisher<geometry_msgs::msg::PoseStamped>(
                         "~/swing_target", qos);
    // RViz2 state text overlay
    pub_marker_      = create_publisher<visualization_msgs::msg::Marker>(
                         "~/fsm_marker", qos);

    // ── subscribers ───────────────────────────────────────────────────────────
    // Footsteps from planner → start the FSM
    sub_footsteps_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/footstep_planner/footsteps", qos,
      std::bind(&WalkingFSMNode::footsteps_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "WalkingFSMNode ready  t_single=%.2f  t_double=%.2f  dt=%.4f",
      p.t_single, p.t_double, p.dt);
    RCLCPP_INFO(get_logger(),
      "Waiting for footsteps on /footstep_planner/footsteps ...");
  }

private:
  std::unique_ptr<WalkingFSM>                                      fsm_;
  rclcpp::TimerBase::SharedPtr                                     control_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr   sub_footsteps_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              pub_state_str_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr               pub_step_idx_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr             pub_swing_phase_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    pub_swing_target_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    pub_marker_;

  std::string  world_frame_;
  double       dt_{0.005};

  // ── footstep callback ─────────────────────────────────────────────────────

  void footsteps_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    // Cancel any running walk
    if (control_timer_) control_timer_->cancel();

    // Convert PoseArray → FSMFootstep list
    std::vector<FSMFootstep> steps;
    steps.reserve(msg->poses.size());
    for (const auto& pose : msg->poses) {
      FSMFootstep fs;
      fs.foot     = (pose.position.y >= 0.0) ? SwingFoot::LEFT : SwingFoot::RIGHT;
      fs.position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
      // Extract yaw from quaternion
      const auto& q = pose.orientation;
      fs.yaw = std::atan2(
        2.0*(q.w*q.z + q.x*q.y),
        1.0 - 2.0*(q.y*q.y + q.z*q.z));
      steps.push_back(fs);
    }

    RCLCPP_INFO(get_logger(),
      "Received %zu footsteps — starting FSM.", steps.size());

    fsm_->start(steps);

    // Start control loop timer
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&WalkingFSMNode::control_loop, this));
  }

  // ── control loop: runs at 1/dt Hz ─────────────────────────────────────────

  void control_loop()
  {
    const FSMOutput out = fsm_->update();
    const auto stamp    = now();

    // ── publish state string ──────────────────────────────────────────────────
    {
      std_msgs::msg::String s;
      s.data = walk_state_name(out.state);
      pub_state_str_->publish(s);
    }

    // ── publish step index ────────────────────────────────────────────────────
    {
      std_msgs::msg::Int32 m;
      m.data = out.step_idx;
      pub_step_idx_->publish(m);
    }

    // ── publish swing phase ───────────────────────────────────────────────────
    if (out.is_single_support()) {
      std_msgs::msg::Float64 m;
      m.data = out.swing_phase;
      pub_swing_phase_->publish(m);

      // Publish current swing foot target pose
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = world_frame_;
      ps.header.stamp    = stamp;
      ps.pose.position.x = out.current_step.position.x();
      ps.pose.position.y = out.current_step.position.y();
      ps.pose.position.z = out.current_step.position.z();
      const auto q = yaw_to_quaternion(out.current_step.yaw);
      ps.pose.orientation.x = q.x;
      ps.pose.orientation.y = q.y;
      ps.pose.orientation.z = q.z;
      ps.pose.orientation.w = q.w;
      pub_swing_target_->publish(ps);
    }

    // ── RViz2 text marker showing current state ───────────────────────────────
    publish_state_marker(out, stamp);

    // Stop timer if walk is done
    if (out.is_done() && control_timer_)
      control_timer_->cancel();
  }

  // ── RViz2 marker ──────────────────────────────────────────────────────────

  void publish_state_marker(const FSMOutput& out,
                            const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp    = stamp;
    m.ns              = "fsm_state";
    m.id              = 0;
    m.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action          = visualization_msgs::msg::Marker::ADD;

    // Position text above origin
    m.pose.position.x    = 0.0;
    m.pose.position.y    = 0.0;
    m.pose.position.z    = 1.2;
    m.pose.orientation.w = 1.0;
    m.scale.z            = 0.12;   // text height

    // Colour by state
    switch (out.state) {
      case WalkState::INIT_DOUBLE:
        m.color.r=1.0f; m.color.g=0.8f; m.color.b=0.0f; break;  // yellow
      case WalkState::SINGLE_SUPPORT:
        m.color.r=0.1f; m.color.g=0.9f; m.color.b=0.3f; break;  // green
      case WalkState::DOUBLE_SUPPORT:
        m.color.r=0.2f; m.color.g=0.5f; m.color.b=1.0f; break;  // blue
      case WalkState::STOP_DOUBLE:
        m.color.r=1.0f; m.color.g=0.3f; m.color.b=0.1f; break;  // red
      default:
        m.color.r=0.7f; m.color.g=0.7f; m.color.b=0.7f; break;  // grey
    }
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);  // auto-expire

    // Build text
    std::string txt = walk_state_name(out.state);
    txt += "\nStep: " + std::to_string(out.step_idx)
         + "/" + std::to_string(fsm_->n_steps());
    if (out.is_single_support()) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "\nSwing: %s  %.0f%%",
        out.swing_foot == SwingFoot::LEFT ? "LEFT" : "RIGHT",
        out.swing_phase * 100.0);
      txt += buf;
    }
    m.text = txt;
    pub_marker_->publish(m);
  }
};

}  // namespace g1_zmp_walking

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_zmp_walking::WalkingFSMNode>());
  rclcpp::shutdown();
  return 0;
}