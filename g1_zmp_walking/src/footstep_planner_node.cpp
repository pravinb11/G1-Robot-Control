#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "builtin_interfaces/msg/duration.hpp"

// TF2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Generated action header (adjust package name if different)
#include "g1_zmp_walking/action/plan_footsteps.hpp"

// Pure-logic planner (no ROS)
#include "g1_zmp_walking/footstep_planner.hpp"

namespace g1_zmp_walking {

using PlanFootsteps    = g1_zmp_walking::action::PlanFootsteps;
using GoalHandlePtr    = rclcpp_action::ServerGoalHandle<PlanFootsteps>;
using PoseArray        = geometry_msgs::msg::PoseArray;
using Pose             = geometry_msgs::msg::Pose;
using MarkerArray      = visualization_msgs::msg::MarkerArray;
using Marker           = visualization_msgs::msg::Marker;
using ColorRGBA        = std_msgs::msg::ColorRGBA;

// ─────────────────────────────────────────────────────────────────────────────
// Colour helpers
// ─────────────────────────────────────────────────────────────────────────────
static ColorRGBA make_color(float r, float g, float b, float a)
{
  ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = a;
  return c;
}

static const ColorRGBA COLOR_LEFT    = make_color(0.10f, 0.85f, 0.30f, 0.85f);
static const ColorRGBA COLOR_RIGHT   = make_color(0.20f, 0.50f, 0.95f, 0.85f);
static const ColorRGBA COLOR_TEXT    = make_color(1.00f, 1.00f, 1.00f, 1.00f);
static const ColorRGBA COLOR_ARROW   = make_color(1.00f, 0.85f, 0.00f, 0.90f);

// ─────────────────────────────────────────────────────────────────────────────
// Node
// ─────────────────────────────────────────────────────────────────────────────
class FootstepPlannerNode : public rclcpp::Node
{
public:
  explicit FootstepPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("footstep_planner", options)
  {
    // ── declare parameters ─────────────────────────────────────────────────
    declare_params();

    // ── build planner from params ──────────────────────────────────────────
    FootstepPlannerParams cfg;
    cfg.hip_width       = get_parameter("g1.hip_width").as_double();
    cfg.max_step_length = get_parameter("g1.max_step_length").as_double();
    cfg.max_step_width  = get_parameter("g1.max_step_width").as_double();
    cfg.max_step_yaw    = get_parameter("g1.max_step_yaw").as_double();
    cfg.t_single        = get_parameter("g1.t_single").as_double();
    cfg.t_double        = get_parameter("g1.t_double").as_double();
    planner_            = std::make_unique<FootstepPlanner>(cfg);

    // ── frame names ────────────────────────────────────────────────────────
    world_frame_      = get_parameter("world_frame").as_string();
    left_foot_frame_  = get_parameter("left_foot_frame").as_string();
    right_foot_frame_ = get_parameter("right_foot_frame").as_string();
    marker_foot_len_  = get_parameter("foot_marker_length").as_double();
    marker_foot_wid_  = get_parameter("foot_marker_width").as_double();

    // ── TF2 ───────────────────────────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // ── publishers ─────────────────────────────────────────────────────────
    const auto qos = rclcpp::QoS(10);
    pub_poses_   = create_publisher<PoseArray>  ("~/footsteps",        qos);
    pub_markers_ = create_publisher<MarkerArray>("~/footstep_markers", qos);

    // ── action server ──────────────────────────────────────────────────────
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<PlanFootsteps>(
      this,
      "~/plan_footsteps",
      std::bind(&FootstepPlannerNode::handle_goal,   this, _1, _2),
      std::bind(&FootstepPlannerNode::handle_cancel, this, _1),
      std::bind(&FootstepPlannerNode::handle_accept, this, _1)
    );

    RCLCPP_INFO(get_logger(),
      "FootstepPlannerNode ready  hip_width=%.3f  max_step=%.3f  t_step=%.2f s",
      cfg.hip_width, cfg.max_step_length, cfg.t_single + cfg.t_double);
  }

private:
  // ── members ───────────────────────────────────────────────────────────────
  std::unique_ptr<FootstepPlanner>                         planner_;
  rclcpp_action::Server<PlanFootsteps>::SharedPtr          action_server_;
  rclcpp::Publisher<PoseArray>::SharedPtr                  pub_poses_;
  rclcpp::Publisher<MarkerArray>::SharedPtr                pub_markers_;
  std::shared_ptr<tf2_ros::Buffer>                         tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>              tf_listener_;

  std::string world_frame_, left_foot_frame_, right_foot_frame_;
  double      marker_foot_len_{0.23}, marker_foot_wid_{0.10};

  // ── parameter declarations ─────────────────────────────────────────────────
  void declare_params()
  {
    declare_parameter("g1.hip_width",        0.19);
    declare_parameter("g1.max_step_length",  0.20);
    declare_parameter("g1.max_step_width",   0.12);
    declare_parameter("g1.max_step_yaw",     0.30);
    declare_parameter("g1.t_single",         0.70);
    declare_parameter("g1.t_double",         0.10);
    declare_parameter("world_frame",         std::string("odom"));
    declare_parameter("left_foot_frame",     std::string("l_ankle_roll_link"));
    declare_parameter("right_foot_frame",    std::string("r_ankle_roll_link"));
    declare_parameter("foot_marker_length",  0.23);
    declare_parameter("foot_marker_width",   0.10);
  }

  // ── action callbacks ───────────────────────────────────────────────────────

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const PlanFootsteps::Goal> goal)
  {
    if (goal->n_steps <= 0 || goal->n_steps > 100) {
      RCLCPP_WARN(get_logger(), "Rejected goal: n_steps=%d (must be 1–100)", goal->n_steps);
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(),
      "Accepted goal: vx=%.2f  vy=%.2f  wz=%.2f  n_steps=%d",
      goal->vx, goal->vy, goal->wz, goal->n_steps);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePtr> /*gh*/)
  {
    RCLCPP_INFO(get_logger(), "Cancel request received.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accept(const std::shared_ptr<GoalHandlePtr> gh)
  {
    // Spin off a thread so the executor is not blocked
    std::thread([this, gh]() { execute(gh); }).detach();
  }

  // ── main execution ─────────────────────────────────────────────────────────

  void execute(const std::shared_ptr<GoalHandlePtr> gh)
  {
    const auto& goal = gh->get_goal();
    auto result      = std::make_shared<PlanFootsteps::Result>();

    // ── resolve starting foot positions ─────────────────────────────────────
    Vec3 left_pos, right_pos;
    double start_yaw = 0.0;

    if (goal->start_from_current) {
      bool ok = lookup_foot_tf(left_pos, right_pos);
      if (!ok) {
        RCLCPP_WARN(get_logger(), "TF lookup failed, using nominal stance.");
        auto [l, r] = planner_->nominal_stance();
        left_pos = l; right_pos = r;
      }
    } else {
      auto [l, r] = planner_->nominal_stance();
      left_pos = l; right_pos = r;
    }

    // ── plan footsteps with per-step feedback ────────────────────────────────
    std::vector<Footstep> steps;
    try {
      for (int i = 1; i <= goal->n_steps; ++i) {
        // Cancellation check
        if (gh->is_canceling()) {
          gh->canceled(result);
          RCLCPP_INFO(get_logger(), "Goal cancelled.");
          return;
        }

        // Plan incrementally (gives live feedback; last iteration = full plan)
        steps = planner_->generate(
          goal->vx, goal->vy, goal->wz, i,
          &left_pos, &right_pos, start_yaw);

        auto feedback          = std::make_shared<PlanFootsteps::Feedback>();
        feedback->steps_planned = i;
        feedback->progress      = static_cast<double>(i) / goal->n_steps;
        gh->publish_feedback(feedback);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Planner error: %s", e.what());
      result->success = false;
      result->message = e.what();
      gh->abort(result);
      return;
    }

    // ── convert and publish ──────────────────────────────────────────────────
    auto pose_array = build_pose_array(steps);
    pub_poses_->publish(pose_array);
    pub_markers_->publish(build_markers(steps));

    // ── fill result ──────────────────────────────────────────────────────────
    result->success   = true;
    result->message   = "Planned " + std::to_string(steps.size()) + " steps.";
    result->footsteps = pose_array;

    for (const auto& s : steps) {
      result->t_start.push_back(s.t_start);
      result->t_end.push_back(s.t_end);
      result->foot_id.push_back(static_cast<int>(s.foot));
    }

    gh->succeed(result);
    RCLCPP_INFO(get_logger(), "Plan complete: %zu footsteps published.", steps.size());
  }

  // ── TF2 helper ─────────────────────────────────────────────────────────────

  bool lookup_foot_tf(Vec3& left_out, Vec3& right_out)
  {
    auto lookup = [&](const std::string& frame, Vec3& out) -> bool {
      try {
        auto tf = tf_buffer_->lookupTransform(
          world_frame_, frame,
          rclcpp::Time(0),
          rclcpp::Duration::from_seconds(0.5));
        out[0] = tf.transform.translation.x;
        out[1] = tf.transform.translation.y;
        out[2] = tf.transform.translation.z;
        return true;
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "TF lookup failed (%s): %s", frame.c_str(), ex.what());
        return false;
      }
    };

    const bool ok_l = lookup(left_foot_frame_,  left_out);
    const bool ok_r = lookup(right_foot_frame_, right_out);
    return ok_l && ok_r;
  }

  // ── message builders ───────────────────────────────────────────────────────

  PoseArray build_pose_array(const std::vector<Footstep>& steps) const
  {
    PoseArray pa;
    pa.header.frame_id = world_frame_;
    pa.header.stamp    = now();

    for (const auto& s : steps) {
      Pose pose;
      pose.position.x = s.position[0];
      pose.position.y = s.position[1];
      pose.position.z = s.position[2];
      auto q = yaw_to_quaternion(s.yaw);
      pose.orientation.x = q.x;
      pose.orientation.y = q.y;
      pose.orientation.z = q.z;
      pose.orientation.w = q.w;
      pa.poses.push_back(pose);
    }
    return pa;
  }

  // ── RViz2 marker builder ───────────────────────────────────────────────────

  MarkerArray build_markers(const std::vector<Footstep>& steps) const
  {
    MarkerArray ma;
    const auto  stamp    = now();
    const auto  frame    = world_frame_;
    const auto  lifetime = rclcpp::Duration::from_seconds(0.0);   // persist

    // 1.  delete all previous markers
    Marker del; del.action = Marker::DELETEALL;
    ma.markers.push_back(del);

    // 2.  sequence line
    {
      Marker m;
      m.header.frame_id = frame;
      m.header.stamp    = stamp;
      m.ns              = "sequence_line";
      m.id              = 0;
      m.type            = Marker::LINE_STRIP;
      m.action          = Marker::ADD;
      m.scale.x         = 0.008;
      m.color           = COLOR_ARROW;
      m.lifetime        = lifetime;
      for (const auto& s : steps) {
        geometry_msgs::msg::Point pt;
        pt.x = s.position[0];
        pt.y = s.position[1];
        pt.z = s.position[2] + 0.005;
        m.points.push_back(pt);
      }
      ma.markers.push_back(m);
    }

    // 3.  per-step markers
    int id = 0;
    for (const auto& s : steps) {
      const auto q     = yaw_to_quaternion(s.yaw);
      const auto color = (s.foot == Foot::LEFT) ? COLOR_LEFT : COLOR_RIGHT;

      // Base pose
      Pose base_pose;
      base_pose.position.x    = s.position[0];
      base_pose.position.y    = s.position[1];
      base_pose.position.z    = s.position[2];
      base_pose.orientation.x = q.x;
      base_pose.orientation.y = q.y;
      base_pose.orientation.z = q.z;
      base_pose.orientation.w = q.w;

      // ── footprint rectangle ──────────────────────────────────────────────
      {
        Marker m;
        m.header.frame_id = frame;
        m.header.stamp    = stamp;
        m.ns              = "footprint";
        m.id              = id;
        m.type            = Marker::CUBE;
        m.action          = Marker::ADD;
        m.pose            = base_pose;
        m.pose.position.z += 0.003;
        m.scale.x         = marker_foot_len_;
        m.scale.y         = marker_foot_wid_;
        m.scale.z         = 0.005;
        m.color           = color;
        m.lifetime        = lifetime;
        ma.markers.push_back(m);
      }

      // ── index + side text ────────────────────────────────────────────────
      {
        Marker m;
        m.header.frame_id = frame;
        m.header.stamp    = stamp;
        m.ns              = "index";
        m.id              = id;
        m.type            = Marker::TEXT_VIEW_FACING;
        m.action          = Marker::ADD;
        m.pose            = base_pose;
        m.pose.position.z += 0.08;
        m.scale.z         = 0.05;
        m.color           = COLOR_TEXT;
        m.text            = std::to_string(id + 1) + "\n" +
                            std::string(s.foot == Foot::LEFT ? "L" : "R");
        m.lifetime        = lifetime;
        ma.markers.push_back(m);
      }

      // ── heading arrow ────────────────────────────────────────────────────
      {
        Marker m;
        m.header.frame_id = frame;
        m.header.stamp    = stamp;
        m.ns              = "heading";
        m.id              = id;
        m.type            = Marker::ARROW;
        m.action          = Marker::ADD;
        m.pose            = base_pose;
        m.pose.position.z += 0.01;
        m.scale.x         = marker_foot_len_ * 0.55;   // shaft length
        m.scale.y         = 0.018;                     // shaft diameter
        m.scale.z         = 0.025;                     // head diameter
        m.color           = COLOR_ARROW;
        m.lifetime        = lifetime;
        ma.markers.push_back(m);
      }

      ++id;
    }

    return ma;
  }
};  // class FootstepPlannerNode

}  // namespace g1_zmp_walking

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<g1_zmp_walking::FootstepPlannerNode>();

  // MultiThreadedExecutor lets the action server callbacks run in parallel
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), /*number_of_threads=*/4);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}