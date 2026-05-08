#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <Eigen/Dense>

#include "g1_zmp_walking/ik_solver.hpp"
#include "g1_zmp_walking/footstep_planner.hpp"   // for yaw_to_quaternion

namespace g1_zmp_walking {

// ─────────────────────────────────────────────────────────────────────────────
class IKSolverNode : public rclcpp::Node
{
public:
  explicit IKSolverNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("ik_solver", opts)
  {
    // ── parameters ────────────────────────────────────────────────────────────
    declare_parameter("g1.hip_width",      0.19);
    declare_parameter("g1.com_height",     0.75);
    declare_parameter("g1.l_thigh",        0.35);
    declare_parameter("g1.l_shank",        0.31);
    declare_parameter("g1.com_to_hip_z",   0.08);
    declare_parameter("g1.ankle_height",   0.04);
    declare_parameter("control.dt",        0.005);
    declare_parameter("world_frame",       std::string("odom"));

    // Joint limits (rad) — set from your URDF <limit> tags
    declare_parameter("g1.limits.hip_yaw_min",      -0.52);
    declare_parameter("g1.limits.hip_yaw_max",       0.52);
    declare_parameter("g1.limits.hip_roll_min",     -0.70);
    declare_parameter("g1.limits.hip_roll_max",      0.70);
    declare_parameter("g1.limits.hip_pitch_min",    -1.75);
    declare_parameter("g1.limits.hip_pitch_max",     0.60);
    declare_parameter("g1.limits.knee_min",          0.00);
    declare_parameter("g1.limits.knee_max",          2.50);
    declare_parameter("g1.limits.ankle_pitch_min",  -0.87);
    declare_parameter("g1.limits.ankle_pitch_max",   0.52);
    declare_parameter("g1.limits.ankle_roll_min",   -0.35);
    declare_parameter("g1.limits.ankle_roll_max",    0.35);

    // ── build IK solver ───────────────────────────────────────────────────────
    IKParams p;
    p.hip_width      = get_parameter("g1.hip_width").as_double();
    p.l_thigh        = get_parameter("g1.l_thigh").as_double();
    p.l_shank        = get_parameter("g1.l_shank").as_double();
    p.com_to_hip_z   = get_parameter("g1.com_to_hip_z").as_double();
    p.ankle_height   = get_parameter("g1.ankle_height").as_double();
    p.hip_yaw_min    = get_parameter("g1.limits.hip_yaw_min").as_double();
    p.hip_yaw_max    = get_parameter("g1.limits.hip_yaw_max").as_double();
    p.hip_roll_min   = get_parameter("g1.limits.hip_roll_min").as_double();
    p.hip_roll_max   = get_parameter("g1.limits.hip_roll_max").as_double();
    p.hip_pitch_min  = get_parameter("g1.limits.hip_pitch_min").as_double();
    p.hip_pitch_max  = get_parameter("g1.limits.hip_pitch_max").as_double();
    p.knee_min       = get_parameter("g1.limits.knee_min").as_double();
    p.knee_max       = get_parameter("g1.limits.knee_max").as_double();
    p.ankle_pitch_min= get_parameter("g1.limits.ankle_pitch_min").as_double();
    p.ankle_pitch_max= get_parameter("g1.limits.ankle_pitch_max").as_double();
    p.ankle_roll_min = get_parameter("g1.limits.ankle_roll_min").as_double();
    p.ankle_roll_max = get_parameter("g1.limits.ankle_roll_max").as_double();
    solver_          = std::make_unique<G1LegIK>(p);

    com_height_  = get_parameter("g1.com_height").as_double();
    ankle_height_= get_parameter("g1.ankle_height").as_double();
    dt_          = get_parameter("control.dt").as_double();
    world_frame_ = get_parameter("world_frame").as_string();

    // ── publishers ────────────────────────────────────────────────────────────
    const auto qos = rclcpp::QoS(10);

    // Joint trajectory for ros2_control
    pub_traj_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
                  "/joint_trajectory_controller/joint_trajectory", qos);

    // Joint states for monitoring / RViz2 robot model
    pub_js_ = create_publisher<sensor_msgs::msg::JointState>(
                "~/joint_states", qos);

    // RViz2 leg geometry markers
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                     "~/ik_markers", qos);

    // ── subscriber: COM path from pattern generator ───────────────────────────
    sub_com_path_ = create_subscription<nav_msgs::msg::Path>(
      "/pattern_generator/com_path", qos,
      std::bind(&IKSolverNode::com_path_cb, this, std::placeholders::_1));

    // ── subscriber: foot positions from footstep planner ─────────────────────
    sub_footsteps_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/footstep_planner/footsteps", qos,
      std::bind(&IKSolverNode::footsteps_cb, this, std::placeholders::_1));

    // Nominal foot positions (standing still)
    const double hw = p.hip_width * 0.5;
    left_foot_  = Eigen::Vector3d(0.0,  hw, ankle_height_);
    right_foot_ = Eigen::Vector3d(0.0, -hw, ankle_height_);

    RCLCPP_INFO(get_logger(),
      "IKSolverNode ready  l_thigh=%.3f  l_shank=%.3f  hw=%.3f  com_h=%.3f",
      p.l_thigh, p.l_shank, p.hip_width, com_height_);
  }

private:
  std::unique_ptr<G1LegIK>                                         solver_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr              sub_com_path_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr    sub_footsteps_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_traj_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr         pub_js_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  std::string     world_frame_;
  double          com_height_{0.75};
  double          ankle_height_{0.04};
  double          dt_{0.005};
  Eigen::Vector3d left_foot_, right_foot_;
  double          com_yaw_{0.0};

  // ── joint name ordering (matches ros2_control config) ────────────────────
  static constexpr int N_LEG_JOINTS = 6;
  const std::vector<std::string> joint_names_ = {
    "left_hip_yaw_joint",   "left_hip_roll_joint",
    "left_hip_pitch_joint", "left_knee_joint",
    "left_ankle_pitch_joint","left_ankle_roll_joint",
    "right_hip_yaw_joint",  "right_hip_roll_joint",
    "right_hip_pitch_joint","right_knee_joint",
    "right_ankle_pitch_joint","right_ankle_roll_joint",
  };

  // ── callbacks ─────────────────────────────────────────────────────────────

  void footsteps_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) return;
    // Extract yaw from last footstep quaternion for current heading estimate
    const auto& q = msg->poses.back().orientation;
    com_yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // Update foot positions from footstep sequence
    for (const auto& pose : msg->poses) {
      const bool is_left = (pose.position.y >= 0.0);
      Eigen::Vector3d pos(
        pose.position.x, pose.position.y, ankle_height_);
      if (is_left) left_foot_  = pos;
      else         right_foot_ = pos;
    }
  }

  void com_path_cb(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    RCLCPP_INFO(get_logger(),
      "Received COM path: %zu poses — solving IK.", msg->poses.size());

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header = msg->header;
    traj.joint_names = joint_names_;

    sensor_msgs::msg::JointState js;
    js.header = msg->header;
    js.name   = joint_names_;

    std::vector<LegJoints> left_qs, right_qs;
    int invalid_count = 0;

    for (std::size_t k = 0; k < msg->poses.size(); ++k) {
      const auto& pose = msg->poses[k];

      // COM position and yaw from path
      Eigen::Vector3d com_pos(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);

      // Extract yaw from pose quaternion
      const auto& q = pose.pose.orientation;
      const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

      // Solve IK for both legs
      // During walking the foot positions are updated by footsteps_cb;
      // here we use current stored positions (static per-step assumption)
      LegJoints ql = solver_->solve(com_pos, yaw, left_foot_,  yaw, true);
      LegJoints qr = solver_->solve(com_pos, yaw, right_foot_, yaw, false);

      if (!ql.valid || !qr.valid) ++invalid_count;

      left_qs.push_back(ql);
      right_qs.push_back(qr);

      // Build trajectory point
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = {
        ql.hip_yaw, ql.hip_roll, ql.hip_pitch,
        ql.knee, ql.ankle_pitch, ql.ankle_roll,
        qr.hip_yaw, qr.hip_roll, qr.hip_pitch,
        qr.knee, qr.ankle_pitch, qr.ankle_roll,
      };
      pt.time_from_start = rclcpp::Duration::from_seconds(
        static_cast<double>(k) * dt_);
      traj.points.push_back(pt);
    }

    if (invalid_count > 0)
      RCLCPP_WARN(get_logger(),
        "%d/%zu IK solutions were out-of-reach and clamped.",
        invalid_count, msg->poses.size());

    // Publish joint trajectory
    pub_traj_->publish(traj);

    // Publish current joint state (last pose)
    if (!left_qs.empty()) {
      const auto& ql = left_qs.back();
      const auto& qr = right_qs.back();
      js.position = {
        ql.hip_yaw, ql.hip_roll, ql.hip_pitch,
        ql.knee, ql.ankle_pitch, ql.ankle_roll,
        qr.hip_yaw, qr.hip_roll, qr.hip_pitch,
        qr.knee, qr.ankle_pitch, qr.ankle_roll,
      };
      pub_js_->publish(js);
    }

    RCLCPP_INFO(get_logger(),
      "IK complete: %zu trajectory points published.", traj.points.size());

    // Publish RViz2 markers showing leg geometry
    publish_markers(msg->poses, left_qs, right_qs, msg->header.stamp);
  }

  // ── RViz2 markers ─────────────────────────────────────────────────────────

  void publish_markers(
    const std::vector<geometry_msgs::msg::PoseStamped>& com_poses,
    const std::vector<LegJoints>& left_qs,
    const std::vector<LegJoints>& right_qs,
    const builtin_interfaces::msg::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray ma;
    const auto lifetime = rclcpp::Duration::from_seconds(0.0);

    // Delete previous
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    // Draw leg skeleton every 20th step
    const std::size_t stride = 20;
    int id = 0;

    for (std::size_t k = 0; k < com_poses.size(); k += stride) {
      const auto& pose = com_poses[k];
      Eigen::Vector3d com_pos(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);

      const auto& q_pose = pose.pose.orientation;
      const double yaw = std::atan2(
        2.0*(q_pose.w*q_pose.z + q_pose.x*q_pose.y),
        1.0 - 2.0*(q_pose.y*q_pose.y + q_pose.z*q_pose.z));

      // Draw both legs
      for (int leg = 0; leg < 2; ++leg) {
        const bool left     = (leg == 0);
        const auto& q       = left ? left_qs[k] : right_qs[k];
        const double sign   = left ? +1.0 : -1.0;

        // Hip position
        Eigen::Matrix3d Rb = rot_z(yaw);
        Eigen::Vector3d hip_off(0.0, sign*solver_->params().hip_width*0.5,
                                -solver_->params().com_to_hip_z);
        Eigen::Vector3d hip_pos = com_pos + Rb * hip_off;

        // Ankle position via FK
        Eigen::Vector3d ankle = solver_->forward_kinematics(hip_pos, q, yaw);

        // Knee position
        Eigen::Matrix3d R = rot_z(yaw)
                          * rot_z(q.hip_yaw)
                          * rot_x(q.hip_roll)
                          * rot_y(q.hip_pitch);
        Eigen::Vector3d knee_pos = hip_pos + R*Eigen::Vector3d(0,0,-solver_->params().l_thigh);

        // Line: hip → knee → ankle
        visualization_msgs::msg::Marker m;
        m.header.frame_id = world_frame_;
        m.header.stamp    = stamp;
        m.ns              = left ? "leg_left" : "leg_right";
        m.id              = id++;
        m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.scale.x         = 0.01;
        if (left) { m.color.r=0.1f; m.color.g=0.85f; m.color.b=0.3f; m.color.a=0.7f; }
        else       { m.color.r=0.2f; m.color.g=0.5f;  m.color.b=0.95f; m.color.a=0.7f; }
        m.lifetime = lifetime;

        auto pt = [](const Eigen::Vector3d& v) {
          geometry_msgs::msg::Point p;
          p.x=v.x(); p.y=v.y(); p.z=v.z(); return p;
        };
        m.points = { pt(hip_pos), pt(knee_pos), pt(ankle) };
        ma.markers.push_back(m);

        // Sphere at each joint
        for (const auto& jpos : {hip_pos, knee_pos, ankle}) {
          visualization_msgs::msg::Marker s;
          s.header = m.header;
          s.ns     = left ? "joints_left" : "joints_right";
          s.id     = id++;
          s.type   = visualization_msgs::msg::Marker::SPHERE;
          s.action = visualization_msgs::msg::Marker::ADD;
          s.pose.position.x = jpos.x();
          s.pose.position.y = jpos.y();
          s.pose.position.z = jpos.z();
          s.pose.orientation.w = 1.0;
          s.scale.x = s.scale.y = s.scale.z = 0.025;
          s.color = m.color;
          s.lifetime = lifetime;
          ma.markers.push_back(s);
        }
      }
    }

    pub_markers_->publish(ma);
  }

  // ── rotation helpers (duplicate from ik_solver.hpp for marker use) ────────
  static Eigen::Matrix3d rot_z(double a)
  {
    double c=std::cos(a), s=std::sin(a);
    Eigen::Matrix3d R; R<<c,-s,0, s,c,0, 0,0,1; return R;
  }
  static Eigen::Matrix3d rot_x(double a)
  {
    double c=std::cos(a), s=std::sin(a);
    Eigen::Matrix3d R; R<<1,0,0, 0,c,-s, 0,s,c; return R;
  }
  static Eigen::Matrix3d rot_y(double a)
  {
    double c=std::cos(a), s=std::sin(a);
    Eigen::Matrix3d R; R<<c,0,s, 0,1,0, -s,0,c; return R;
  }
};

}  // namespace g1_zmp_walking

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_zmp_walking::IKSolverNode>());
  rclcpp::shutdown();
  return 0;
}