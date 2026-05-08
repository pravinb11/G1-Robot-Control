#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <Eigen/Dense>

#include "g1_zmp_walking/swing_foot_trajectory.hpp"
#include "g1_zmp_walking/footstep_planner.hpp"   // for Foot enum + FootId

namespace g1_zmp_walking {

// ─────────────────────────────────────────────────────────────────────────────

class SwingFootNode : public rclcpp::Node
{
public:
  explicit SwingFootNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("swing_foot_trajectory", opts)
  {
    // ── parameters ────────────────────────────────────────────────────────────
    declare_parameter("g1.hip_width",         0.19);
    declare_parameter("g1.ankle_height",      0.04);
    declare_parameter("g1.t_single",          0.70);
    declare_parameter("g1.t_double",          0.10);
    declare_parameter("control.dt",           0.005);
    declare_parameter("control.n_init",       500);
    declare_parameter("swing.step_height",    0.06);
    declare_parameter("swing.profile",        std::string("cubic"));
    declare_parameter("world_frame",          std::string("odom"));

    const double hw          = get_parameter("g1.hip_width").as_double()    * 0.5;
    ankle_height_            = get_parameter("g1.ankle_height").as_double();
    t_single_                = get_parameter("g1.t_single").as_double();
    t_double_                = get_parameter("g1.t_double").as_double();
    dt_                      = get_parameter("control.dt").as_double();
    n_init_                  = get_parameter("control.n_init").as_int();
    world_frame_             = get_parameter("world_frame").as_string();

    // n_samples per phase
    n_single_ = static_cast<int>(std::round(t_single_ / dt_));
    n_double_ = static_cast<int>(std::round(t_double_ / dt_));
    n_step_   = n_single_ + n_double_;

    // Build swing generator
    const double step_height = get_parameter("swing.step_height").as_double();
    const std::string profile_str = get_parameter("swing.profile").as_string();
    SwingProfile profile = (profile_str == "parabolic")
                         ? SwingProfile::PARABOLIC
                         : SwingProfile::CUBIC;
    swing_ = std::make_unique<SwingFootTrajectory>(step_height, profile);

    // Nominal foot positions
    left_foot_  = Eigen::Vector3d(0.0,  hw, ankle_height_);
    right_foot_ = Eigen::Vector3d(0.0, -hw, ankle_height_);

    // ── publishers ────────────────────────────────────────────────────────────
    const auto qos = rclcpp::QoS(10);

    // Left/right foot paths for IK node
    pub_left_path_  = create_publisher<nav_msgs::msg::Path>(
                        "~/left_foot_path",  qos);
    pub_right_path_ = create_publisher<nav_msgs::msg::Path>(
                        "~/right_foot_path", qos);

    // Raw foot state array: [lx,ly,lz, lvx,lvy,lvz, rx,ry,rz, rvx,rvy,rvz] per step
    pub_raw_state_  = create_publisher<std_msgs::msg::Float64MultiArray>(
                        "~/foot_state", qos);

    // RViz2 markers
    pub_markers_    = create_publisher<visualization_msgs::msg::MarkerArray>(
                        "~/swing_markers", qos);

    // ── subscribers ───────────────────────────────────────────────────────────
    // Footsteps from planner → tells us which foot swings when
    sub_footsteps_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/footstep_planner/footsteps", qos,
      std::bind(&SwingFootNode::footsteps_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "SwingFootNode ready  step_height=%.3f m  profile=%s  "
      "n_single=%d  n_double=%d  dt=%.4f",
      step_height, profile_str.c_str(), n_single_, n_double_, dt_);
  }

private:
  std::unique_ptr<SwingFootTrajectory>                             swing_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr   sub_footsteps_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                pub_left_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                pub_right_path_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   pub_raw_state_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  std::string     world_frame_;
  double          ankle_height_{0.04};
  double          t_single_{0.70}, t_double_{0.10}, dt_{0.005};
  int             n_single_{140}, n_double_{20}, n_step_{160}, n_init_{500};
  Eigen::Vector3d left_foot_, right_foot_;

  // ── per-timestep foot state ───────────────────────────────────────────────
  struct FootState {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;   // d(pos)/dt  (m/s)
  };

  // ── callback ──────────────────────────────────────────────────────────────

  void footsteps_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    RCLCPP_INFO(get_logger(),
      "Received %zu footsteps — generating swing trajectories.", msg->poses.size());

    // Reconstruct footstep sequence: position + foot side
    struct Step {
      Eigen::Vector3d pos;   // landing position
      bool            left;  // true = left foot swings
    };

    std::vector<Step> steps;
    steps.reserve(msg->poses.size());
    for (const auto& pose : msg->poses) {
      steps.push_back({
        Eigen::Vector3d(pose.position.x, pose.position.y, ankle_height_),
        pose.position.y >= 0.0   // y > 0 → left foot
      });
    }

    // ── build full trajectory ─────────────────────────────────────────────────
    // Total samples = n_init + n_steps * n_step_
    const int N = n_init_ + static_cast<int>(steps.size()) * n_step_;
    std::vector<FootState> left_traj(N), right_traj(N);

    // Initialisation phase: both feet at nominal stance
    Eigen::Vector3d cur_left  = left_foot_;
    Eigen::Vector3d cur_right = right_foot_;

    for (int k = 0; k < n_init_; ++k) {
      left_traj[k]  = { cur_left,  Eigen::Vector3d::Zero() };
      right_traj[k] = { cur_right, Eigen::Vector3d::Zero() };
    }

    // Walk phase: for each footstep generate single + double support
    int k = n_init_;
    for (const auto& step : steps) {
      // Which foot is swinging vs supporting?
      Eigen::Vector3d& swing_start = step.left ? cur_left  : cur_right;
      Eigen::Vector3d& swing_land  = step.left ? cur_left  : cur_right;  // updated after
      const Eigen::Vector3d p_land = step.pos;

      // Cache start of swing
      const Eigen::Vector3d p_start = swing_start;

      // ── single-support: swing foot arcs from p_start → p_land ────────────
      for (int i = 0; i < n_single_; ++i) {
        const double phase = static_cast<double>(i) / n_single_;
        const SwingState ss = swing_->get_state(p_start, p_land, phase);

        FootState& swing_state   = step.left ? left_traj[k]  : right_traj[k];
        FootState& support_state = step.left ? right_traj[k] : left_traj[k];

        swing_state   = { ss.position, ss.velocity / t_single_ };
        support_state = { step.left ? cur_right : cur_left,
                          Eigen::Vector3d::Zero() };
        ++k;
      }

      // Foot lands
      swing_land = p_land;
      if (step.left) cur_left  = p_land;
      else           cur_right = p_land;

      // ── double-support: both feet on ground ───────────────────────────────
      for (int i = 0; i < n_double_; ++i) {
        left_traj[k]  = { cur_left,  Eigen::Vector3d::Zero() };
        right_traj[k] = { cur_right, Eigen::Vector3d::Zero() };
        ++k;
      }
    }

    RCLCPP_INFO(get_logger(),
      "Generated %d foot trajectory samples  (%.2f s)",
      N, static_cast<double>(N) * dt_);

    // ── publish ───────────────────────────────────────────────────────────────
    const auto stamp = now();
    publish_path(left_traj,  pub_left_path_,  stamp, "left");
    publish_path(right_traj, pub_right_path_, stamp, "right");
    publish_raw_state(left_traj, right_traj);
    publish_markers(left_traj, right_traj, stamp);
  }

  // ── publishers ─────────────────────────────────────────────────────────────

  void publish_path(
    const std::vector<FootState>& traj,
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub,
    const rclcpp::Time& stamp,
    const std::string& /*side*/)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = world_frame_;
    path.header.stamp    = stamp;

    for (std::size_t k = 0; k < traj.size(); ++k) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.header.stamp = stamp + rclcpp::Duration::from_seconds(
                          static_cast<double>(k) * dt_);
      ps.pose.position.x    = traj[k].pos.x();
      ps.pose.position.y    = traj[k].pos.y();
      ps.pose.position.z    = traj[k].pos.z();
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    pub->publish(path);
  }

  void publish_raw_state(
    const std::vector<FootState>& left,
    const std::vector<FootState>& right)
  {
    // Layout: N × 12  [lx,ly,lz, lvx,lvy,lvz, rx,ry,rz, rvx,rvy,rvz]
    std_msgs::msg::Float64MultiArray msg;
    const int N = static_cast<int>(left.size());
    msg.data.reserve(N * 12);

    for (int k = 0; k < N; ++k) {
      for (int d = 0; d < 3; ++d) msg.data.push_back(left[k].pos[d]);
      for (int d = 0; d < 3; ++d) msg.data.push_back(left[k].vel[d]);
      for (int d = 0; d < 3; ++d) msg.data.push_back(right[k].pos[d]);
      for (int d = 0; d < 3; ++d) msg.data.push_back(right[k].vel[d]);
    }

    msg.layout.dim.resize(2);
    msg.layout.dim[0].label  = "samples";
    msg.layout.dim[0].size   = static_cast<uint32_t>(N);
    msg.layout.dim[0].stride = static_cast<uint32_t>(N * 12);
    msg.layout.dim[1].label  = "lx_ly_lz_lvx_lvy_lvz_rx_ry_rz_rvx_rvy_rvz";
    msg.layout.dim[1].size   = 12;
    msg.layout.dim[1].stride = 12;
    pub_raw_state_->publish(msg);
  }

  void publish_markers(
    const std::vector<FootState>& left,
    const std::vector<FootState>& right,
    const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray ma;
    const auto lifetime = rclcpp::Duration::from_seconds(0.0);

    // Delete previous markers
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    // ── Left foot path (green) ────────────────────────────────────────────────
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = world_frame_;
      m.header.stamp    = stamp;
      m.ns              = "left_foot_path";
      m.id              = 0;
      m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action          = visualization_msgs::msg::Marker::ADD;
      m.scale.x         = 0.008;
      m.color.r = 0.1f; m.color.g = 0.9f; m.color.b = 0.3f; m.color.a = 0.9f;
      m.lifetime = lifetime;

      // Downsample for performance
      const int stride = std::max(1, static_cast<int>(left.size()) / 500);
      for (std::size_t k = 0; k < left.size(); k += stride) {
        geometry_msgs::msg::Point pt;
        pt.x = left[k].pos.x();
        pt.y = left[k].pos.y();
        pt.z = left[k].pos.z();
        m.points.push_back(pt);
      }
      ma.markers.push_back(m);
    }

    // ── Right foot path (blue) ────────────────────────────────────────────────
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = world_frame_;
      m.header.stamp    = stamp;
      m.ns              = "right_foot_path";
      m.id              = 0;
      m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action          = visualization_msgs::msg::Marker::ADD;
      m.scale.x         = 0.008;
      m.color.r = 0.2f; m.color.g = 0.5f; m.color.b = 1.0f; m.color.a = 0.9f;
      m.lifetime = lifetime;

      const int stride = std::max(1, static_cast<int>(right.size()) / 500);
      for (std::size_t k = 0; k < right.size(); k += stride) {
        geometry_msgs::msg::Point pt;
        pt.x = right[k].pos.x();
        pt.y = right[k].pos.y();
        pt.z = right[k].pos.z();
        m.points.push_back(pt);
      }
      ma.markers.push_back(m);
    }

    // ── Apex markers: sphere at peak of each swing arc ────────────────────────
    {
      int id = 0;
      auto add_apex = [&](const std::vector<FootState>& traj,
                          bool left_side, int step_idx)
      {
        // Peak is at n_init + step_idx*n_step_ + n_single_/2
        const int k_peak = n_init_ + step_idx * n_step_ + n_single_ / 2;
        if (k_peak >= static_cast<int>(traj.size())) return;

        visualization_msgs::msg::Marker s;
        s.header.frame_id = world_frame_;
        s.header.stamp    = stamp;
        s.ns              = left_side ? "apex_left" : "apex_right";
        s.id              = id++;
        s.type            = visualization_msgs::msg::Marker::SPHERE;
        s.action          = visualization_msgs::msg::Marker::ADD;
        s.pose.position.x = traj[k_peak].pos.x();
        s.pose.position.y = traj[k_peak].pos.y();
        s.pose.position.z = traj[k_peak].pos.z();
        s.pose.orientation.w = 1.0;
        s.scale.x = s.scale.y = s.scale.z = 0.04;
        if (left_side) {
          s.color.r=0.1f; s.color.g=0.9f; s.color.b=0.3f; s.color.a=1.0f;
        } else {
          s.color.r=0.2f; s.color.g=0.5f; s.color.b=1.0f; s.color.a=1.0f;
        }
        s.lifetime = lifetime;
        ma.markers.push_back(s);
      };

      // Estimate number of steps from trajectory length
      const int n_steps = (static_cast<int>(left.size()) - n_init_) / n_step_;
      for (int i = 0; i < n_steps; ++i) {
        // Left swings on odd steps, right on even (depends on first_swing)
        // Check which foot is actually moving by comparing adjacent positions
        const int k_mid = n_init_ + i * n_step_ + n_single_ / 2;
        if (k_mid < static_cast<int>(left.size())) {
          const bool left_moving =
            (left[k_mid].pos.z() > ankle_height_ + 0.005);
          add_apex(left_moving ? left : right, left_moving, i);
        }
      }
    }

    pub_markers_->publish(ma);
  }
};

}  // namespace g1_zmp_walking

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_zmp_walking::SwingFootNode>());
  rclcpp::shutdown();
  return 0;
}