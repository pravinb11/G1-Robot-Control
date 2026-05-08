#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <Eigen/Dense>

#include "g1_zmp_walking/pattern_generator.hpp"

namespace g1_zmp_walking {

// ─────────────────────────────────────────────────────────────────────────────
class PatternGeneratorNode : public rclcpp::Node
{
public:
  explicit PatternGeneratorNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("pattern_generator", opts)
  {
    // ── declare parameters ────────────────────────────────────────────────────
    declare_parameter("g1.total_mass",    35.0);
    declare_parameter("g1.com_height",    0.75);
    declare_parameter("control.dt",       0.005);
    declare_parameter("control.n_preview",200);
    declare_parameter("control.n_init",   500);
    declare_parameter("zmp.Qe_scale",     1.0);
    declare_parameter("zmp.lambda_H",     10.0);
    declare_parameter("zmp.R_scale",      1e-6);
    declare_parameter("world_frame",      std::string("odom"));

    // ── build controller ──────────────────────────────────────────────────────
    PatternGeneratorParams p;
    p.M        = get_parameter("g1.total_mass").as_double();
    p.zc       = get_parameter("g1.com_height").as_double();
    p.dt       = get_parameter("control.dt").as_double();
    p.NL       = get_parameter("control.n_preview").as_int();
    p.Qe_scale = get_parameter("zmp.Qe_scale").as_double();
    p.lambda_H = get_parameter("zmp.lambda_H").as_double();
    p.R_scale  = get_parameter("zmp.R_scale").as_double();

    world_frame_ = get_parameter("world_frame").as_string();
    dt_          = p.dt;
    NL_          = p.NL;
    n_init_      = get_parameter("control.n_init").as_int();

    RCLCPP_INFO(get_logger(), "Solving DARE (NL=%d)… this may take a moment.", NL_);
    controller_ = std::make_unique<GeneralZMPPreviewController>(p);
    RCLCPP_INFO(get_logger(),
      "PatternGeneratorNode ready  M=%.1f kg  zc=%.3f m  dt=%.4f s  NL=%d",
      p.M, p.zc, p.dt, p.NL);

    // ── publishers ────────────────────────────────────────────────────────────
    const auto qos = rclcpp::QoS(10);

    // COM trajectory for IK node
    pub_com_path_   = create_publisher<nav_msgs::msg::Path>(
                        "~/com_path", qos);
    // ZMP actual trajectory for monitoring
    pub_zmp_actual_ = create_publisher<nav_msgs::msg::Path>(
                        "~/zmp_actual", qos);
    // Raw COM+HAM state array: [x,y, vx,vy, Hx,Hy, ax,ay, Hxd,Hyd] per step
    pub_raw_state_  = create_publisher<std_msgs::msg::Float64MultiArray>(
                        "~/pattern_state", qos);
    // RViz2 markers
    pub_markers_    = create_publisher<visualization_msgs::msg::MarkerArray>(
                        "~/pattern_markers", qos);

    // ── subscriber: ZMP reference from zmp_reference_node ────────────────────
    sub_zmp_ref_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/zmp_reference/zmp_trajectory",
      qos,
      std::bind(&PatternGeneratorNode::zmp_ref_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Subscribed to /zmp_reference/zmp_trajectory");
  }

private:
  std::unique_ptr<GeneralZMPPreviewController>                   controller_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_zmp_ref_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               pub_com_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               pub_zmp_actual_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  pub_raw_state_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  std::string world_frame_;
  double      dt_;
  int         NL_;
  int         n_init_{500};

  // ── callback: receive ZMP reference and run pattern generator ────────────
  void zmp_ref_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Unpack flat [x0,y0, x1,y1, ...] into Eigen matrix
    const int total = static_cast<int>(msg->data.size()) / 2;
    if (total < NL_) {
      RCLCPP_WARN(get_logger(),
        "ZMP reference too short (%d samples), need at least %d (NL).",
        total, NL_);
      return;
    }

    RCLCPP_INFO(get_logger(),
      "Received ZMP reference: %d samples — running pattern generator.", total);

    // Build (total × 2) matrix
    Eigen::MatrixXd pd(total, 2);
    for (int i = 0; i < total; ++i) {
      pd(i, 0) = msg->data[2 * i];
      pd(i, 1) = msg->data[2 * i + 1];
    }

    // ── reset controller for a fresh plan ────────────────────────────────────
    controller_->reset();

    // Storage
    const int N = total - NL_;   // usable steps (last NL_ rows are padding)
    std::vector<PatternState> states;
    states.reserve(N);

    // ── run preview controller step-by-step ───────────────────────────────────
    for (int k = 0; k < N; ++k) {
      // Extract preview window pd[k … k+NL-1]
      Eigen::MatrixXd window = pd.middleRows(k, NL_);

      PatternState s = controller_->step(window);
      states.push_back(s);
    }

    RCLCPP_INFO(get_logger(),
      "Pattern generation complete: %zu steps  (%.2f s)",
      states.size(), static_cast<double>(states.size()) * dt_);

    // ── publish ───────────────────────────────────────────────────────────────
    const auto stamp = now();
    publish_com_path(states, stamp);
    publish_zmp_actual(states, stamp);
    publish_raw_state(states);
    publish_markers(states, stamp);
  }

  // ── publishers ─────────────────────────────────────────────────────────────

  void publish_com_path(const std::vector<PatternState>& states,
                        const rclcpp::Time& stamp)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = world_frame_;
    path.header.stamp    = stamp;

    double t = 0.0;
    for (const auto& s : states) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.header.stamp = (stamp + rclcpp::Duration::from_seconds(t));
      ps.pose.position.x    = s.com_pos.x();
      ps.pose.position.y    = s.com_pos.y();
      ps.pose.position.z    = controller_->params().zc;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
      t += dt_;
    }
    pub_com_path_->publish(path);
  }

  void publish_zmp_actual(const std::vector<PatternState>& states,
                          const rclcpp::Time& stamp)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = world_frame_;
    path.header.stamp    = stamp;

    for (const auto& s : states) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x    = s.zmp.x();
      ps.pose.position.y    = s.zmp.y();
      ps.pose.position.z    = 0.01;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    pub_zmp_actual_->publish(path);
  }

  void publish_raw_state(const std::vector<PatternState>& states)
  {
    // Layout: [N × 10]  each row = [rx,ry, vx,vy, Hx,Hy, ax,ay, Hxd,Hyd]
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(states.size() * 10);
    for (const auto& s : states) {
      msg.data.push_back(s.com_pos.x()); msg.data.push_back(s.com_pos.y());
      msg.data.push_back(s.com_vel.x()); msg.data.push_back(s.com_vel.y());
      msg.data.push_back(s.ham.x());     msg.data.push_back(s.ham.y());
      msg.data.push_back(s.com_acc.x()); msg.data.push_back(s.com_acc.y());
      msg.data.push_back(s.ham_dot.x()); msg.data.push_back(s.ham_dot.y());
    }
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label  = "steps";
    msg.layout.dim[0].size   = static_cast<uint32_t>(states.size());
    msg.layout.dim[0].stride = static_cast<uint32_t>(states.size() * 10);
    msg.layout.dim[1].label  = "state";
    msg.layout.dim[1].size   = 10;
    msg.layout.dim[1].stride = 10;
    pub_raw_state_->publish(msg);
  }

  void publish_markers(const std::vector<PatternState>& states,
                       const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray ma;
    const auto lifetime = rclcpp::Duration::from_seconds(0.0);

    // Delete previous markers
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    // ── COM trajectory line (blue) ────────────────────────────────────────────
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = world_frame_;
      m.header.stamp    = stamp;
      m.ns              = "com_trajectory";
      m.id              = 0;
      m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action          = visualization_msgs::msg::Marker::ADD;
      m.scale.x         = 0.015;
      m.color.r = 0.1f; m.color.g = 0.4f; m.color.b = 1.0f; m.color.a = 0.9f;
      m.lifetime = lifetime;

      for (const auto& s : states) {
        geometry_msgs::msg::Point pt;
        pt.x = s.com_pos.x();
        pt.y = s.com_pos.y();
        pt.z = controller_->params().zc;
        m.points.push_back(pt);
      }
      ma.markers.push_back(m);
    }

    // ── actual ZMP trajectory line (red) ─────────────────────────────────────
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = world_frame_;
      m.header.stamp    = stamp;
      m.ns              = "zmp_actual";
      m.id              = 0;
      m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action          = visualization_msgs::msg::Marker::ADD;
      m.scale.x         = 0.010;
      m.color.r = 1.0f; m.color.g = 0.2f; m.color.b = 0.2f; m.color.a = 0.9f;
      m.lifetime = lifetime;

      for (const auto& s : states) {
        geometry_msgs::msg::Point pt;
        pt.x = s.zmp.x();
        pt.y = s.zmp.y();
        pt.z = 0.005;
        m.points.push_back(pt);
      }
      ma.markers.push_back(m);
    }

    // ── HAM magnitude spheres (every 20th step, colour = HAM magnitude) ───────
    {
      int id = 0;
      for (std::size_t k = 0; k < states.size(); k += 20) {
        const auto& s      = states[k];
        double ham_mag      = s.ham.norm();
        double ham_max      = 0.01;   // expected max ~0.5e-3 Nms, scale for vis

        visualization_msgs::msg::Marker m;
        m.header.frame_id = world_frame_;
        m.header.stamp    = stamp;
        m.ns              = "ham_spheres";
        m.id              = id++;
        m.type            = visualization_msgs::msg::Marker::SPHERE;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = s.com_pos.x();
        m.pose.position.y = s.com_pos.y();
        m.pose.position.z = controller_->params().zc + 0.05;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.03;
        // Colour: green→red with HAM magnitude
        double ratio = std::min(ham_mag / ham_max, 1.0);
        m.color.r = static_cast<float>(ratio);
        m.color.g = static_cast<float>(1.0 - ratio);
        m.color.b = 0.0f;
        m.color.a = 0.7f;
        m.lifetime = lifetime;
        ma.markers.push_back(m);
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
  rclcpp::spin(std::make_shared<g1_zmp_walking::PatternGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}