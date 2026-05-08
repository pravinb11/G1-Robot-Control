#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "builtin_interfaces/msg/duration.hpp"

#include "g1_zmp_walking/zmp_reference_generator.hpp"
#include "g1_zmp_walking/footstep_planner.hpp"   // for FootId enum + Quaternion

namespace g1_zmp_walking {

// ─────────────────────────────────────────────────────────────────────────────
class ZMPReferenceNode : public rclcpp::Node
{
public:
  explicit ZMPReferenceNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("zmp_reference_generator", opts)
  {
    // ── parameters ────────────────────────────────────────────────────────────
    declare_parameter("g1.t_single",   0.70);
    declare_parameter("g1.t_double",   0.10);
    declare_parameter("g1.hip_width",  0.19);
    declare_parameter("control.dt",    0.005);
    declare_parameter("control.n_preview", 200);
    declare_parameter("world_frame",   std::string("odom"));

    ZMPRefParams p;
    p.dt        = get_parameter("control.dt").as_double();
    p.t_single  = get_parameter("g1.t_single").as_double();
    p.t_double  = get_parameter("g1.t_double").as_double();
    p.n_preview = get_parameter("control.n_preview").as_int();
    gen_        = std::make_unique<ZMPReferenceGenerator>(p);

    hip_width_   = get_parameter("g1.hip_width").as_double();
    world_frame_ = get_parameter("world_frame").as_string();

    // ── publishers ────────────────────────────────────────────────────────────
    const auto qos = rclcpp::QoS(10);

    // Raw trajectory as Float64MultiArray: [x0,y0, x1,y1, ...]
    pub_raw_  = create_publisher<std_msgs::msg::Float64MultiArray>(
                  "~/zmp_trajectory", qos);

    // nav_msgs/Path for RViz2 visualisation
    pub_path_ = create_publisher<nav_msgs::msg::Path>(
                  "~/zmp_path", qos);

    // Single current desired ZMP point (for live monitoring)
    pub_point_ = create_publisher<geometry_msgs::msg::PointStamped>(
                   "~/zmp_current", qos);

    // MarkerArray visualisation of the ZMP trajectory
    pub_marker_ = create_publisher<visualization_msgs::msg::Marker>(
                    "~/zmp_marker", qos);

    // ── subscriber: receives footsteps from footstep_planner ─────────────────
    sub_footsteps_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/footstep_planner/footsteps",
      qos,
      std::bind(&ZMPReferenceNode::footsteps_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "ZMPReferenceNode ready  dt=%.4f  t_single=%.2f  t_double=%.2f  n_preview=%d",
      p.dt, p.t_single, p.t_double, p.n_preview);
  }

private:
  std::unique_ptr<ZMPReferenceGenerator>                         gen_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_footsteps_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  pub_raw_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               pub_path_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr  pub_point_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   pub_marker_;

  std::string world_frame_;
  double      hip_width_;

  // ── callback: new footstep plan received ─────────────────────────────────
  void footsteps_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty PoseArray, ignoring.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Received %zu footsteps — generating ZMP reference.",
                msg->poses.size());

    // ── reconstruct FootstepInput list from PoseArray ─────────────────────
    //    foot_id is encoded in the action result but not in PoseArray.
    //    We infer it from the y-position: left foot has y > 0.
    std::vector<FootstepInput> steps;
    steps.reserve(msg->poses.size());

    for (const auto& pose : msg->poses) {
      FootstepInput fs;
      fs.position = { pose.position.x, pose.position.y };
      // Infer foot side from lateral position relative to centre
      fs.foot = (pose.position.y >= 0.0) ? FootId::LEFT : FootId::RIGHT;
      steps.push_back(fs);
    }

    // ── nominal initial foot positions ────────────────────────────────────
    const double hw = hip_width_ * 0.5;
    const Point2D init_left  = {0.0,  hw};
    const Point2D init_right = {0.0, -hw};

    // ── generate pd[k] ────────────────────────────────────────────────────
    std::vector<Point2D> pd;
    try {
      pd = gen_->generate(steps, init_left, init_right);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "ZMP generation failed: %s", e.what());
      return;
    }

    RCLCPP_INFO(get_logger(),
      "Generated %zu ZMP samples  (%.2f s total)",
      pd.size(), static_cast<double>(pd.size()) * gen_->dt());

    // ── publish raw trajectory ────────────────────────────────────────────
    publish_raw(pd);

    // ── publish nav_msgs/Path for RViz2 ──────────────────────────────────
    publish_path(pd, msg->header.stamp);

    // ── publish tube marker (line strip) ─────────────────────────────────
    publish_marker(pd, msg->header.stamp);
  }

  // ── publishers ───────────────────────────────────────────────────────────

  void publish_raw(const std::vector<Point2D>& pd)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(pd.size() * 2);
    for (const auto& p : pd) {
      msg.data.push_back(p[0]);
      msg.data.push_back(p[1]);
    }
    // layout: dim[0] = samples, dim[1] = xy
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label  = "samples";
    msg.layout.dim[0].size   = static_cast<uint32_t>(pd.size());
    msg.layout.dim[0].stride = static_cast<uint32_t>(pd.size() * 2);
    msg.layout.dim[1].label  = "xy";
    msg.layout.dim[1].size   = 2;
    msg.layout.dim[1].stride = 2;
    pub_raw_->publish(msg);
  }

  void publish_path(const std::vector<Point2D>& pd,
                    const builtin_interfaces::msg::Time& stamp)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = world_frame_;
    path.header.stamp    = stamp;
    path.poses.reserve(pd.size());

    for (const auto& p : pd) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x    = p[0];
      ps.pose.position.y    = p[1];
      ps.pose.position.z    = 0.01;   // slightly above ground
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    pub_path_->publish(path);
  }

  void publish_marker(const std::vector<Point2D>& pd,
                      const builtin_interfaces::msg::Time& stamp)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp    = stamp;
    m.ns              = "zmp_trajectory";
    m.id              = 0;
    m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.scale.x         = 0.012;          // line width
    m.color.r         = 1.0f;
    m.color.g         = 0.3f;
    m.color.b         = 0.0f;
    m.color.a         = 0.9f;
    m.lifetime        = rclcpp::Duration::from_seconds(0.0);   // persist

    m.points.reserve(pd.size());
    for (const auto& p : pd) {
      geometry_msgs::msg::Point pt;
      pt.x = p[0];
      pt.y = p[1];
      pt.z = 0.01;
      m.points.push_back(pt);
    }
    pub_marker_->publish(m);
  }
};

}  // namespace g1_zmp_walking

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_zmp_walking::ZMPReferenceNode>());
  rclcpp::shutdown();
  return 0;
}