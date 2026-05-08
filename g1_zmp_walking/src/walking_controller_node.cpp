#include <memory>
#include <string>
#include <vector>
#include <array>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Messages
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Eigen
#include <Eigen/Dense>

// Our modules (all header-only, no ROS)
#include "g1_zmp_walking/footstep_planner.hpp"
#include "g1_zmp_walking/zmp_reference_generator.hpp"
#include "g1_zmp_walking/pattern_generator.hpp"
#include "g1_zmp_walking/swing_foot_trajectory.hpp"
#include "g1_zmp_walking/ik_solver.hpp"
#include "g1_zmp_walking/walking_fsm.hpp"

// Action interface
#include "g1_zmp_walking/action/plan_footsteps.hpp"

namespace g1_zmp_walking {

using PlanFootsteps = g1_zmp_walking::action::PlanFootsteps;
using GoalHandle    = rclcpp_action::ClientGoalHandle<PlanFootsteps>;

// ─────────────────────────────────────────────────────────────────────────────
class G1WalkingController : public rclcpp::Node
{
public:
  explicit G1WalkingController(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("g1_walking_controller", opts)
  {
    // ── declare parameters ────────────────────────────────────────────────────
    declare_parameter("g1.total_mass",        35.0);
    declare_parameter("g1.com_height",        0.75);
    declare_parameter("g1.hip_width",         0.19);
    declare_parameter("g1.ankle_height",      0.04);
    declare_parameter("g1.l_thigh",           0.35);
    declare_parameter("g1.l_shank",           0.31);
    declare_parameter("g1.com_to_hip_z",      0.08);
    declare_parameter("g1.t_single",          0.70);
    declare_parameter("g1.t_double",          0.10);
    declare_parameter("g1.max_step_length",   0.20);
    declare_parameter("g1.max_step_width",    0.12);
    declare_parameter("g1.max_step_yaw",      0.30);
    declare_parameter("control.dt",           0.005);
    declare_parameter("control.n_preview",    200);
    declare_parameter("control.n_init",       500);
    declare_parameter("zmp.Qe_scale",         1.0);
    declare_parameter("zmp.lambda_H",         10.0);
    declare_parameter("zmp.R_scale",          1e-6);
    declare_parameter("swing.step_height",    0.06);
    declare_parameter("world_frame",          std::string("odom"));

    // ── read parameters ───────────────────────────────────────────────────────
    dt_          = get_parameter("control.dt").as_double();
    NL_          = get_parameter("control.n_preview").as_int();
    n_init_      = get_parameter("control.n_init").as_int();
    zc_          = get_parameter("g1.com_height").as_double();
    ankle_height_= get_parameter("g1.ankle_height").as_double();
    world_frame_ = get_parameter("world_frame").as_string();

    const double hw = get_parameter("g1.hip_width").as_double() * 0.5;

    // ── build modules ─────────────────────────────────────────────────────────

    // Footstep planner
    FootstepPlannerParams fpp;
    fpp.hip_width       = get_parameter("g1.hip_width").as_double();
    fpp.max_step_length = get_parameter("g1.max_step_length").as_double();
    fpp.max_step_width  = get_parameter("g1.max_step_width").as_double();
    fpp.max_step_yaw    = get_parameter("g1.max_step_yaw").as_double();
    fpp.t_single        = get_parameter("g1.t_single").as_double();
    fpp.t_double        = get_parameter("g1.t_double").as_double();
    planner_ = std::make_unique<FootstepPlanner>(fpp);

    // ZMP reference generator
    ZMPRefParams zrp;
    zrp.dt        = dt_;
    zrp.t_single  = fpp.t_single;
    zrp.t_double  = fpp.t_double;
    zrp.n_preview = NL_;
    zrp.n_init    = n_init_;
    zmp_ref_      = std::make_unique<ZMPReferenceGenerator>(zrp);

    // Pattern generator (solves DARE on construction)
    PatternGeneratorParams pgp;
    pgp.dt       = dt_;
    pgp.zc       = zc_;
    pgp.M        = get_parameter("g1.total_mass").as_double();
    pgp.NL       = NL_;
    pgp.Qe_scale = get_parameter("zmp.Qe_scale").as_double();
    pgp.lambda_H = get_parameter("zmp.lambda_H").as_double();
    pgp.R_scale  = get_parameter("zmp.R_scale").as_double();
    RCLCPP_INFO(get_logger(), "Solving DARE for preview controller...");
    pattern_gen_ = std::make_unique<GeneralZMPPreviewController>(pgp);
    RCLCPP_INFO(get_logger(), "DARE solved OK.");

    // Swing foot trajectory
    swing_ = std::make_unique<SwingFootTrajectory>(
               get_parameter("swing.step_height").as_double(),
               SwingProfile::CUBIC);

    // IK solver
    IKParams ikp;
    ikp.hip_width      = fpp.hip_width;
    ikp.l_thigh        = get_parameter("g1.l_thigh").as_double();
    ikp.l_shank        = get_parameter("g1.l_shank").as_double();
    ikp.com_to_hip_z   = get_parameter("g1.com_to_hip_z").as_double();
    ikp.ankle_height   = ankle_height_;
    ik_ = std::make_unique<G1LegIK>(ikp);

    // Walking FSM
    FSMParams fsmp;
    fsmp.t_single = fpp.t_single;
    fsmp.t_double = fpp.t_double;
    fsmp.dt       = dt_;
    fsm_ = std::make_unique<WalkingFSM>(fsmp);

    // FSM callbacks
    fsm_->on_lift_off = [this](const FSMFootstep& step) {
      RCLCPP_INFO(get_logger(), "LIFT-OFF  %s → [%.3f, %.3f]",
        step.foot == SwingFoot::LEFT ? "LEFT" : "RIGHT",
        step.position.x(), step.position.y());
      // Update swing start position
      if (step.foot == SwingFoot::LEFT) swing_start_left_  = foot_left_;
      else                               swing_start_right_ = foot_right_;
    };

    fsm_->on_touch_down = [this](const FSMFootstep& step) {
      RCLCPP_INFO(get_logger(), "TOUCH-DOWN %s",
        step.foot == SwingFoot::LEFT ? "LEFT" : "RIGHT");
      // Update foot position on landing
      if (step.foot == SwingFoot::LEFT) foot_left_  = step.position;
      else                               foot_right_ = step.position;
      com_yaw_ = step.yaw;
    };

    fsm_->on_walk_complete = [this]() {
      RCLCPP_INFO(get_logger(), "Walk complete.");
      if (control_timer_) control_timer_->cancel();
      publish_status("COMPLETE");
    };

    // ── initial foot positions ────────────────────────────────────────────────
    foot_left_         = Eigen::Vector3d(0.0,  hw, ankle_height_);
    foot_right_        = Eigen::Vector3d(0.0, -hw, ankle_height_);
    swing_start_left_  = foot_left_;
    swing_start_right_ = foot_right_;

    // ── publishers ────────────────────────────────────────────────────────────
    const auto qos = rclcpp::QoS(10);
    pub_traj_   = create_publisher<trajectory_msgs::msg::JointTrajectory>(
                    "/joint_trajectory_controller/joint_trajectory", qos);
    pub_js_     = create_publisher<sensor_msgs::msg::JointState>(
                    "~/joint_states", qos);
    pub_status_ = create_publisher<std_msgs::msg::String>(
                    "~/status", qos);
    pub_markers_= create_publisher<visualization_msgs::msg::MarkerArray>(
                    "~/debug_markers", qos);

    // ── subscriber: receive new walk commands ─────────────────────────────────
    sub_cmd_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/footstep_planner/footsteps", qos,
      std::bind(&G1WalkingController::footsteps_cb, this, std::placeholders::_1));

    // ── action client: send goals to footstep planner ─────────────────────────
    action_client_ = rclcpp_action::create_client<PlanFootsteps>(
                       this, "/footstep_planner/plan_footsteps");

    RCLCPP_INFO(get_logger(),
      "G1WalkingController ready — waiting for footstep goals.");
    publish_status("IDLE");
  }

  // ── public API: request a walk ────────────────────────────────────────────

  void request_walk(double vx, double vy, double wz, int n_steps)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "Footstep planner action server not available.");
      return;
    }
    auto goal = PlanFootsteps::Goal();
    goal.vx                 = vx;
    goal.vy                 = vy;
    goal.wz                 = wz;
    goal.n_steps            = n_steps;
    goal.start_from_current = false;

    RCLCPP_INFO(get_logger(),
      "Requesting walk: vx=%.2f vy=%.2f wz=%.2f n_steps=%d",
      vx, vy, wz, n_steps);
    action_client_->async_send_goal(goal);
  }

private:
  // ── modules ───────────────────────────────────────────────────────────────
  std::unique_ptr<FootstepPlanner>               planner_;
  std::unique_ptr<ZMPReferenceGenerator>         zmp_ref_;
  std::unique_ptr<GeneralZMPPreviewController>   pattern_gen_;
  std::unique_ptr<SwingFootTrajectory>           swing_;
  std::unique_ptr<G1LegIK>                       ik_;
  std::unique_ptr<WalkingFSM>                    fsm_;

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr                                       control_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr     sub_cmd_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_traj_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr          pub_js_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  pub_markers_;
  rclcpp_action::Client<PlanFootsteps>::SharedPtr                     action_client_;

  // ── runtime state ─────────────────────────────────────────────────────────
  std::string     world_frame_;
  double          dt_{0.005}, zc_{0.75}, ankle_height_{0.04};
  int             NL_{200}, n_init_{500};
  int             k_{0};                      // timestep counter
  double          com_yaw_{0.0};

  Eigen::Vector3d foot_left_, foot_right_;
  Eigen::Vector3d swing_start_left_, swing_start_right_;

  std::vector<Point2D> pd_;                   // ZMP reference trajectory
  std::mutex           pd_mutex_;

  // ── Joint names (ros2_control order) ─────────────────────────────────────
  // Joint order MUST match controllers.yaml joint_trajectory_controller joints
  const std::vector<std::string> joint_names_ = {
    "left_hip_pitch_joint",  "left_hip_roll_joint",
    "left_hip_yaw_joint",    "left_knee_joint",
    "left_ankle_pitch_joint","left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint",
    "right_hip_yaw_joint",   "right_knee_joint",
    "right_ankle_pitch_joint","right_ankle_roll_joint",
  };

  // ── footstep callback: new plan received → build trajectories ────────────

  void footsteps_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    // Stop existing walk
    if (control_timer_) control_timer_->cancel();
    pattern_gen_->reset();
    fsm_->stop();
    k_ = 0;

    RCLCPP_INFO(get_logger(),
      "Received %zu footsteps — building walk plan.", msg->poses.size());
    publish_status("PLANNING");

    // ── 1. Convert PoseArray → FootstepInput for ZMP reference ───────────────
    std::vector<FootstepInput> zmp_steps;
    std::vector<FSMFootstep>   fsm_steps;
    zmp_steps.reserve(msg->poses.size());
    fsm_steps.reserve(msg->poses.size());

    for (const auto& pose : msg->poses) {
      FootstepInput fi;
      fi.position = { pose.position.x, pose.position.y };
      fi.foot     = (pose.position.y >= 0.0) ? FootId::LEFT : FootId::RIGHT;
      zmp_steps.push_back(fi);

      FSMFootstep fs;
      fs.foot     = (pose.position.y >= 0.0) ? SwingFoot::LEFT : SwingFoot::RIGHT;
      fs.position = Eigen::Vector3d(pose.position.x, pose.position.y, ankle_height_);
      const auto& q = pose.orientation;
      fs.yaw = std::atan2(
        2.0*(q.w*q.z + q.x*q.y),
        1.0 - 2.0*(q.y*q.y + q.z*q.z));
      fsm_steps.push_back(fs);
    }

    // ── 2. Generate ZMP reference ─────────────────────────────────────────────
    {
      std::lock_guard<std::mutex> lock(pd_mutex_);
      const Point2D il = { foot_left_.x(),  foot_left_.y()  };
      const Point2D ir = { foot_right_.x(), foot_right_.y() };
      pd_ = zmp_ref_->generate(zmp_steps, il, ir);
    }

    RCLCPP_INFO(get_logger(),
      "ZMP reference: %zu samples  (%.2f s)",
      pd_.size(), static_cast<double>(pd_.size()) * dt_);

    // ── 3. Start FSM (delayed by n_init_ ticks to align with pd init phase) ──
    // We start the FSM immediately but SINGLE_SUPPORT only begins after n_init_
    // ticks — the pattern generator is settling during that time.
    fsm_->start(fsm_steps);

    // Override FSM: stay in INIT_DOUBLE for n_init_ steps
    // We do this by tracking k_ and only letting FSM run after k_ >= n_init_
    fsm_init_done_ = false;

    // ── 4. Start control timer ────────────────────────────────────────────────
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&G1WalkingController::control_loop, this));

    publish_status("WALKING");
    RCLCPP_INFO(get_logger(), "Control loop started at %.0f Hz.", 1.0/dt_);
  }

  bool fsm_init_done_{false};

  // ── 200 Hz control loop ───────────────────────────────────────────────────

  void control_loop()
  {
    std::lock_guard<std::mutex> lock(pd_mutex_);

    // ── Guard: need full preview window ──────────────────────────────────────
    if (k_ + NL_ >= static_cast<int>(pd_.size())) {
      RCLCPP_INFO_ONCE(get_logger(), "Preview window exhausted — stopping.");
      control_timer_->cancel();
      publish_status("DONE");
      return;
    }

    // ── 1. Extract preview window pd[k : k+NL] ───────────────────────────────
    Eigen::MatrixXd pd_window(NL_, 2);
    for (int i = 0; i < NL_; ++i) {
      pd_window(i, 0) = pd_[k_ + i][0];
      pd_window(i, 1) = pd_[k_ + i][1];
    }

    // ── 2. Pattern generator → COM position ──────────────────────────────────
    PatternState ps = pattern_gen_->step(pd_window);
    const Eigen::Vector3d com_pos(ps.com_pos.x(), ps.com_pos.y(), zc_);

    // ── 3. FSM update (delayed by n_init_ ticks) ─────────────────────────────
    FSMOutput fsm_out;
    if (k_ < n_init_) {
      // Still in init phase — return dummy idle output
      fsm_out.state = WalkState::INIT_DOUBLE;
    } else {
      if (!fsm_init_done_) {
        RCLCPP_INFO(get_logger(), "Init phase complete — FSM walking begins.");
        fsm_init_done_ = true;
      }
      fsm_out = fsm_->update();
    }

    // ── 4. Foot targets ───────────────────────────────────────────────────────
    Eigen::Vector3d target_left  = foot_left_;
    Eigen::Vector3d target_right = foot_right_;

    if (fsm_out.is_single_support()) {
      const double phase = fsm_out.swing_phase;
      if (fsm_out.swing_foot == SwingFoot::LEFT) {
        target_left = swing_->get_pos(swing_start_left_,
                                      fsm_out.current_step.position, phase);
      } else {
        target_right = swing_->get_pos(swing_start_right_,
                                       fsm_out.current_step.position, phase);
      }
    }

    // ── 5. IK ─────────────────────────────────────────────────────────────────
    LegJoints ql = ik_->solve(com_pos, com_yaw_, target_left,  com_yaw_, true);
    LegJoints qr = ik_->solve(com_pos, com_yaw_, target_right, com_yaw_, false);

    if (!ql.valid || !qr.valid)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "IK out of reach at k=%d — joint angles clamped.", k_);

    // ── 6. Publish joint trajectory ───────────────────────────────────────────
    publish_joints(ql, qr);

    // ── 7. Debug markers every 20th step ─────────────────────────────────────
    if (k_ % 20 == 0)
      publish_debug_marker(com_pos, target_left, target_right, fsm_out);

    ++k_;
  }

  // ── publishers ─────────────────────────────────────────────────────────────

  void publish_joints(const LegJoints& ql, const LegJoints& qr)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.frame_id = world_frame_;
    traj.header.stamp    = now();
    traj.joint_names     = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    // Order matches controllers.yaml: pitch, roll, yaw, knee, ankle_pitch, ankle_roll
    pt.positions = {
      ql.hip_pitch, ql.hip_roll, ql.hip_yaw,
      ql.knee, ql.ankle_pitch, ql.ankle_roll,
      qr.hip_pitch, qr.hip_roll, qr.hip_yaw,
      qr.knee, qr.ankle_pitch, qr.ankle_roll,
    };
    // Time from start = one control timestep
    pt.time_from_start = rclcpp::Duration::from_seconds(dt_);
    traj.points.push_back(pt);
    pub_traj_->publish(traj);

    // Also publish JointState for monitoring
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name         = joint_names_;
    js.position     = pt.positions;
    pub_js_->publish(js);
  }

  void publish_status(const std::string& status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    pub_status_->publish(msg);
  }

  void publish_debug_marker(
    const Eigen::Vector3d& com,
    const Eigen::Vector3d& lf,
    const Eigen::Vector3d& rf,
    const FSMOutput& fsm_out)
  {
    visualization_msgs::msg::MarkerArray ma;
    const auto stamp   = now();
    const auto lifetime= rclcpp::Duration::from_seconds(1.0);
    int id = 0;

    // COM sphere
    auto add_sphere = [&](const Eigen::Vector3d& p,
                          float r, float g, float b, const std::string& ns)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = world_frame_;
      m.header.stamp    = stamp;
      m.ns = ns; m.id = id++;
      m.type   = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = p.x(); m.pose.position.y = p.y();
      m.pose.position.z = p.z(); m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.05;
      m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=0.9f;
      m.lifetime = lifetime;
      ma.markers.push_back(m);
    };

    add_sphere(com, 1.0f, 0.5f, 0.0f, "com");    // orange = COM
    add_sphere(lf,  0.1f, 0.9f, 0.3f, "lfoot");  // green  = left foot
    add_sphere(rf,  0.2f, 0.5f, 1.0f, "rfoot");  // blue   = right foot

    // State text
    visualization_msgs::msg::Marker txt;
    txt.header.frame_id = world_frame_;
    txt.header.stamp    = stamp;
    txt.ns   = "state"; txt.id = id++;
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.pose.position.x = com.x(); txt.pose.position.y = com.y();
    txt.pose.position.z = com.z() + 0.15;
    txt.pose.orientation.w = 1.0;
    txt.scale.z = 0.08;
    txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0f;
    txt.text = walk_state_name(fsm_out.state)
             + " k=" + std::to_string(k_);
    txt.lifetime = lifetime;
    ma.markers.push_back(txt);

    pub_markers_->publish(ma);
  }
};

}  // namespace g1_zmp_walking

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<g1_zmp_walking::G1WalkingController>();

  // Use MultiThreadedExecutor so the action client callbacks don't block
  // the 200 Hz control timer
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}