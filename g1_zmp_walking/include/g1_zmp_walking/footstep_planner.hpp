#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace g1_zmp_walking {

// ─────────────────────────────────────────────────────────────────────────────
// Types
// ─────────────────────────────────────────────────────────────────────────────

enum class Foot : int { LEFT = 0, RIGHT = 1 };

inline std::string foot_name(Foot f)
{
  return (f == Foot::LEFT) ? "LEFT" : "RIGHT";
}

/// Foot position in world frame (x, y, z).
using Vec3 = std::array<double, 3>;

/// 2-D rotation matrix row-major [cos -sin / sin cos].
using Mat2 = std::array<double, 4>;

struct Footstep
{
  Foot   foot;
  Vec3   position;   ///< ankle projected to ground, world frame
  double yaw;        ///< heading of the foot (rad)
  double t_start;    ///< phase start time (sec)
  double t_end;      ///< phase end   time (sec)
};

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────

struct FootstepPlannerParams
{
  double hip_width        = 0.19;   ///< lateral ankle–ankle distance (m)
  double max_step_length  = 0.20;   ///< max |dx| per step (m)
  double max_step_width   = 0.12;   ///< max extra lateral per step (m)
  double max_step_yaw     = 0.30;   ///< max |dyaw| per step (rad) ~17 deg
  double t_single         = 0.70;   ///< single-support duration (s)
  double t_double         = 0.10;   ///< double-support duration (s)
};

// ─────────────────────────────────────────────────────────────────────────────
// FootstepPlanner
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Kinematic footstep planner for the Unitree G1.
 *
 * Algorithm
 * ─────────
 * Given a velocity command (vx, vy, wz) and the current foot positions
 * the planner alternates swing feet and places each new footstep at:
 *
 *   p_new = p_support + R(yaw) * [dx,  dy ± hip_width]
 *
 * where dx, dy are per-step displacements clamped to kinematic limits.
 *
 * Coordinate convention
 * ─────────────────────
 *   World frame: right-hand, z-up, x-forward, y-left
 */
class FootstepPlanner
{
public:
  explicit FootstepPlanner(const FootstepPlannerParams& params = {})
    : p_(params)
    , t_step_(params.t_single + params.t_double)
  {}

  /**
   * @brief Generate n_steps footsteps from a velocity command.
   *
   * @param vx, vy    COM velocity in the CURRENT heading frame (m/s)
   * @param wz        Yaw rate (rad/s)
   * @param n_steps   Number of footsteps to generate
   * @param start_left   Initial left  foot position [x,y,z]  (nullptr → nominal)
   * @param start_right  Initial right foot position [x,y,z]  (nullptr → nominal)
   * @param start_yaw    Initial heading (rad)
   * @param first_swing  Which foot moves first
   * @return Vector of Footstep
   */
  std::vector<Footstep> generate(
    double       vx,
    double       vy,
    double       wz,
    int          n_steps,
    const Vec3*  start_left   = nullptr,
    const Vec3*  start_right  = nullptr,
    double       start_yaw    = 0.0,
    Foot         first_swing  = Foot::RIGHT) const
  {
    if (n_steps <= 0)
      throw std::invalid_argument("n_steps must be > 0");

    // ── clamp velocity to kinematic limits ───────────────────────────────
    const double dx   = clamp(vx * t_step_, -p_.max_step_length, p_.max_step_length);
    const double dy   = clamp(vy * t_step_, -p_.max_step_width,  p_.max_step_width);
    const double dyaw = clamp(wz * t_step_, -p_.max_step_yaw,    p_.max_step_yaw);

    // ── initial foot positions ───────────────────────────────────────────
    const double hw = p_.hip_width * 0.5;
    Vec3 pos_left  = start_left  ? *start_left  : Vec3{0.0,  hw, 0.0};
    Vec3 pos_right = start_right ? *start_right : Vec3{0.0, -hw, 0.0};

    double yaw        = start_yaw;
    double t          = 0.0;
    Foot   swing_foot = first_swing;

    std::vector<Footstep> steps;
    steps.reserve(n_steps);

    for (int i = 0; i < n_steps; ++i) {
      Foot support_foot = (swing_foot == Foot::RIGHT) ? Foot::LEFT : Foot::RIGHT;
      Vec3& p_support   = (support_foot == Foot::LEFT) ? pos_left : pos_right;

      // lateral nominal offset (always step to the outside)
      const double sign    = (swing_foot == Foot::LEFT) ? +1.0 : -1.0;
      const double lateral = sign * p_.hip_width;

      // rotate displacement into world frame
      Mat2 R = rot2d(yaw);
      double world_x = R[0] * dx + R[1] * (dy + lateral);
      double world_y = R[2] * dx + R[3] * (dy + lateral);

      Vec3 new_pos{
        p_support[0] + world_x,
        p_support[1] + world_y,
        0.0
      };

      yaw = wrap_angle(yaw + dyaw);

      steps.push_back(Footstep{
        swing_foot,
        new_pos,
        yaw,
        t,
        t + t_step_
      });

      // update swing foot position and advance time
      if (swing_foot == Foot::LEFT)  pos_left  = new_pos;
      else                           pos_right = new_pos;

      t         += t_step_;
      swing_foot = support_foot;   // alternate
    }

    return steps;
  }

  /// Default nominal stance (standing still, no offsets).
  std::pair<Vec3, Vec3> nominal_stance() const
  {
    const double hw = p_.hip_width * 0.5;
    return { Vec3{0.0,  hw, 0.0},
             Vec3{0.0, -hw, 0.0} };
  }

  double step_duration()  const { return t_step_; }
  double single_support() const { return p_.t_single; }
  double double_support() const { return p_.t_double; }

  const FootstepPlannerParams& params() const { return p_; }

private:
  FootstepPlannerParams p_;
  double                t_step_;

  // ── helpers ──────────────────────────────────────────────────────────────

  static double clamp(double v, double lo, double hi)
  {
    return (v < lo) ? lo : (v > hi) ? hi : v;
  }

  /// Wrap angle to (-π, π].
  static double wrap_angle(double a)
  {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  /// 2-D rotation matrix as flat array [c -s / s c].
  static Mat2 rot2d(double yaw)
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Mat2{c, -s, s, c};
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Utility: pure-yaw → quaternion  (x, y, z, w)
// ─────────────────────────────────────────────────────────────────────────────

struct Quaternion { double x, y, z, w; };

inline Quaternion yaw_to_quaternion(double yaw)
{
  return { 0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5) };
}

}  // namespace g1_zmp_walking