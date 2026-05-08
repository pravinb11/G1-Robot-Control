#pragma once

/**
 * swing_foot_trajectory.hpp
 * ─────────────────────────
 * Generates the 3-D swing foot trajectory between lift-off and touch-down.
 *
 * Two vertical profiles are provided:
 *
 *  PARABOLIC  z(t) = h · 4t(1−t)
 *    • Peak at t = 0.5
 *    • Non-zero velocity at endpoints (velocity discontinuity at lift-off
 *      and touch-down) — matches the Python reference implementation
 *
 *  CUBIC      z(t) = h · 16t²(1−t)²
 *    • Peak at t = 0.5  (same peak height as parabolic)
 *    • Zero vertical velocity at t = 0 and t = 1  (C1 continuous)
 *    • Recommended for real hardware — reduces impact at touch-down
 *
 * Horizontal motion is always linear interpolation in XY.
 *
 * Usage
 * ─────
 *   SwingFootTrajectory swing(0.06);          // 6 cm step height
 *   auto pos = swing.get_pos(p_start, p_end, phase);   // phase ∈ [0, 1]
 *
 * Coordinate convention
 * ─────────────────────
 *   All positions in world frame, z-up.
 *   p_start / p_end: ankle position at ground level (z = ankle_height).
 *   Returned position: ankle position at current swing phase.
 */

#include <array>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

namespace g1_zmp_walking {

// ─────────────────────────────────────────────────────────────────────────────

enum class SwingProfile {
  PARABOLIC,   ///< z = h·4t(1-t)            — matches Python reference
  CUBIC,       ///< z = h·16t²(1-t)²         — C1 continuous (recommended)
};

struct SwingFootParams
{
  double       step_height = 0.06;                    ///< apex height (m)
  SwingProfile profile     = SwingProfile::CUBIC;     ///< vertical profile
  double       lift_fraction   = 0.1;  ///< fraction of phase spent lifting
  double       land_fraction   = 0.1;  ///< fraction of phase spent landing
};

// ─────────────────────────────────────────────────────────────────────────────

struct SwingState
{
  Eigen::Vector3d position;   ///< foot position (world frame)
  Eigen::Vector3d velocity;   ///< foot velocity  (m/s per unit phase)
};

// ─────────────────────────────────────────────────────────────────────────────

class SwingFootTrajectory
{
public:
  explicit SwingFootTrajectory(const SwingFootParams& params = {})
    : p_(params)
  {
    if (p_.step_height < 0.0)
      throw std::invalid_argument("step_height must be >= 0");
    if (p_.lift_fraction + p_.land_fraction >= 1.0)
      throw std::invalid_argument("lift + land fractions must be < 1");
  }

  explicit SwingFootTrajectory(double step_height,
                               SwingProfile profile = SwingProfile::CUBIC)
    : SwingFootTrajectory(SwingFootParams{step_height, profile})
  {}

  // ── main API ───────────────────────────────────────────────────────────────

  /**
   * @brief Compute swing foot position at a given phase.
   *
   * @param p_start  Lift-off position  [x, y, z_ground]
   * @param p_end    Touch-down position [x, y, z_ground]
   * @param phase    Normalised swing phase: 0 = lift-off, 1 = touch-down
   * @return Foot position [x, y, z] in world frame
   */
  Eigen::Vector3d get_pos(
    const Eigen::Vector3d& p_start,
    const Eigen::Vector3d& p_end,
    double phase) const
  {
    phase = std::clamp(phase, 0.0, 1.0);

    // ── horizontal: linear interpolation ─────────────────────────────────────
    const double t = phase;
    const Eigen::Vector3d horizontal =
        (1.0 - t) * p_start + t * p_end;

    // ── vertical: profile above p_end.z (ground level) ───────────────────────
    const double z_arc = vertical_arc(t);

    return Eigen::Vector3d(
      horizontal.x(),
      horizontal.y(),
      p_end.z() + z_arc);
  }

  /**
   * @brief Compute swing foot position AND velocity at a given phase.
   *        Velocity is d(pos)/d(phase) — multiply by (1/T_swing) for m/s.
   *
   * @param p_start  Lift-off position
   * @param p_end    Touch-down position
   * @param phase    Swing phase ∈ [0, 1]
   * @return SwingState with position and velocity
   */
  SwingState get_state(
    const Eigen::Vector3d& p_start,
    const Eigen::Vector3d& p_end,
    double phase) const
  {
    phase = std::clamp(phase, 0.0, 1.0);

    const double t   = phase;
    const double eps = 1e-7;

    SwingState s;
    s.position = get_pos(p_start, p_end, t);

    // Numerical velocity (d pos / d phase)
    const double t_fwd = std::min(t + eps, 1.0);
    const double t_bwd = std::max(t - eps, 0.0);
    s.velocity = (get_pos(p_start, p_end, t_fwd) -
                  get_pos(p_start, p_end, t_bwd))
                 / (t_fwd - t_bwd);

    return s;
  }

  // ── accessors ──────────────────────────────────────────────────────────────
  double             step_height() const { return p_.step_height; }
  SwingProfile       profile()     const { return p_.profile; }
  const SwingFootParams& params()  const { return p_; }

private:
  SwingFootParams p_;

  // ── vertical arc height above ground at normalised phase t ─────────────────
  double vertical_arc(double t) const
  {
    switch (p_.profile) {
      case SwingProfile::PARABOLIC:
        // z(t) = h · 4t(1-t)  — peaks at t=0.5, non-zero vel at endpoints
        return p_.step_height * 4.0 * t * (1.0 - t);

      case SwingProfile::CUBIC:
      default:
        // z(t) = h · 16t²(1-t)²  — peaks at t=0.5, zero vel at endpoints
        return p_.step_height * 16.0 * t * t * (1.0 - t) * (1.0 - t);
    }
  }
};

}  // namespace g1_zmp_walking