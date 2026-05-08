#pragma once

/**
 * ik_solver.hpp
 * ─────────────
 * Analytical Inverse Kinematics for the Unitree G1 6-DOF leg.
 *
 * Joint order (both legs):
 *   0  hip_yaw      Z-axis rotation
 *   1  hip_roll     X-axis rotation
 *   2  hip_pitch    Y-axis rotation
 *   3  knee         Y-axis rotation  (always ≥ 0)
 *   4  ankle_pitch  Y-axis rotation
 *   5  ankle_roll   X-axis rotation
 *
 * Coordinate convention
 * ─────────────────────
 *   World frame : right-hand, z-up, x-forward, y-left
 *   Foot target : ankle joint projected to ground (z = ankle_height)
 *
 * Algorithm (sagittal-plane two-link IK)
 * ───────────────────────────────────────
 *  1. Remove body yaw → body frame.
 *  2. Remove hip_yaw  → hip-yaw frame.
 *  3. hip_roll  = atan2(dy, -dz)  [lateral lean].
 *  4. Remove hip_roll → sagittal plane (dx, dz only).
 *  5. Cosine-rule knee angle from hip–ankle distance L.
 *  6. hip_pitch from angle of (hip→ankle) vector + law-of-sines correction.
 *  7. ankle_pitch = −(hip_pitch + knee)  keeps sole parallel to ground.
 *  8. ankle_roll  = −hip_roll            cancels lateral tilt.
 *
 * Verified link lengths for G1 (measure from your URDF if different):
 *   l_thigh = 0.35 m   (hip joint → knee joint)
 *   l_shank = 0.31 m   (knee joint → ankle joint)
 *   com_to_hip_z = 0.08 m  (vertical COM → hip joint offset)
 *   ankle_height = 0.04 m  (ankle joint → ground contact)
 */

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

namespace g1_zmp_walking {

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────

struct IKParams
{
  double l_thigh      = 0.35;   ///< hip joint  → knee joint   (m)
  double l_shank      = 0.31;   ///< knee joint → ankle joint  (m)
  double hip_width    = 0.19;   ///< lateral distance between hip joints (m)
  double com_to_hip_z = 0.08;   ///< vertical offset COM → hip joint (m)
  double ankle_height = 0.04;   ///< ankle joint → ground contact plane (m)

  // Joint limits [rad]  (set from URDF <limit> tags)
  double hip_yaw_min   = -0.52;  double hip_yaw_max   =  0.52;
  double hip_roll_min  = -0.70;  double hip_roll_max  =  0.70;
  double hip_pitch_min = -1.75;  double hip_pitch_max =  0.60;
  double knee_min      =  0.00;  double knee_max      =  2.50;
  double ankle_pitch_min = -0.87; double ankle_pitch_max = 0.52;
  double ankle_roll_min  = -0.35; double ankle_roll_max  = 0.35;
};

// ─────────────────────────────────────────────────────────────────────────────
// Result
// ─────────────────────────────────────────────────────────────────────────────

struct LegJoints
{
  double hip_yaw     = 0.0;
  double hip_roll    = 0.0;
  double hip_pitch   = 0.0;
  double knee        = 0.0;
  double ankle_pitch = 0.0;
  double ankle_roll  = 0.0;

  bool   valid       = true;   ///< false if target is out of reach

  /// Return as Eigen vector [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
  Eigen::Vector<double, 6> to_vector() const
  {
    Eigen::Vector<double, 6> v;
    v << hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll;
    return v;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// G1LegIK
// ─────────────────────────────────────────────────────────────────────────────

class G1LegIK
{
public:
  explicit G1LegIK(const IKParams& params = {}) : p_(params) {}

  // ── main solve ─────────────────────────────────────────────────────────────

  /**
   * @brief Solve IK for one leg.
   *
   * @param com_pos   COM position in world frame [x, y, z].
   * @param com_yaw   Body heading (rad).
   * @param foot_pos  Desired foot (ankle) position in world frame [x, y, z].
   * @param foot_yaw  Desired foot heading (rad).
   * @param left      true = left leg, false = right leg.
   * @return LegJoints  (check .valid before using)
   */
  LegJoints solve(
    const Eigen::Vector3d& com_pos,
    double                 com_yaw,
    const Eigen::Vector3d& foot_pos,
    double                 foot_yaw,
    bool                   left) const
  {
    LegJoints q;
    const double sign = left ? +1.0 : -1.0;

    // ── Hip joint position in world frame ────────────────────────────────────
    // Hip is offset laterally from COM and slightly below it
    const Eigen::Matrix3d R_body = rot_z(com_yaw);
    const Eigen::Vector3d hip_offset{
      0.0,
      sign * p_.hip_width * 0.5,
      -p_.com_to_hip_z
    };
    const Eigen::Vector3d hip_pos = com_pos + R_body * hip_offset;

    // ── Vector hip → foot in body frame ─────────────────────────────────────
    const Eigen::Vector3d d_world = foot_pos - hip_pos;
    const Eigen::Vector3d d_body  = R_body.transpose() * d_world;

    // ── Step 1: hip_yaw ───────────────────────────────────────────────────────
    q.hip_yaw = wrap(foot_yaw - com_yaw);

    // ── Step 2: transform into hip-yaw frame ─────────────────────────────────
    const Eigen::Vector3d d_yaw = rot_z(q.hip_yaw).transpose() * d_body;

    // ── Step 3: hip_roll ──────────────────────────────────────────────────────
    // Project hip→foot onto the YZ plane; roll tilts the leg laterally.
    q.hip_roll = std::atan2(d_yaw.y(), -d_yaw.z());

    // ── Step 4: transform into sagittal plane after roll ─────────────────────
    const Eigen::Vector3d d_roll = rot_x(q.hip_roll).transpose() * d_yaw;
    // d_roll.y() ≈ 0 after this; only x (forward) and z (down) remain.

    // ── Step 5: distance hip → ankle ─────────────────────────────────────────
    const double L_max = p_.l_thigh + p_.l_shank - 1e-4;
    double L = d_roll.norm();
    if (L < 1e-4) L = 1e-4;
    if (L > L_max) {
      L = L_max;
      q.valid = false;   // out of reach, clamp and continue
    }

    // ── Step 6: knee angle via cosine rule ────────────────────────────────────
    // Interior angle at the knee joint (between thigh and shank)
    const double cos_k = (p_.l_thigh * p_.l_thigh
                        + p_.l_shank * p_.l_shank
                        - L * L)
                       / (2.0 * p_.l_thigh * p_.l_shank);
    // Interior angle between thigh and shank.
    // Unitree knee joint = π − interior (0 when straight, + when bent).
    const double knee_interior = std::acos(std::clamp(cos_k, -1.0, 1.0));
    q.knee = M_PI - knee_interior;

    // ── Step 7: hip_pitch ─────────────────────────────────────────────────────
    // Angle of (hip→ankle) from the −Z axis in the sagittal plane
    const double gamma = std::atan2(-d_roll.x(), -d_roll.z());

    // Law of sines: angle at hip in the triangle
    const double sin_alpha = p_.l_shank * std::sin(q.knee) / L;
    const double alpha = std::asin(std::clamp(sin_alpha, -1.0, 1.0));

    q.hip_pitch = gamma - alpha;

    // ── Step 8: ankle_pitch — keep sole parallel to ground ───────────────────
    q.ankle_pitch = -(q.hip_pitch + q.knee);

    // ── Step 9: ankle_roll — cancel hip roll ──────────────────────────────────
    q.ankle_roll = -q.hip_roll;

    // ── Clamp to joint limits ─────────────────────────────────────────────────
    q.hip_yaw     = std::clamp(q.hip_yaw,     p_.hip_yaw_min,     p_.hip_yaw_max);
    q.hip_roll    = std::clamp(q.hip_roll,    p_.hip_roll_min,    p_.hip_roll_max);
    q.hip_pitch   = std::clamp(q.hip_pitch,   p_.hip_pitch_min,   p_.hip_pitch_max);
    q.knee        = std::clamp(q.knee,        p_.knee_min,        p_.knee_max);
    q.ankle_pitch = std::clamp(q.ankle_pitch, p_.ankle_pitch_min, p_.ankle_pitch_max);
    q.ankle_roll  = std::clamp(q.ankle_roll,  p_.ankle_roll_min,  p_.ankle_roll_max);

    return q;
  }

  // ── forward kinematics (for validation) ───────────────────────────────────

  /**
   * @brief Compute ankle position from joint angles (for IK validation).
   *
   * @param hip_pos  Hip joint position in world frame.
   * @param q        Joint angles.
   * @param com_yaw  Body heading (to apply body rotation first).
   * @return Ankle position in world frame.
   */
  Eigen::Vector3d forward_kinematics(
    const Eigen::Vector3d& hip_pos,
    const LegJoints&       q,
    double                 com_yaw) const
  {
    Eigen::Matrix3d R = rot_z(com_yaw)
                      * rot_z(q.hip_yaw)
                      * rot_x(q.hip_roll)
                      * rot_y(q.hip_pitch);

    Eigen::Vector3d knee_pos = hip_pos + R * Eigen::Vector3d(0, 0, -p_.l_thigh);
    R = R * rot_y(q.knee);
    return knee_pos + R * Eigen::Vector3d(0, 0, -p_.l_shank);
  }

  const IKParams& params() const { return p_; }

private:
  IKParams p_;

  // ── rotation matrices ──────────────────────────────────────────────────────

  static Eigen::Matrix3d rot_z(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    Eigen::Matrix3d R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
  }

  static Eigen::Matrix3d rot_x(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    Eigen::Matrix3d R;
    R << 1, 0,  0,
         0, c, -s,
         0, s,  c;
    return R;
  }

  static Eigen::Matrix3d rot_y(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    Eigen::Matrix3d R;
    R <<  c, 0, s,
          0, 1, 0,
         -s, 0, c;
    return R;
  }

  static double wrap(double a)
  {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }
};

}  // namespace g1_zmp_walking