#pragma once

/**
 * pattern_generator.hpp
 * ─────────────────────
 * C++ implementation of the General ZMP Preview Controller from:
 *   Park & Youm, "General ZMP Preview Control for Bipedal Walking"
 *   IEEE ICRA 2007
 *
 * State vector  X  ∈ ℝ¹⁰ :  [r(2), ṙ(2), H(2), r̈(2), Ḣ(2)]
 *   r   = COM position (x,y)
 *   ṙ   = COM velocity
 *   H   = Horizontal Angular Momentum about COM (x,y)
 *   r̈   = COM acceleration
 *   Ḣ   = HAM rate
 *
 * Augmented state  X̃  ∈ ℝ¹² :  [p(2), ΔX(10)]
 *   p   = ZMP position (integral error state)
 *   ΔX  = incremental state (X(k) − X(k−1))
 *
 * Control input  U  ∈ ℝ⁴ :  [u(2), w(2)]
 *   u   = d(r̈)/dt  (COM jerk)
 *   w   = d(Ḣ)/dt  (HAM jerk)
 *
 * ZMP equation (exact, full dynamics):
 *   p = r − Tc² r̈ + (1/Mg) S Ḣ
 *   where S = [[0,-1],[1,0]],  Tc = sqrt(zc/g)
 */

#include <vector>
#include <stdexcept>
#include <cmath>

#include <Eigen/Dense>

#include "g1_zmp_walking/dare_solver.hpp"

namespace g1_zmp_walking {

// ─────────────────────────────────────────────────────────────────────────────
struct PatternGeneratorParams
{
  double dt        = 0.005;    ///< control timestep (s)
  double zc        = 0.75;     ///< nominal COM height (m)
  double M         = 35.0;     ///< total robot mass (kg)
  double g         = 9.81;     ///< gravity (m/s²)
  double Qe_scale  = 1.0;      ///< ZMP error weight
  double lambda_H  = 10.0;     ///< HAM penalty weight
  double R_scale   = 1e-6;     ///< control effort weight
  int    NL        = 200;      ///< preview horizon (samples)
};

// ─────────────────────────────────────────────────────────────────────────────
struct PatternState
{
  Eigen::Vector2d com_pos;   ///< COM position  (x, y)
  Eigen::Vector2d com_vel;   ///< COM velocity  (x, y)
  Eigen::Vector2d ham;       ///< Horizontal Angular Momentum
  Eigen::Vector2d com_acc;   ///< COM acceleration
  Eigen::Vector2d ham_dot;   ///< HAM rate
  Eigen::Vector2d zmp;       ///< actual ZMP position
};

// ─────────────────────────────────────────────────────────────────────────────
class GeneralZMPPreviewController
{
public:
  explicit GeneralZMPPreviewController(const PatternGeneratorParams& p = {})
    : p_(p)
  {
    build_system();
    compute_gains();
    reset();
  }

  // ── online step ────────────────────────────────────────────────────────────

  /**
   * @brief Advance the controller by one timestep.
   *
   * @param pd_preview  (NL × 2) matrix — desired ZMP for the next NL steps.
   *                    Row i = desired ZMP at time k+i.
   * @return PatternState with COM, HAM, and actual ZMP.
   */
  PatternState step(const Eigen::MatrixXd& pd_preview)
  {
    if (pd_preview.rows() < p_.NL || pd_preview.cols() != 2)
      throw std::invalid_argument("pd_preview must be (NL x 2)");

    // ── current ZMP ─────────────────────────────────────────────────────────
    Eigen::Vector2d p_current = C_ * X_;

    // ── integral error ───────────────────────────────────────────────────────
    Eigen::Vector2d p_desired_now = pd_preview.row(0).transpose();
    zmp_integral_ += (p_current - p_desired_now);

    // ── preview sum ──────────────────────────────────────────────────────────
    Eigen::Vector4d preview_sum = Eigen::Vector4d::Zero();
    for (int i = 0; i < p_.NL; ++i)
      preview_sum += G_[i] * pd_preview.row(i).transpose();

    // ── optimal control  u(k) = -Ks*e_sum - Kx*X - preview_sum ─────────────
    // Note: this is the ABSOLUTE control value, not an increment.
    // The integral action is embedded in zmp_integral_ (accumulated ZMP error).
    // Ref: Katayama 1985 eq(6), applied to full-dynamics ZMP eq.
    U_ = - Ks_ * zmp_integral_
         - Kx_ * X_
         - preview_sum;

    // ── state update ─────────────────────────────────────────────────────────
    X_ = A_ * X_ + B_ * U_;

    // ── pack output ──────────────────────────────────────────────────────────
    PatternState out;
    out.com_pos = X_.segment<2>(0);
    out.com_vel = X_.segment<2>(2);
    out.ham     = X_.segment<2>(4);
    out.com_acc = X_.segment<2>(6);
    out.ham_dot = X_.segment<2>(8);
    out.zmp     = C_ * X_;
    return out;
  }

  // ── reset ──────────────────────────────────────────────────────────────────
  void reset()
  {
    X_            = Eigen::VectorXd::Zero(10);
    U_            = Eigen::Vector4d::Zero();
    zmp_integral_ = Eigen::Vector2d::Zero();
  }

  // ── accessors ──────────────────────────────────────────────────────────────
  int                         NL()      const { return p_.NL; }
  const PatternGeneratorParams& params() const { return p_; }
  const Eigen::MatrixXd&      A()       const { return A_; }
  const Eigen::MatrixXd&      B()       const { return B_; }
  const Eigen::MatrixXd&      C()       const { return C_; }

private:
  PatternGeneratorParams p_;

  // System matrices
  Eigen::MatrixXd A_;   // 10×10
  Eigen::MatrixXd B_;   // 10×4
  Eigen::MatrixXd C_;   // 2×10

  // Augmented system
  Eigen::MatrixXd A_tilde_;   // 12×12
  Eigen::MatrixXd B_tilde_;   // 12×4

  // Gains
  Eigen::MatrixXd Ks_;   // 4×2   integral gain
  Eigen::MatrixXd Kx_;   // 4×10  state feedback gain

  // Preview gains  G_[0]…G_[NL-1]  each (4×2)
  std::vector<Eigen::MatrixXd> G_;

  // Runtime state
  Eigen::VectorXd  X_;             // 10×1
  Eigen::Vector4d  U_;             // 4×1
  Eigen::Vector2d  zmp_integral_;  // 2×1

  // ── build A, B, C ──────────────────────────────────────────────────────────
  void build_system()
  {
    const double dt  = p_.dt;
    const double Tc  = std::sqrt(p_.zc / p_.g);
    const double Mg  = p_.M * p_.g;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    Eigen::Matrix2d I2 = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d S;
    S << 0, -1, 1, 0;

    // ── A (10×10) ────────────────────────────────────────────────────────────
    A_ = Eigen::MatrixXd::Zero(10, 10);
    // row-block 0: r
    A_.block<2,2>(0,0) = I2;
    A_.block<2,2>(0,2) = dt * I2;
    A_.block<2,2>(0,6) = (dt2 / 2.0) * I2;
    // row-block 1: ṙ
    A_.block<2,2>(2,2) = I2;
    A_.block<2,2>(2,6) = dt * I2;
    // row-block 2: H
    A_.block<2,2>(4,4) = I2;
    A_.block<2,2>(4,8) = dt * I2;
    // row-block 3: r̈
    A_.block<2,2>(6,6) = I2;
    // row-block 4: Ḣ
    A_.block<2,2>(8,8) = I2;

    // ── B (10×4) ─────────────────────────────────────────────────────────────
    B_ = Eigen::MatrixXd::Zero(10, 4);
    B_.block<2,2>(0,0) = (dt3 / 6.0) * I2;
    B_.block<2,2>(2,0) = (dt2 / 2.0) * I2;
    B_.block<2,2>(4,2) = (dt2 / 2.0) * I2;
    B_.block<2,2>(6,0) = dt * I2;
    B_.block<2,2>(8,2) = dt * I2;

    // ── C (2×10) ─────────────────────────────────────────────────────────────
    C_ = Eigen::MatrixXd::Zero(2, 10);
    C_.block<2,2>(0,0) =  I2;
    C_.block<2,2>(0,6) = -Tc * Tc * I2;
    C_.block<2,2>(0,8) =  (1.0 / Mg) * S;
  }

  // ── solve DARE and precompute gains ────────────────────────────────────────
  void compute_gains()
  {
    Eigen::Matrix2d I2 = Eigen::Matrix2d::Identity();

    // ── augmented system X̃ = [p(2), ΔX(10)] ────────────────────────────────
    Eigen::MatrixXd CA = C_ * A_;
    Eigen::MatrixXd CB = C_ * B_;

    A_tilde_ = Eigen::MatrixXd::Zero(12, 12);
    A_tilde_.block<2,2>(0,0) = I2;
    A_tilde_.block<2,10>(0,2) = CA;
    A_tilde_.block<10,10>(2,2) = A_;

    B_tilde_ = Eigen::MatrixXd::Zero(12, 4);
    B_tilde_.block<2,4>(0,0)  = CB;
    B_tilde_.block<10,4>(2,0) = B_;

    // ── cost matrices ────────────────────────────────────────────────────────
    Eigen::MatrixXd Q_tilde = Eigen::MatrixXd::Zero(12, 12);
    // Qe — ZMP tracking weight (top-left 2×2)
    Q_tilde.block<2,2>(0,0) = p_.Qe_scale * I2;
    // Qx — HAM penalty (indices 6,7 in X̃ = 2+4, 2+5)
    Q_tilde(2 + 4, 2 + 4) = p_.lambda_H;
    Q_tilde(2 + 5, 2 + 5) = p_.lambda_H;

    Eigen::MatrixXd R_mat = p_.R_scale * Eigen::Matrix4d::Identity();

    // ── solve DARE ───────────────────────────────────────────────────────────
    Eigen::MatrixXd P = dare(A_tilde_, B_tilde_, Q_tilde, R_mat);

    // ── optimal gain  K = (R + BᵀPB)⁻¹ BᵀPA ────────────────────────────────
    Eigen::MatrixXd RBPB = R_mat + B_tilde_.transpose() * P * B_tilde_;
    Eigen::MatrixXd RBP  = RBPB.lu().solve(Eigen::MatrixXd::Identity(4, 4));
    Eigen::MatrixXd K_all = RBP * B_tilde_.transpose() * P * A_tilde_;  // 4×12

    Ks_ = K_all.leftCols(2);    // 4×2
    Kx_ = K_all.rightCols(10);  // 4×10

    // ── precompute preview gains G[0]…G[NL-1] ────────────────────────────────
    Eigen::MatrixXd Ac = A_tilde_ - B_tilde_ * K_all;   // 12×12
    Eigen::MatrixXd top(12, 2);
    top.topRows(2)    = I2;
    top.bottomRows(10) = Eigen::MatrixXd::Zero(10, 2);

    Eigen::MatrixXd Xe = -Ac.transpose() * P * top;   // 12×2

    G_.resize(p_.NL);
    for (int i = 0; i < p_.NL; ++i) {
      G_[i] = RBP * B_tilde_.transpose() * Xe;   // 4×2
      Xe    = Ac.transpose() * Xe;
    }
  }
};

}  // namespace g1_zmp_walking