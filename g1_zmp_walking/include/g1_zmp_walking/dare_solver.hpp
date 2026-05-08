#pragma once

/**
 * dare_solver.hpp
 * ───────────────
 * Iterative Discrete Algebraic Riccati Equation solver using
 * Schur decomposition via the doubling algorithm.
 *
 * Solves:  P = AᵀPA − AᵀPB(R + BᵀPB)⁻¹BᵀPA + Q
 *
 * Reference: Laub (1979), "A Schur method for solving algebraic Riccati equations"
 * Implementation: structure-preserving doubling algorithm (SDA)
 */

#include <Eigen/Dense>
#include <stdexcept>

namespace g1_zmp_walking {

/**
 * @brief Solve the Discrete Algebraic Riccati Equation using
 *        the structure-preserving doubling algorithm.
 *
 * @param A  (n×n) state matrix
 * @param B  (n×m) input matrix
 * @param Q  (n×n) state cost (symmetric, PSD)
 * @param R  (m×m) input cost (symmetric, PD)
 * @param max_iter  max iterations (default 200)
 * @param tol       convergence tolerance (default 1e-10)
 * @return P  (n×n) solution matrix (symmetric, PSD)
 */
inline Eigen::MatrixXd dare(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    int    max_iter = 200,
    double tol      = 1e-10)
{
  const int n = A.rows();
  const int m = B.cols();

  if (A.rows() != A.cols() || A.rows() != n)
    throw std::invalid_argument("DARE: A must be square");
  if (B.rows() != n)
    throw std::invalid_argument("DARE: B row count must match A");
  if (Q.rows() != n || Q.cols() != n)
    throw std::invalid_argument("DARE: Q must be n×n");
  if (R.rows() != m || R.cols() != m)
    throw std::invalid_argument("DARE: R must be m×m");

  // Initialise doubling
  Eigen::MatrixXd Ak = A;
  Eigen::MatrixXd Gk = B * R.llt().solve(B.transpose());   // B R⁻¹ Bᵀ
  Eigen::MatrixXd Pk = Q;

  for (int i = 0; i < max_iter; ++i) {
    const Eigen::MatrixXd I   = Eigen::MatrixXd::Identity(n, n);
    const Eigen::MatrixXd tmp = (I + Gk * Pk).lu().solve(I);   // (I + G P)⁻¹

    const Eigen::MatrixXd Ak_new = Ak * tmp * Ak;
    const Eigen::MatrixXd Gk_new = Gk + Ak * tmp * Gk * Ak.transpose();
    const Eigen::MatrixXd Pk_new = Pk + Ak.transpose() * Pk * tmp * Ak;

    // Convergence check
    const double err = (Pk_new - Pk).norm() / (1.0 + Pk_new.norm());

    Ak = Ak_new;
    Gk = Gk_new;
    Pk = Pk_new;

    if (err < tol) break;
  }

  return Pk;
}

}  // namespace g1_zmp_walking