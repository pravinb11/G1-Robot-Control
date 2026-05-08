#pragma once

/**
 * walking_fsm.hpp
 * ───────────────
 * Finite State Machine for bipedal walking pattern execution.
 *
 * State diagram
 * ─────────────
 *
 *   ┌──────┐  start()   ┌─────────────┐  timer≥t_double  ┌────────────────┐
 *   │ IDLE │──────────→ │ INIT_DOUBLE │─────────────────→ │ SINGLE_SUPPORT │
 *   └──────┘            └─────────────┘                   └────────────────┘
 *                                                                  │  ↑
 *                                                  timer≥t_single  │  │ step_idx++,
 *                                                                  ↓  │ timer≥t_double
 *                              ┌─────────────┐              ┌──────────────┐
 *                              │ STOP_DOUBLE │ ←──────────  │DOUBLE_SUPPORT│
 *                              └─────────────┘  last step   └──────────────┘
 *
 * Outputs per update()
 * ────────────────────
 *   Always  : state, step_idx, elapsed_time
 *   In SS   : swing_foot (LEFT/RIGHT), swing_phase [0→1], current step info
 *   In DS   : ds_phase [0→1]
 *
 * Thread safety
 * ─────────────
 *   Not thread-safe. Call update() from a single control thread.
 */

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace g1_zmp_walking {

// ─────────────────────────────────────────────────────────────────────────────
// Types
// ─────────────────────────────────────────────────────────────────────────────

enum class WalkState : int {
  IDLE           = 0,
  INIT_DOUBLE    = 1,   ///< initial weight shift (controller settling)
  SINGLE_SUPPORT = 2,   ///< one foot swinging
  DOUBLE_SUPPORT = 3,   ///< both feet on ground (weight transfer)
  STOP_DOUBLE    = 4,   ///< final double support after last step
};

inline std::string walk_state_name(WalkState s)
{
  switch (s) {
    case WalkState::IDLE:           return "IDLE";
    case WalkState::INIT_DOUBLE:    return "INIT_DOUBLE";
    case WalkState::SINGLE_SUPPORT: return "SINGLE_SUPPORT";
    case WalkState::DOUBLE_SUPPORT: return "DOUBLE_SUPPORT";
    case WalkState::STOP_DOUBLE:    return "STOP_DOUBLE";
    default:                        return "UNKNOWN";
  }
}

enum class SwingFoot : int { LEFT = 0, RIGHT = 1 };

/// One footstep entry passed to the FSM
struct FSMFootstep
{
  SwingFoot       foot;        ///< which foot is swinging
  Eigen::Vector3d position;    ///< landing position (world frame)
  double          yaw;         ///< landing heading (rad)
};

/// Output of FSM::update() — snapshot of current walking state
struct FSMOutput
{
  WalkState   state      = WalkState::IDLE;
  int         step_idx   = 0;
  double      elapsed    = 0.0;   ///< time since current state entered (s)

  // Only valid in SINGLE_SUPPORT
  SwingFoot   swing_foot  = SwingFoot::RIGHT;
  double      swing_phase = 0.0;   ///< 0 (lift-off) → 1 (touch-down)
  FSMFootstep current_step{};

  // Only valid in DOUBLE_SUPPORT
  double      ds_phase = 0.0;     ///< 0 (SS end) → 1 (next SS start)

  bool        is_single_support() const { return state == WalkState::SINGLE_SUPPORT; }
  bool        is_double_support() const {
    return state == WalkState::DOUBLE_SUPPORT || state == WalkState::INIT_DOUBLE; }
  bool        is_done()           const { return state == WalkState::STOP_DOUBLE; }
  bool        is_idle()           const { return state == WalkState::IDLE; }
};

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────

struct FSMParams
{
  double t_single = 0.70;   ///< single-support phase duration (s)
  double t_double = 0.10;   ///< double-support phase duration (s)
  double dt       = 0.005;  ///< control timestep (s)
};

// ─────────────────────────────────────────────────────────────────────────────
// WalkingFSM
// ─────────────────────────────────────────────────────────────────────────────

class WalkingFSM
{
public:
  explicit WalkingFSM(const FSMParams& params = {})
    : p_(params)
  {
    if (p_.dt <= 0.0)
      throw std::invalid_argument("FSM: dt must be > 0");
    if (p_.t_single <= 0.0 || p_.t_double <= 0.0)
      throw std::invalid_argument("FSM: t_single and t_double must be > 0");
  }

  // ── control ───────────────────────────────────────────────────────────────

  /// Load a new footstep plan and transition to INIT_DOUBLE.
  void start(const std::vector<FSMFootstep>& footsteps)
  {
    if (footsteps.empty())
      throw std::invalid_argument("FSM: footstep list must not be empty");
    footsteps_ = footsteps;
    step_idx_  = 0;
    timer_     = 0.0;
    state_     = WalkState::INIT_DOUBLE;
    on_state_enter(WalkState::INIT_DOUBLE);
  }

  /// Interrupt walking and return to IDLE immediately.
  void stop()
  {
    state_    = WalkState::IDLE;
    timer_    = 0.0;
    step_idx_ = 0;
    footsteps_.clear();
  }

  // ── main update ───────────────────────────────────────────────────────────

  /**
   * @brief Advance FSM by one control timestep.
   *        Must be called at exactly 1/dt Hz.
   * @return FSMOutput snapshot of current state.
   */
  FSMOutput update()
  {
    timer_ += p_.dt;

    switch (state_) {

      case WalkState::INIT_DOUBLE:
        if (timer_ >= p_.t_double) {
          transition_to(WalkState::SINGLE_SUPPORT);
        }
        break;

      case WalkState::SINGLE_SUPPORT:
        if (timer_ >= p_.t_single) {
          transition_to(WalkState::DOUBLE_SUPPORT);
        }
        break;

      case WalkState::DOUBLE_SUPPORT:
        if (timer_ >= p_.t_double) {
          step_idx_++;
          if (step_idx_ >= static_cast<int>(footsteps_.size())) {
            transition_to(WalkState::STOP_DOUBLE);
          } else {
            transition_to(WalkState::SINGLE_SUPPORT);
          }
        }
        break;

      case WalkState::STOP_DOUBLE:
      case WalkState::IDLE:
      default:
        break;
    }

    return build_output();
  }

  // ── optional callbacks ────────────────────────────────────────────────────

  /// Called once when entering SINGLE_SUPPORT (lift-off event).
  std::function<void(const FSMFootstep&)> on_lift_off;

  /// Called once when entering DOUBLE_SUPPORT (touch-down event).
  std::function<void(const FSMFootstep&)> on_touch_down;

  /// Called once when walk is complete.
  std::function<void()> on_walk_complete;

  // ── accessors ─────────────────────────────────────────────────────────────

  WalkState   state()     const { return state_; }
  int         step_idx()  const { return step_idx_; }
  int         n_steps()   const { return static_cast<int>(footsteps_.size()); }
  double      timer()     const { return timer_; }
  bool        is_done()   const { return state_ == WalkState::STOP_DOUBLE; }
  bool        is_active() const {
    return state_ != WalkState::IDLE && state_ != WalkState::STOP_DOUBLE; }
  const FSMParams& params() const { return p_; }

private:
  FSMParams                 p_;
  WalkState                 state_    = WalkState::IDLE;
  double                    timer_    = 0.0;
  int                       step_idx_ = 0;
  std::vector<FSMFootstep>  footsteps_;

  // ── helpers ───────────────────────────────────────────────────────────────

  void transition_to(WalkState next)
  {
    // Fire callbacks before updating state
    if (next == WalkState::SINGLE_SUPPORT && on_lift_off
        && step_idx_ < static_cast<int>(footsteps_.size()))
      on_lift_off(footsteps_[step_idx_]);

    if (next == WalkState::DOUBLE_SUPPORT && on_touch_down
        && step_idx_ < static_cast<int>(footsteps_.size()))
      on_touch_down(footsteps_[step_idx_]);

    if (next == WalkState::STOP_DOUBLE && on_walk_complete)
      on_walk_complete();

    state_ = next;
    timer_ = 0.0;
    on_state_enter(next);
  }

  void on_state_enter(WalkState /*s*/) { /* hook for subclasses */ }

  FSMOutput build_output() const
  {
    FSMOutput out;
    out.state    = state_;
    out.step_idx = step_idx_;
    out.elapsed  = timer_;

    if (state_ == WalkState::SINGLE_SUPPORT
        && step_idx_ < static_cast<int>(footsteps_.size())) {
      const auto& step  = footsteps_[step_idx_];
      out.swing_foot    = step.foot;
      out.swing_phase   = std::min(timer_ / p_.t_single, 1.0);
      out.current_step  = step;
    }

    if (state_ == WalkState::DOUBLE_SUPPORT || state_ == WalkState::INIT_DOUBLE) {
      out.ds_phase = std::min(timer_ / p_.t_double, 1.0);
    }

    return out;
  }
};

}  // namespace g1_zmp_walking