#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace g1_zmp_walking {

/// 2-D point (x, y)
using Point2D = std::array<double, 2>;

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────

struct ZMPRefParams
{
  double dt        = 0.005;   ///< control timestep (s)
  double t_single  = 0.70;    ///< single-support duration (s)
  double t_double  = 0.10;    ///< double-support duration (s)
  int    n_preview = 200;     ///< padding samples beyond last step
  int    n_init    = 500;     ///< settling samples before first footstep
                              ///<  (preview controller needs ~500 at dt=0.005)
};

// ─────────────────────────────────────────────────────────────────────────────

enum class FootId : int { LEFT = 0, RIGHT = 1 };

struct FootstepInput
{
  FootId  foot;
  Point2D position;   ///< ankle projected to ground (x, y)
};

// ─────────────────────────────────────────────────────────────────────────────
// ZMPReferenceGenerator
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Converts a footstep sequence into a discrete ZMP reference pd[k].
 *
 * Output layout
 * ─────────────
 *   [ n_init  samples at midpoint      ]  ← controller settling phase
 *   [ n_steps × (n_single + n_double)  ]  ← walking phase
 *   [ n_preview samples at final ZMP   ]  ← look-ahead padding
 *
 * During single support : ZMP = support foot centre (constant).
 * During double support : ZMP linearly interpolates from current support
 *                         foot to the landing position of the swing foot.
 *
 * Why n_init matters
 * ──────────────────
 * The preview controller accumulates a ZMP integral error. Starting from
 * X=0 the COM requires ~500 steps (2.5 s at 200 Hz) to reach the first
 * desired ZMP position. Running the init phase at the initial midpoint
 * lets this transient die out before the first footstep begins.
 */
class ZMPReferenceGenerator
{
public:
  explicit ZMPReferenceGenerator(const ZMPRefParams& params = {})
    : p_(params)
  {
    if (p_.dt <= 0.0)
      throw std::invalid_argument("dt must be > 0");
    if (p_.t_single <= 0.0 || p_.t_double <= 0.0)
      throw std::invalid_argument("t_single and t_double must be > 0");

    n_single_ = static_cast<int>(std::round(p_.t_single / p_.dt));
    n_double_ = static_cast<int>(std::round(p_.t_double / p_.dt));
  }

  /**
   * @param footsteps   Ordered footstep list from FootstepPlanner.
   * @param init_left   Initial left  foot position (x, y).
   * @param init_right  Initial right foot position (x, y).
   * @return            pd[k] vector — one Point2D per timestep.
   */
  std::vector<Point2D> generate(
    const std::vector<FootstepInput>& footsteps,
    const Point2D& init_left,
    const Point2D& init_right) const
  {
    if (footsteps.empty())
      throw std::invalid_argument("footsteps must not be empty");

    std::vector<Point2D> pd;
    pd.reserve(p_.n_init +
               footsteps.size() * (n_single_ + n_double_) +
               p_.n_preview);

    // ── Phase 0: settling hold at feet midpoint ───────────────────────────────
    const Point2D init_zmp = midpoint(init_left, init_right);
    for (int i = 0; i < p_.n_init; ++i)
      pd.push_back(init_zmp);

    // ── Track foot positions so we know the support foot at each step ─────────
    Point2D pos_left  = init_left;
    Point2D pos_right = init_right;

    // ── Phase 1: walk ─────────────────────────────────────────────────────────
    for (const auto& step : footsteps) {
      // Support foot = opposite of swing foot
      const bool swing_left    = (step.foot == FootId::LEFT);
      const Point2D support_pos = swing_left ? pos_right : pos_left;
      const Point2D next_zmp    = step.position;  // landing position of swing foot

      // Single support: ZMP fixed at support foot
      for (int i = 0; i < n_single_; ++i)
        pd.push_back(support_pos);

      // Double support: linear interpolation to next support foot
      for (int j = 0; j < n_double_; ++j) {
        const double alpha = (n_double_ > 1)
          ? static_cast<double>(j) / static_cast<double>(n_double_ - 1)
          : 1.0;
        pd.push_back(lerp(support_pos, next_zmp, alpha));
      }

      // Update the foot that just landed
      if (swing_left) pos_left  = step.position;
      else            pos_right = step.position;
    }

    // ── Phase 2: preview padding at final position ────────────────────────────
    const Point2D final_zmp = pd.back();
    for (int i = 0; i < p_.n_preview; ++i)
      pd.push_back(final_zmp);

    return pd;
  }

  // ── accessors ──────────────────────────────────────────────────────────────
  int    n_single()  const { return n_single_; }
  int    n_double()  const { return n_double_; }
  int    n_init()    const { return p_.n_init; }
  int    n_preview() const { return p_.n_preview; }
  double dt()        const { return p_.dt; }
  const ZMPRefParams& params() const { return p_; }

private:
  ZMPRefParams p_;
  int          n_single_{};
  int          n_double_{};

  static Point2D midpoint(const Point2D& a, const Point2D& b)
  { return { (a[0]+b[0])*0.5, (a[1]+b[1])*0.5 }; }

  static Point2D lerp(const Point2D& a, const Point2D& b, double t)
  { return { a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]) }; }
};

}  // namespace g1_zmp_walking