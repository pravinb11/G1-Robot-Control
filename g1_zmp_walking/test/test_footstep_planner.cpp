#include <gtest/gtest.h>

#include "g1_zmp_walking/footstep_planner.hpp"

using namespace g1_zmp_walking;

// ── Helpers ──────────────────────────────────────────────────────────────────


// ─────────────────────────────────────────────────────────────────────────────

TEST(FootstepPlanner, CorrectNumberOfSteps)
{
  FootstepPlanner p;
  auto steps = p.generate(0.1, 0.0, 0.0, 8);
  EXPECT_EQ(static_cast<int>(steps.size()), 8);
}

TEST(FootstepPlanner, FeetAlternate)
{
  FootstepPlanner p;
  auto steps = p.generate(0.1, 0.0, 0.0, 10);
  for (std::size_t i = 1; i < steps.size(); ++i)
    EXPECT_NE(steps[i].foot, steps[i-1].foot)
      << "Steps " << i-1 << " and " << i << " have same foot.";
}

TEST(FootstepPlanner, StepLengthClamped)
{
  FootstepPlannerParams cfg;
  cfg.max_step_length = 0.20;
  FootstepPlanner p(cfg);

  // Very high velocity — must be clamped
  auto steps = p.generate(5.0, 0.0, 0.0, 6);
  for (std::size_t i = 1; i < steps.size(); ++i) {
    // longitudinal distance between consecutive same-side steps ≤ 2 * max_step
    if (steps[i].foot == steps[i-1].foot) continue;
    double d = std::abs(steps[i].position[0] - steps[i-1].position[0]);
    // Each step advances at most max_step_length from the SUPPORT foot
    // so forward distance per step ≤ max_step_length + hip_width tolerance
    EXPECT_LE(d, cfg.max_step_length + 0.21)  // 0.21 covers lateral geometry
      << "Step " << i << " forward displacement too large: " << d;
  }
}

TEST(FootstepPlanner, ZeroVelocityStaysNearNominal)
{
  FootstepPlannerParams cfg;
  cfg.hip_width = 0.19;
  FootstepPlanner p(cfg);

  auto steps = p.generate(0.0, 0.0, 0.0, 6);

  for (const auto& s : steps) {
    // y should stay near ±hw
    EXPECT_NEAR(std::abs(s.position[1]), cfg.hip_width / 2.0, 0.001)
      << "Foot " << foot_name(s.foot) << " lateral pos wrong.";
    // x should stay near 0
    EXPECT_NEAR(s.position[0], 0.0, 0.001);
  }
}

TEST(FootstepPlanner, TimestampsMonotonicallyIncreasing)
{
  FootstepPlanner p;
  auto steps = p.generate(0.1, 0.0, 0.0, 6);
  for (std::size_t i = 1; i < steps.size(); ++i) {
    EXPECT_GT(steps[i].t_start, steps[i-1].t_start);
    EXPECT_GT(steps[i].t_end,   steps[i].t_start);
  }
}

TEST(FootstepPlanner, TurnInPlaceYawAccumulates)
{
  FootstepPlanner p;
  const double wz = 0.3;
  auto steps = p.generate(0.0, 0.0, wz, 4);

  // Each step should have a larger yaw than the previous
  for (std::size_t i = 1; i < steps.size(); ++i)
    EXPECT_GT(steps[i].yaw, steps[i-1].yaw);
}

TEST(FootstepPlanner, YawToQuaternionUnitLength)
{
  for (double yaw : {0.0, M_PI / 4, M_PI / 2, M_PI, -M_PI / 3}) {
    auto q = yaw_to_quaternion(yaw);
    double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    EXPECT_NEAR(norm, 1.0, 1e-12) << "Quaternion not unit for yaw=" << yaw;
  }
}

TEST(FootstepPlanner, InvalidNStepsThrows)
{
  FootstepPlanner p;
  EXPECT_THROW(p.generate(0.1, 0.0, 0.0, 0),  std::invalid_argument);
  EXPECT_THROW(p.generate(0.1, 0.0, 0.0, -1), std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}