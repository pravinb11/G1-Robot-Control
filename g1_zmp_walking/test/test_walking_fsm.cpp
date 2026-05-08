#include <gtest/gtest.h>
#include <cmath>
#include "g1_zmp_walking/walking_fsm.hpp"

using namespace g1_zmp_walking;

// ── helpers ──────────────────────────────────────────────────────────────────

static std::vector<FSMFootstep> make_steps(int n)
{
  std::vector<FSMFootstep> steps;
  for (int i = 0; i < n; ++i) {
    FSMFootstep fs;
    fs.foot     = (i % 2 == 0) ? SwingFoot::RIGHT : SwingFoot::LEFT;
    fs.position = Eigen::Vector3d(0.12 * (i + 1),
                                  fs.foot == SwingFoot::LEFT ? 0.095 : -0.095,
                                  0.04);
    fs.yaw = 0.0;
    steps.push_back(fs);
  }
  return steps;
}

static FSMParams default_params()
{
  FSMParams p; p.t_single=0.7; p.t_double=0.1; p.dt=0.005; return p;
}

// ── tests ─────────────────────────────────────────────────────────────────────

TEST(WalkingFSM, InitialStateIsIdle)
{
  WalkingFSM fsm(default_params());
  EXPECT_EQ(fsm.state(), WalkState::IDLE);
}

TEST(WalkingFSM, StartTransitionsToInitDouble)
{
  WalkingFSM fsm(default_params());
  fsm.start(make_steps(4));
  EXPECT_EQ(fsm.state(), WalkState::INIT_DOUBLE);
  EXPECT_EQ(fsm.step_idx(), 0);
}

TEST(WalkingFSM, InitDoubleTransitionsAfterTDouble)
{
  auto p = default_params();
  WalkingFSM fsm(p);
  fsm.start(make_steps(4));

  const int n_init = static_cast<int>(std::round(p.t_double / p.dt));
  for (int i = 0; i < n_init; ++i) {
    auto out = fsm.update();
    EXPECT_EQ(out.state, WalkState::INIT_DOUBLE) << "at step " << i;
  }
  auto out = fsm.update();
  EXPECT_EQ(out.state, WalkState::SINGLE_SUPPORT);
}

TEST(WalkingFSM, SingleSupportDuration)
{
  auto p = default_params();
  WalkingFSM fsm(p);
  fsm.start(make_steps(4));

  // Skip INIT_DOUBLE
  const int n_d = static_cast<int>(std::round(p.t_double / p.dt)) + 1;
  for (int i = 0; i < n_d; ++i) fsm.update();
  ASSERT_EQ(fsm.state(), WalkState::SINGLE_SUPPORT);

  // Count SS ticks
  const int n_s = static_cast<int>(std::round(p.t_single / p.dt));
  for (int i = 0; i < n_s; ++i) {
    auto out = fsm.update();
    EXPECT_EQ(out.state, WalkState::SINGLE_SUPPORT) << "at step " << i;
  }
  auto out = fsm.update();
  EXPECT_EQ(out.state, WalkState::DOUBLE_SUPPORT);
}

TEST(WalkingFSM, SwingPhaseMonotonicallyIncreases)
{
  auto p = default_params();
  WalkingFSM fsm(p);
  fsm.start(make_steps(2));

  // Advance to SINGLE_SUPPORT
  while (fsm.state() != WalkState::SINGLE_SUPPORT)
    fsm.update();

  double prev_phase = -1.0;
  while (fsm.state() == WalkState::SINGLE_SUPPORT) {
    auto out = fsm.update();
    // Only check phase while still in SS (last update may transition out)
    if (out.state == WalkState::SINGLE_SUPPORT) {
      EXPECT_GE(out.swing_phase, prev_phase);
      prev_phase = out.swing_phase;
    }
  }
  EXPECT_NEAR(prev_phase, 1.0, 0.02);
}

TEST(WalkingFSM, StepIndexAdvancesCorrectly)
{
  auto p = default_params();
  WalkingFSM fsm(p);
  const int N = 4;
  fsm.start(make_steps(N));

  // Run full walk
  while (!fsm.is_done() && fsm.timer() < 100.0)
    fsm.update();

  EXPECT_EQ(fsm.state(), WalkState::STOP_DOUBLE);
  EXPECT_EQ(fsm.step_idx(), N);
}

TEST(WalkingFSM, CallbacksFire)
{
  WalkingFSM fsm(default_params());
  fsm.start(make_steps(2));

  int lift_count = 0, touch_count = 0;
  bool complete = false;
  fsm.on_lift_off      = [&](const FSMFootstep&) { ++lift_count; };
  fsm.on_touch_down    = [&](const FSMFootstep&) { ++touch_count; };
  fsm.on_walk_complete = [&]() { complete = true; };

  while (!fsm.is_done() && fsm.timer() < 100.0)
    fsm.update();

  EXPECT_EQ(lift_count,  2);    // one per step
  EXPECT_EQ(touch_count, 2);    // one per step
  EXPECT_TRUE(complete);
}

TEST(WalkingFSM, StopResetsToIdle)
{
  WalkingFSM fsm(default_params());
  fsm.start(make_steps(4));
  fsm.update(); fsm.update();
  fsm.stop();
  EXPECT_EQ(fsm.state(), WalkState::IDLE);
  EXPECT_EQ(fsm.step_idx(), 0);
}

TEST(WalkingFSM, ThrowsOnEmptyFootsteps)
{
  WalkingFSM fsm(default_params());
  EXPECT_THROW(fsm.start({}), std::invalid_argument);
}

TEST(WalkingFSM, TotalWalkDurationCorrect)
{
  auto p = default_params();
  WalkingFSM fsm(p);
  const int N = 4;
  fsm.start(make_steps(N));

  // Expected: t_double (init) + N*(t_single + t_double)
  const double expected = p.t_double + N * (p.t_single + p.t_double);
  double elapsed = 0.0;

  while (!fsm.is_done()) {
    fsm.update();
    elapsed += p.dt;
  }

  EXPECT_NEAR(elapsed, expected, p.dt * 2);  // within 2 timesteps
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}