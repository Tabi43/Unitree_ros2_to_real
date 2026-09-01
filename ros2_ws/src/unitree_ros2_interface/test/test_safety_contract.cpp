/**
 * Contract test for UNITREE_LEGGED_SDK::Safety, the guards applySafetyClamps() runs over
 * every outgoing LowCmd.
 *
 * The SDK ships as a precompiled static library and its header documents none of the
 * behaviour these guards are relied on for, so the claims are pinned here:
 *
 *  - PositionLimit clamps commanded angles and NOTHING else. If it ever started touching
 *    tau/Kp/Kd it would silently detune the whole-body controller that drives this
 *    interface, which is precisely why it is safe to leave enabled by default.
 *  - The protections trip by forcing all twelve motors to damping (mode 0), not by
 *    scaling torques. That is what makes a trip a fault to report rather than a quiet
 *    degradation.
 *  - PositionProtect trips on the Go1's own resting pose, which is why
 *    position_protect_limit defaults to 0 (disabled).
 *
 * No robot and no ROS node required.
 */

#include <gtest/gtest.h>

#include <cmath>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/go1_const.h"

namespace {

// Joint indices within a leg: 0 = hip (HAA), 1 = thigh (HFE), 2 = calf (KFE).
constexpr int kHip = 0;
constexpr int kThigh = 1;
constexpr int kCalf = 2;

int jointIndex(int leg, int joint) { return leg * 3 + joint; }

/// Mid-range angle for each joint type, i.e. a pose no guard should object to.
float midRange(int joint) {
  switch (joint) {
    case kHip:   return static_cast<float>((UNITREE_LEGGED_SDK::go1_Hip_max + UNITREE_LEGGED_SDK::go1_Hip_min) / 2.0);
    case kThigh: return static_cast<float>((UNITREE_LEGGED_SDK::go1_Thigh_max + UNITREE_LEGGED_SDK::go1_Thigh_min) / 2.0);
    default:     return static_cast<float>((UNITREE_LEGGED_SDK::go1_Calf_max + UNITREE_LEGGED_SDK::go1_Calf_min) / 2.0);
  }
}

/// A LowCmd in servo mode with a distinctive torque/gain signature to detect tampering.
UNITREE_LEGGED_SDK::LowCmd makeCmd(float q) {
  UNITREE_LEGGED_SDK::LowCmd cmd{};
  for (int i = 0; i < 12; ++i) {
    cmd.motorCmd[i].mode = 0x0A;  // servo
    cmd.motorCmd[i].q = q;
    cmd.motorCmd[i].dq = 0.0f;
    cmd.motorCmd[i].tau = 3.5f;
    cmd.motorCmd[i].Kp = 60.0f;
    cmd.motorCmd[i].Kd = 5.0f;
  }
  return cmd;
}

/// A LowState describing a robot standing still in a legal pose.
UNITREE_LEGGED_SDK::LowState makeRestingState() {
  UNITREE_LEGGED_SDK::LowState state{};
  for (int leg = 0; leg < 4; ++leg) {
    for (int joint = 0; joint < 3; ++joint) {
      auto & motor = state.motorState[jointIndex(leg, joint)];
      motor.q = midRange(joint);
      motor.dq = 0.0f;
      motor.tauEst = 0.0f;
    }
  }
  return state;
}

bool allMotorsDamping(const UNITREE_LEGGED_SDK::LowCmd & cmd) {
  for (int i = 0; i < 12; ++i) {
    if (cmd.motorCmd[i].mode != 0) {
      return false;
    }
  }
  return true;
}

}  // namespace

/**
 * Commanded angles far outside the mechanical range must come back inside it. This is the
 * guard that runs on every outgoing command by default.
 */
TEST(SafetyContract, PositionLimitClampsCommandedJointAngles) {
  UNITREE_LEGGED_SDK::Safety safety(UNITREE_LEGGED_SDK::LeggedType::Go1);
  auto cmd = makeCmd(10.0f);  // wildly out of range in the positive direction

  safety.PositionLimit(cmd);

  for (int leg = 0; leg < 4; ++leg) {
    EXPECT_LE(cmd.motorCmd[jointIndex(leg, kHip)].q, UNITREE_LEGGED_SDK::go1_Hip_max + 1e-4)
        << "leg " << leg << " hip not clamped";
    EXPECT_LE(cmd.motorCmd[jointIndex(leg, kThigh)].q, UNITREE_LEGGED_SDK::go1_Thigh_max + 1e-4)
        << "leg " << leg << " thigh not clamped";
    EXPECT_LE(cmd.motorCmd[jointIndex(leg, kCalf)].q, UNITREE_LEGGED_SDK::go1_Calf_max + 1e-4)
        << "leg " << leg << " calf not clamped";
  }

  auto cmd_low = makeCmd(-10.0f);
  safety.PositionLimit(cmd_low);

  for (int leg = 0; leg < 4; ++leg) {
    EXPECT_GE(cmd_low.motorCmd[jointIndex(leg, kHip)].q, UNITREE_LEGGED_SDK::go1_Hip_min - 1e-4);
    EXPECT_GE(cmd_low.motorCmd[jointIndex(leg, kThigh)].q, UNITREE_LEGGED_SDK::go1_Thigh_min - 1e-4);
    EXPECT_GE(cmd_low.motorCmd[jointIndex(leg, kCalf)].q, UNITREE_LEGGED_SDK::go1_Calf_min - 1e-4);
  }
}

/**
 * The load-bearing property behind enabling PositionLimit unconditionally: it must not be
 * able to alter a torque command. If this ever fails, the guard has become capable of
 * silently detuning the WBC and must no longer default to on.
 */
TEST(SafetyContract, PositionLimitLeavesTorqueAndGainsUntouched) {
  UNITREE_LEGGED_SDK::Safety safety(UNITREE_LEGGED_SDK::LeggedType::Go1);
  auto cmd = makeCmd(10.0f);

  safety.PositionLimit(cmd);

  for (int i = 0; i < 12; ++i) {
    EXPECT_FLOAT_EQ(cmd.motorCmd[i].tau, 3.5f) << "motor " << i << ": tau was modified";
    EXPECT_FLOAT_EQ(cmd.motorCmd[i].Kp, 60.0f) << "motor " << i << ": Kp was modified";
    EXPECT_FLOAT_EQ(cmd.motorCmd[i].Kd, 5.0f) << "motor " << i << ": Kd was modified";
    EXPECT_EQ(cmd.motorCmd[i].mode, 0x0A) << "motor " << i << ": mode was modified";
  }
}

/**
 * A still robot in a legal pose draws no mechanical power, so the power guard must pass
 * the command through untouched even at its strictest setting. A guard that trips here
 * would drop the robot at the slightest provocation.
 */
TEST(SafetyContract, PowerProtectPassesAnIdleRobot) {
  UNITREE_LEGGED_SDK::Safety safety(UNITREE_LEGGED_SDK::LeggedType::Go1);
  auto cmd = makeCmd(midRange(kHip));
  auto state = makeRestingState();

  for (int i = 0; i < 100; ++i) {
    ASSERT_GE(safety.PowerProtect(cmd, state, 1), 0) << "tripped on iteration " << i;
  }

  EXPECT_FALSE(allMotorsDamping(cmd)) << "idle robot was forced to damping";
}

/**
 * The trip mechanism: a measured joint outside the limit table by more than the tolerance
 * must force every motor to damping and report failure. This also documents why
 * position_protect_limit defaults to 0: the offending value used here (-2.9 rad calf) is
 * close to where a Go1 rests on its hocks, while go1_const.h declares the calf minimum as
 * -2.721 rad, so the guard would trip during every bring-up.
 */
TEST(SafetyContract, PositionProtectTripsToDampingOnOutOfRangeJoint) {
  UNITREE_LEGGED_SDK::Safety safety(UNITREE_LEGGED_SDK::LeggedType::Go1);
  auto cmd = makeCmd(midRange(kHip));
  auto state = makeRestingState();

  const float out_of_range = static_cast<float>(UNITREE_LEGGED_SDK::go1_Calf_min) - 0.18f;
  state.motorState[jointIndex(0, kCalf)].q = out_of_range;

  const int rc = safety.PositionProtect(cmd, state, 0.087);

  EXPECT_LT(rc, 0) << "out-of-range joint was not reported";
  EXPECT_TRUE(allMotorsDamping(cmd)) << "trip did not force the motors to damping";
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
