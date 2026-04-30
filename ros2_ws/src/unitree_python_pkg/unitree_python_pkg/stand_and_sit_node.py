#!/usr/bin/env python3
"""Stand and Sit Node for Unitree Go1 real robot.

Sequence:
    1. INIT       - wait for the first /unitree_go1/low_state message
    2. STAND_UP   - linearly interpolate joints to standing position over
                    transition_steps control steps (unitree_guide semantics)
    3. HOLD       - hold standing position for hold_seconds seconds
    4. SIT_DOWN   - linearly interpolate joints to kennel position over
                    transition_steps steps
    5. PASSIVE    - damp-only command (Kp=0, Kd=3) then stop

Gains follow unitree_guide setRealStanceGain / State_Passive (real robot).
"""

import enum

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from unitree_legged_msgs.msg import BmsCmd, LowCmd, LowState, MotorCmd

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PMSM_MODE = 0x0A  # servo mode used by the original controller

MOTOR_NAMES = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
]

# Standing target  (FL_hip, FL_thigh, FL_calf,  FR_hip, FR_thigh, FR_calf,
#                   RL_hip, RL_thigh, RL_calf,  RR_hip, RR_thigh, RR_calf)
# Source: State_FixedStand.h  _targetPos
STAND_POS = [
    0.0, 0.67, -1.3,
    0.0, 0.67, -1.3,
    0.0, 0.67, -1.3,
    0.0, 0.67, -1.3,
]

# Kennel (sitting) target
# Source: State_FixedKennel.h  _targetPos
KENNEL_POS = [
    0.35, 1.24, -2.81,
    -0.35, 1.24, -2.81,
    0.35, 1.24, -2.81,
    -0.35, 1.24, -2.81,
]

# Real-robot stance gains  (setRealStanceGain in LowlevelCmd.h)
# Per joint-type inside each leg: 0=hip, 1=thigh, 2=calf
REAL_STANCE_KP = [60.0, 40.0, 80.0]  # hip, thigh, calf
REAL_STANCE_KD = [5.0, 4.0, 7.0]  # hip, thigh, calf

# Passive gains  (State_Passive, real-robot branch)
PASSIVE_KP = 0.0
PASSIVE_KD = 3.0


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------
class Phase(enum.Enum):
    INIT = 0
    STAND_UP = 1
    HOLD = 2
    SIT_DOWN = 3
    PASSIVE = 4
    DONE = 5


class StandAndSitNode(Node):
    def __init__(self) -> None:
        super().__init__("stand_and_sit_node")

        # --- parameters ---
        self.declare_parameter("hold_seconds", 5.0)
        self.declare_parameter("transition_steps", 1000)
        self.declare_parameter("publish_rate", 500.0)
        self.declare_parameter("low_cmd_topic", "/unitree_go1/low_cmd")
        self.declare_parameter("low_state_topic", "/unitree_go1/low_state")
        self.declare_parameter("wait_for_low_state", True)

        self._hold_seconds = float(self._param("hold_seconds").double_value)
        self._transition_steps = int(
            self._param("transition_steps").integer_value
        )
        self._publish_rate = float(self._param("publish_rate").double_value)
        cmd_topic = self._param("low_cmd_topic").string_value
        state_topic = self._param("low_state_topic").string_value
        self._wait_for_low_state = self._param(
            "wait_for_low_state"
        ).bool_value

        if self._transition_steps < 1:
            self.get_logger().warning(
                "transition_steps must be >= 1. Forcing to 1."
            )
            self._transition_steps = 1
        if self._publish_rate <= 0.0:
            self.get_logger().warning(
                "publish_rate must be > 0. Forcing to 500.0 Hz."
            )
            self._publish_rate = 500.0
        if self._hold_seconds < 0.0:
            self.get_logger().warning(
                "hold_seconds must be >= 0. Forcing to 0.0s."
            )
            self._hold_seconds = 0.0

        self._dt = 1.0 / self._publish_rate

        # --- internal state ---
        self._phase = Phase.INIT
        self._step = 0  # step counter inside current transition
        self._hold_elapsed = 0.0  # seconds elapsed in HOLD phase
        self._passive_elapsed = 0.0
        self._start_pos = [0.0] * 12  # interpolation start snapshot
        self._current_joint_q = [0.0] * 12

        # --- publisher ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self._pub = self.create_publisher(LowCmd, cmd_topic, qos)

        # --- subscriber ---
        self._sub = self.create_subscription(
            LowState,
            state_topic,
            self._low_state_cb,
            10,
        )

        # --- pre-build the LowCmd skeleton ---
        self._low_cmd = self._make_low_cmd()

        self._log_startup_configuration(cmd_topic, state_topic)

        # --- if not waiting for low_state, start from KENNEL_POS ---
        if not self._wait_for_low_state:
            self.get_logger().info(
                "wait_for_low_state=False - starting stand-up from "
                "KENNEL_POS immediately."
            )
            self._start_pos = list(KENNEL_POS)
            self._apply_real_stance_gains()
            self._step = 0
            self._set_phase(Phase.STAND_UP, "starting from KENNEL_POS")

        # --- control timer ---
        self._timer = self.create_timer(
            self._dt, self._control_loop
        )

        if self._wait_for_low_state:
            self.get_logger().info(
                f"Waiting for first state on '{state_topic}' ..."
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _param(self, name: str):
        return self.get_parameter(name).get_parameter_value()

    def _log_startup_configuration(
        self,
        cmd_topic: str,
        state_topic: str,
    ) -> None:
        self.get_logger().info(
            "\n".join(
                [
                    "Stand-and-sit configuration:",
                    f"  publish LowCmd topic: {cmd_topic}",
                    f"  subscribe LowState topic: {state_topic}",
                    f"  hold_seconds: {self._hold_seconds:.3f}s",
                    f"  transition_steps: {self._transition_steps}",
                    (
                        "  transition_duration: "
                        f"{self._transition_steps * self._dt:.3f}s"
                    ),
                    f"  publish_rate: {self._publish_rate:.3f} Hz",
                    "  passive_duration: 1.000s",
                    f"  wait_for_low_state: {self._wait_for_low_state}",
                    self._format_joint_targets("STAND_POS", STAND_POS),
                    self._format_joint_targets("KENNEL_POS", KENNEL_POS),
                    self._format_stance_gains(),
                ]
            )
        )

    def _format_joint_targets(self, label: str, values: list) -> str:
        lines = [f"  {label}:"]
        for leg_name, offset in (
            ("FL", 0),
            ("FR", 3),
            ("RL", 6),
            ("RR", 9),
        ):
            lines.append(
                "    "
                f"{leg_name}: "
                f"hip={values[offset]: .3f}  "
                f"thigh={values[offset + 1]: .3f}  "
                f"calf={values[offset + 2]: .3f}"
            )
        return "\n".join(lines)

    def _format_stance_gains(self) -> str:
        return (
            "  REAL_STANCE gains: "
            f"hip(kp={REAL_STANCE_KP[0]:.1f}, kd={REAL_STANCE_KD[0]:.1f})  "
            f"thigh(kp={REAL_STANCE_KP[1]:.1f}, kd={REAL_STANCE_KD[1]:.1f})  "
            f"calf(kp={REAL_STANCE_KP[2]:.1f}, kd={REAL_STANCE_KD[2]:.1f})  "
            f"PASSIVE(kp={PASSIVE_KP:.1f}, kd={PASSIVE_KD:.1f})"
        )

    def _format_float_array(self, values: list) -> str:
        return "[" + ", ".join(f"{float(value):.3f}" for value in values) + "]"

    def _log_low_state(self, msg: LowState) -> None:
        lines = [
            "First LowState data:",
            (
                "  "
                f"head={list(msg.head)}  "
                f"level_flag={msg.level_flag}  "
                f"band_width={msg.band_width}  "
                f"tick={msg.tick}  "
                f"crc={msg.crc}"
            ),
            f"  foot_force={self._format_float_array(msg.foot_force)}",
            f"  foot_force_est={self._format_float_array(msg.foot_force_est)}",
            "  idx  motor       q(rad)    dq(rad/s)  tau_est(Nm)  temp(C)",
        ]
        for i, motor in enumerate(msg.motor_state):
            lines.append(
                "  "
                f"{i:>2}   {MOTOR_NAMES[i]:<9}  "
                f"{motor.q:>8.3f}  "
                f"{motor.dq:>9.3f}  "
                f"{motor.tau_est:>11.3f}  "
                f"{motor.temperature:>7}"
            )
        self.get_logger().info("\n".join(lines))

    def _log_command(self, title: str) -> None:
        lines = [
            title,
            "  idx  motor       q(rad)       kp       kd",
        ]
        for i, motor in enumerate(self._low_cmd.motor_cmd):
            lines.append(
                "  "
                f"{i:>2}   {MOTOR_NAMES[i]:<9}  "
                f"{motor.q:>8.3f}  "
                f"{motor.kp:>7.1f}  "
                f"{motor.kd:>7.1f}"
            )
        self.get_logger().info("\n".join(lines))

    def _set_phase(self, phase: Phase, reason: str) -> None:
        self._phase = phase
        self.get_logger().info(f"Phase -> {phase.name}: {reason}")

    def _make_low_cmd(self) -> LowCmd:
        cmd = LowCmd()
        cmd.head = [0xFE, 0xEF]
        cmd.level_flag = 0x01
        cmd.frame_reserve = 0
        cmd.sn = [0, 0]
        cmd.version = [1, 0]
        cmd.band_width = int(round(self._publish_rate))
        cmd.motor_cmd = [MotorCmd() for _ in range(12)]
        cmd.bms = BmsCmd()
        cmd.bms.off = 0
        cmd.bms.reserve = [0, 0, 0]
        cmd.crc = 0
        for i in range(12):
            cmd.motor_cmd[i].mode = PMSM_MODE
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].kd = 0.0
        return cmd

    def _apply_real_stance_gains(self) -> None:
        """Set Kp/Kd for each motor matching setRealStanceGain."""
        for leg in range(4):
            for jt in range(3):
                idx = leg * 3 + jt
                self._low_cmd.motor_cmd[idx].mode = PMSM_MODE
                self._low_cmd.motor_cmd[idx].kp = REAL_STANCE_KP[jt]
                self._low_cmd.motor_cmd[idx].kd = REAL_STANCE_KD[jt]
                self._low_cmd.motor_cmd[idx].dq = 0.0
                self._low_cmd.motor_cmd[idx].tau = 0.0

    def _apply_passive_gains(self) -> None:
        """Set damping-only gains matching State_Passive (real robot)."""
        for i in range(12):
            self._low_cmd.motor_cmd[i].mode = PMSM_MODE
            self._low_cmd.motor_cmd[i].kp = PASSIVE_KP
            self._low_cmd.motor_cmd[i].kd = PASSIVE_KD
            self._low_cmd.motor_cmd[i].q = 0.0
            self._low_cmd.motor_cmd[i].dq = 0.0
            self._low_cmd.motor_cmd[i].tau = 0.0

    def _lerp_and_publish(self, start: list, target: list) -> None:
        """Publish an interpolated command between start and target."""
        alpha = min(float(self._step) / float(self._transition_steps), 1.0)
        for i in range(12):
            self._low_cmd.motor_cmd[i].q = (
                (1.0 - alpha) * start[i] + alpha * target[i]
            )
        self._pub.publish(self._low_cmd)

    def _publish_position_command(self, target: list) -> None:
        for i in range(12):
            self._low_cmd.motor_cmd[i].q = target[i]
        self._pub.publish(self._low_cmd)

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------

    def _low_state_cb(self, msg: LowState) -> None:
        for i in range(12):
            self._current_joint_q[i] = float(msg.motor_state[i].q)

        if self._phase == Phase.INIT:
            self.get_logger().info("Got first low_state - starting stand-up.")
            self._log_low_state(msg)
            # Snapshot start positions and configure gains
            self._start_pos = list(self._current_joint_q)
            self._apply_real_stance_gains()
            self._step = 0
            self._set_phase(Phase.STAND_UP, "first LowState received")

    # ------------------------------------------------------------------
    # Control loop (runs at publish_rate Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if self._phase == Phase.INIT:
            return  # waiting for first state

        if self._phase == Phase.STAND_UP:
            self._step += 1
            self._lerp_and_publish(self._start_pos, STAND_POS)
            if self._step >= self._transition_steps:
                self._publish_position_command(STAND_POS)
                self._log_command("Published STAND_UP target command:")
                self.get_logger().info(
                    "Standing up complete. "
                    f"Holding for {self._hold_seconds}s ..."
                )
                self._hold_elapsed = 0.0
                self._set_phase(Phase.HOLD, "stand-up complete")

        elif self._phase == Phase.HOLD:
            # Keep publishing the stand position
            for i in range(12):
                self._low_cmd.motor_cmd[i].q = STAND_POS[i]
            self._pub.publish(self._low_cmd)

            self._hold_elapsed += self._dt
            if self._hold_elapsed >= self._hold_seconds:
                self.get_logger().info("Hold complete. Sitting down ...")
                # Start from currently commanded pose for continuity.
                self._start_pos = [
                    float(motor.q) for motor in self._low_cmd.motor_cmd
                ]
                self._step = 0
                self._set_phase(Phase.SIT_DOWN, "hold complete")

        elif self._phase == Phase.SIT_DOWN:
            self._step += 1
            self._lerp_and_publish(self._start_pos, KENNEL_POS)
            if self._step >= self._transition_steps:
                self._publish_position_command(KENNEL_POS)
                self._log_command("Published SIT_DOWN target command:")
                self.get_logger().info(
                    "Sit-down complete. Switching to passive mode."
                )
                self._apply_passive_gains()
                self._passive_elapsed = 0.0
                self._pub.publish(self._low_cmd)
                self._log_command("Published PASSIVE command:")
                self._set_phase(Phase.PASSIVE, "sit-down complete")

        elif self._phase == Phase.PASSIVE:
            self._pub.publish(self._low_cmd)
            self._passive_elapsed += self._dt
            if self._passive_elapsed >= 1.0:
                self._log_command("Final PASSIVE command:")
                self.get_logger().info("Passive phase done. Shutting down.")
                self._timer.cancel()
                self._set_phase(Phase.DONE, "passive phase complete")
                rclpy.shutdown()

        # Phase.DONE: timer already cancelled, nothing to do


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = StandAndSitNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()