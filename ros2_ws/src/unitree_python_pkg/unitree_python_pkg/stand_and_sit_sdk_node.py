#!/usr/bin/env python3
"""Stand and sit sequence executed directly through Unitree legged SDK.

Stand-up interpolation follows unitree_guide State_FixedStand semantics:
percent is incremented before command update, so first command is already at
1/transition_steps between start and target.
"""

import enum

import rclpy
from rclpy.node import Node

from unitree_python_pkg.unitree_sdk import load_robot_interface

PMSM_MODE = 0x0A
LOWLEVEL = 0xFF

STAND_POS = [
    0.0, 0.67, -1.3,
    0.0, 0.67, -1.3,
    0.0, 0.67, -1.3,
    0.0, 0.67, -1.3,
]

KENNEL_POS = [
    0.35, 1.24, -2.81,
    -0.35, 1.24, -2.81,
    0.35, 1.24, -2.81,
    -0.35, 1.24, -2.81,
]

REAL_STANCE_KP = [60.0, 40.0, 80.0]
REAL_STANCE_KD = [5.0, 4.0, 7.0]
PASSIVE_KP = 0.0
PASSIVE_KD = 3.0


class Phase(enum.Enum):
    INIT = 0
    STAND_UP = 1
    HOLD = 2
    SIT_DOWN = 3
    PASSIVE = 4
    DONE = 5


class StandAndSitSdkNode(Node):
    def __init__(self) -> None:
        super().__init__("stand_and_sit_sdk_node")

        self.declare_parameter("hold_seconds", 5.0)
        self.declare_parameter("transition_steps", 1000)
        self.declare_parameter("publish_rate", 500.0)
        self.declare_parameter("robot_ip", "192.168.123.10")
        self.declare_parameter("local_port", 8080)
        self.declare_parameter("robot_port", 8007)
        self.declare_parameter("safety_level", 1)

        self._hold_seconds = float(
            self.get_parameter("hold_seconds").get_parameter_value().double_value
        )
        self._transition_steps = int(
            self.get_parameter("transition_steps")
            .get_parameter_value()
            .integer_value
        )
        self._publish_rate = float(
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )
        self._robot_ip = (
            self.get_parameter("robot_ip").get_parameter_value().string_value
        )
        self._local_port = int(
            self.get_parameter("local_port").get_parameter_value().integer_value
        )
        self._robot_port = int(
            self.get_parameter("robot_port").get_parameter_value().integer_value
        )
        self._safety_level = int(
            self.get_parameter("safety_level").get_parameter_value().integer_value
        )

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

        self._phase = Phase.INIT
        self._step = 0
        self._hold_elapsed = 0.0
        self._passive_elapsed = 0.0
        self._start_pos = [0.0] * 12

        self._sdk = load_robot_interface()
        self._udp = self._sdk.UDP(
            LOWLEVEL,
            self._local_port,
            str(self._robot_ip),
            self._robot_port,
        )
        self._safety = self._sdk.Safety(self._sdk.LeggedType.Go1)

        self._cmd = self._sdk.LowCmd()
        self._state = self._sdk.LowState()
        self._udp.InitCmdData(self._cmd)

        self._apply_real_stance_gains()
        self._set_phase(Phase.INIT, "waiting first SDK state")
        self.get_logger().info(
            "SDK direct control configured: "
            f"robot_ip={self._robot_ip}, local_port={self._local_port}, "
            f"robot_port={self._robot_port}, publish_rate={self._publish_rate}Hz, "
            "transition_duration="
            f"{self._transition_steps * self._dt:.3f}s, passive_duration=1.000s"
        )

        self._timer = self.create_timer(self._dt, self._control_loop)

    def _set_phase(self, phase: Phase, reason: str) -> None:
        self._phase = phase
        self.get_logger().info(f"Phase -> {phase.name}: {reason}")

    def _read_state(self) -> None:
        self._udp.Recv()
        self._udp.GetRecv(self._state)

    def _send_command(self) -> None:
        if self._safety_level > 0:
            self._safety.PowerProtect(self._cmd, self._state, int(self._safety_level))
        self._udp.SetSend(self._cmd)
        self._udp.Send()

    def _apply_real_stance_gains(self) -> None:
        for leg in range(4):
            for jt in range(3):
                idx = leg * 3 + jt
                self._cmd.motorCmd[idx].mode = PMSM_MODE
                self._cmd.motorCmd[idx].Kp = REAL_STANCE_KP[jt]
                self._cmd.motorCmd[idx].Kd = REAL_STANCE_KD[jt]
                self._cmd.motorCmd[idx].dq = 0.0
                self._cmd.motorCmd[idx].tau = 0.0

    def _apply_passive_gains(self) -> None:
        for i in range(12):
            self._cmd.motorCmd[i].mode = PMSM_MODE
            self._cmd.motorCmd[i].Kp = PASSIVE_KP
            self._cmd.motorCmd[i].Kd = PASSIVE_KD
            self._cmd.motorCmd[i].q = 0.0
            self._cmd.motorCmd[i].dq = 0.0
            self._cmd.motorCmd[i].tau = 0.0

    def _set_target(self, target: list) -> None:
        for i in range(12):
            self._cmd.motorCmd[i].q = float(target[i])

    def _lerp_target(self, start: list, target: list, alpha: float) -> None:
        for i in range(12):
            self._cmd.motorCmd[i].q = (1.0 - alpha) * start[i] + alpha * target[i]

    def _joint_positions(self) -> list:
        return [float(self._state.motorState[i].q) for i in range(12)]

    def _control_loop(self) -> None:
        self._read_state()

        if self._phase == Phase.INIT:
            self._start_pos = self._joint_positions()
            self._step = 0
            self._set_phase(Phase.STAND_UP, "first SDK state received")

        if self._phase == Phase.STAND_UP:
            self._step += 1
            alpha = min(float(self._step) / float(self._transition_steps), 1.0)
            self._lerp_target(self._start_pos, STAND_POS, alpha)
            self._send_command()

            if self._step >= int(self._transition_steps):
                self._set_target(STAND_POS)
                self._send_command()
                self._hold_elapsed = 0.0
                self._set_phase(Phase.HOLD, "stand-up complete")

        elif self._phase == Phase.HOLD:
            self._set_target(STAND_POS)
            self._send_command()

            self._hold_elapsed += self._dt
            if self._hold_elapsed >= float(self._hold_seconds):
                self._start_pos = self._joint_positions()
                self._step = 0
                self._set_phase(Phase.SIT_DOWN, "hold complete")

        elif self._phase == Phase.SIT_DOWN:
            self._step += 1
            alpha = min(float(self._step) / float(self._transition_steps), 1.0)
            self._lerp_target(self._start_pos, KENNEL_POS, alpha)
            self._send_command()

            if self._step >= int(self._transition_steps):
                self._set_target(KENNEL_POS)
                self._send_command()
                self._apply_passive_gains()
                self._passive_elapsed = 0.0
                self._set_phase(Phase.PASSIVE, "sit-down complete")

        elif self._phase == Phase.PASSIVE:
            self._send_command()
            self._passive_elapsed += self._dt
            if self._passive_elapsed >= 1.0:
                self.get_logger().info("Passive phase done. Shutting down.")
                self._timer.cancel()
                self._set_phase(Phase.DONE, "passive phase complete")
                rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = StandAndSitSdkNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()