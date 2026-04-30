#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

namespace {

namespace UT = UNITREE_LEGGED_SDK;

constexpr uint8_t kPmsmMode = 0x0A;
constexpr double kPassiveDurationSeconds = 1.0;

constexpr std::array<double, 12> kStandPos = {
    0.0, 0.67, -1.3,
    0.0, 0.67, -1.3,
    0.0, 0.67, -1.3,
    0.0, 0.67, -1.3,
};

constexpr std::array<double, 12> kKennelPos = {
    0.35, 1.24, -2.81,
    -0.35, 1.24, -2.81,
    0.35, 1.24, -2.81,
    -0.35, 1.24, -2.81,
};

constexpr std::array<double, 3> kRealStanceKp = {60.0, 40.0, 80.0};
constexpr std::array<double, 3> kRealStanceKd = {5.0, 4.0, 7.0};
constexpr double kPassiveKp = 0.0;
constexpr double kPassiveKd = 3.0;

enum class Phase {
  INIT = 0,
  STAND_UP = 1,
  HOLD = 2,
  SIT_DOWN = 3,
  PASSIVE = 4,
  DONE = 5,
};

class StandAndSitSdkNode : public rclcpp::Node {
 public:
  StandAndSitSdkNode()
  : Node("stand_and_sit_sdk_node"),
    phase_(Phase::INIT),
    step_(0),
    hold_elapsed_(0.0),
    passive_elapsed_(0.0) {
    this->declare_parameter<double>("hold_seconds", 5.0);
    this->declare_parameter<int>("transition_steps", 1000);
    this->declare_parameter<double>("publish_rate", 500.0);
    this->declare_parameter<int>("init_state_samples", 20);
    this->declare_parameter<std::string>("robot_ip", "192.168.123.10");
    this->declare_parameter<int>("local_port", 8080);
    this->declare_parameter<int>("robot_port", 8007);
    this->declare_parameter<int>("safety_level", 1);

    hold_seconds_ = this->get_parameter("hold_seconds").as_double();
    transition_steps_ = this->get_parameter("transition_steps").as_int();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    init_state_samples_ = this->get_parameter("init_state_samples").as_int();
    robot_ip_ = this->get_parameter("robot_ip").as_string();
    local_port_ = this->get_parameter("local_port").as_int();
    robot_port_ = this->get_parameter("robot_port").as_int();
    safety_level_ = this->get_parameter("safety_level").as_int();

    if (transition_steps_ < 1) {
      RCLCPP_WARN(this->get_logger(), "transition_steps must be >= 1. Forcing to 1.");
      transition_steps_ = 1;
    }
    if (publish_rate_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "publish_rate must be > 0. Forcing to 500.0 Hz.");
      publish_rate_ = 500.0;
    }
    if (hold_seconds_ < 0.0) {
      RCLCPP_WARN(this->get_logger(), "hold_seconds must be >= 0. Forcing to 0.0s.");
      hold_seconds_ = 0.0;
    }
    if (init_state_samples_ < 1) {
      RCLCPP_WARN(this->get_logger(), "init_state_samples must be >= 1. Forcing to 1.");
      init_state_samples_ = 1;
    }

    dt_ = 1.0 / publish_rate_;
    start_pos_.fill(0.0);
    init_q_sum_.fill(0.0);

    udp_ = std::make_unique<UT::UDP>(
      UT::LOWLEVEL,
      static_cast<uint16_t>(local_port_),
      robot_ip_.c_str(),
      static_cast<uint16_t>(robot_port_));
    safety_ = std::make_unique<UT::Safety>(UT::LeggedType::Go1);

    udp_->InitCmdData(cmd_);
    apply_init_safe_command();
    set_phase(Phase::INIT, "waiting first SDK state");

    RCLCPP_INFO(
      this->get_logger(),
      "SDK configured: robot_ip=%s local_port=%d robot_port=%d publish_rate=%.3fHz transition_duration=%.3fs passive_duration=%.3fs",
      robot_ip_.c_str(),
      local_port_,
      robot_port_,
      publish_rate_,
      static_cast<double>(transition_steps_) * dt_,
      kPassiveDurationSeconds);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&StandAndSitSdkNode::control_loop, this));
  }

 private:
  void set_phase(Phase new_phase, const std::string & reason) {
    phase_ = new_phase;
    RCLCPP_INFO(this->get_logger(), "Phase -> %d: %s", static_cast<int>(phase_), reason.c_str());
  }

  void read_state() {
    udp_->Recv();
    udp_->GetRecv(state_);
  }

  void send_command(bool use_safety = true) {
    if (use_safety && safety_level_ > 0) {
      safety_->PowerProtect(cmd_, state_, safety_level_);
    }
    udp_->SetSend(cmd_);
    udp_->Send();
  }

  void send_then_read(bool use_safety = true) {
    send_command(use_safety);
    read_state();
  }

  void apply_init_safe_command() {
    for (int i = 0; i < 12; ++i) {
      cmd_.motorCmd[i].mode = kPmsmMode;
      cmd_.motorCmd[i].q = 0.0F;
      cmd_.motorCmd[i].dq = 0.0F;
      cmd_.motorCmd[i].Kp = 0.0F;
      cmd_.motorCmd[i].Kd = 0.0F;
      cmd_.motorCmd[i].tau = 0.0F;
    }
  }

  void apply_real_stance_gains() {
    for (int leg = 0; leg < 4; ++leg) {
      for (int jt = 0; jt < 3; ++jt) {
        const int idx = leg * 3 + jt;
        cmd_.motorCmd[idx].mode = kPmsmMode;
        cmd_.motorCmd[idx].Kp = static_cast<float>(kRealStanceKp[jt]);
        cmd_.motorCmd[idx].Kd = static_cast<float>(kRealStanceKd[jt]);
        cmd_.motorCmd[idx].dq = 0.0F;
        cmd_.motorCmd[idx].tau = 0.0F;
      }
    }
  }

  void apply_passive_gains() {
    for (int i = 0; i < 12; ++i) {
      cmd_.motorCmd[i].mode = kPmsmMode;
      cmd_.motorCmd[i].Kp = static_cast<float>(kPassiveKp);
      cmd_.motorCmd[i].Kd = static_cast<float>(kPassiveKd);
      cmd_.motorCmd[i].q = 0.0F;
      cmd_.motorCmd[i].dq = 0.0F;
      cmd_.motorCmd[i].tau = 0.0F;
    }
  }

  void set_target(const std::array<double, 12> & target) {
    for (int i = 0; i < 12; ++i) {
      cmd_.motorCmd[i].q = static_cast<float>(target[i]);
    }
  }

  void lerp_target(const std::array<double, 12> & start, const std::array<double, 12> & target) {
    const double alpha = std::min(static_cast<double>(step_) / static_cast<double>(transition_steps_), 1.0);
    for (int i = 0; i < 12; ++i) {
      const double q = (1.0 - alpha) * start[i] + alpha * target[i];
      cmd_.motorCmd[i].q = static_cast<float>(q);
    }
  }

  std::array<double, 12> joint_positions() const {
    std::array<double, 12> q{};
    for (int i = 0; i < 12; ++i) {
      q[i] = static_cast<double>(state_.motorState[i].q);
    }
    return q;
  }

  bool has_valid_sensor_state() {
    if (state_.tick == 0U) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "SDK LowState tick is zero; using joint values and skipping duplicate-tick filtering.");
    }

    double max_abs_q = 0.0;
    for (int i = 0; i < 12; ++i) {
      const double q = static_cast<double>(state_.motorState[i].q);
      if (!std::isfinite(q) || std::abs(q) > 10.0) {
        return false;
      }
      max_abs_q = std::max(max_abs_q, std::abs(q));
    }
    return max_abs_q > 0.05;
  }

  void control_loop() {
    if (phase_ == Phase::INIT) {
      if (!has_valid_sensor_state()) {
        // Keep traffic alive while waiting for the first valid feedback frame.
        apply_init_safe_command();
        send_then_read(false);
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Ignoring invalid SDK LowState while initializing start posture.");
        return;
      }

      if (state_.tick != 0U) {
        if (state_.tick == last_tick_) {
          apply_init_safe_command();
          send_then_read(false);
          return;
        }
        last_tick_ = state_.tick;
      }

      const auto q = joint_positions();
      for (int i = 0; i < 12; ++i) {
        init_q_sum_[i] += q[i];
      }
      ++init_sample_count_;

      if (init_sample_count_ < init_state_samples_) {
        apply_init_safe_command();
        send_then_read(false);
        return;
      }

      for (int i = 0; i < 12; ++i) {
        start_pos_[i] = init_q_sum_[i] / static_cast<double>(init_sample_count_);
      }

      apply_real_stance_gains();
      set_target(start_pos_);
      send_then_read(false);
      step_ = 0;
      set_phase(Phase::STAND_UP, "sensor initialization complete");
      return;
    }

    if (phase_ == Phase::STAND_UP) {
      ++step_;
      lerp_target(start_pos_, kStandPos);
      send_then_read();

      if (step_ >= transition_steps_) {
        set_target(kStandPos);
        send_then_read();
        hold_elapsed_ = 0.0;
        set_phase(Phase::HOLD, "stand-up complete");
      }
      return;
    }

    if (phase_ == Phase::HOLD) {
      set_target(kStandPos);
      send_then_read();

      hold_elapsed_ += dt_;
      if (hold_elapsed_ >= hold_seconds_) {
        start_pos_ = joint_positions();
        step_ = 0;
        set_phase(Phase::SIT_DOWN, "hold complete");
      }
      return;
    }

    if (phase_ == Phase::SIT_DOWN) {
      ++step_;
      lerp_target(start_pos_, kKennelPos);
      send_then_read();

      if (step_ >= transition_steps_) {
        set_target(kKennelPos);
        send_then_read();
        apply_passive_gains();
        passive_elapsed_ = 0.0;
        set_phase(Phase::PASSIVE, "sit-down complete");
      }
      return;
    }

    if (phase_ == Phase::PASSIVE) {
      send_then_read();
      passive_elapsed_ += dt_;
      if (passive_elapsed_ >= kPassiveDurationSeconds) {
        timer_->cancel();
        set_phase(Phase::DONE, "passive phase complete");
        rclcpp::shutdown();
      }
    }
  }

  double hold_seconds_{5.0};
  int transition_steps_{1000};
  double publish_rate_{500.0};
  int init_state_samples_{20};
  std::string robot_ip_;
  int local_port_{8080};
  int robot_port_{8007};
  int safety_level_{1};
  double dt_{0.002};

  Phase phase_;
  int step_;
  double hold_elapsed_;
  double passive_elapsed_;
  int init_sample_count_{0};
  uint32_t last_tick_{0};
  std::array<double, 12> start_pos_;
  std::array<double, 12> init_q_sum_;

  UT::LowCmd cmd_{};
  UT::LowState state_{};
  std::unique_ptr<UT::UDP> udp_;
  std::unique_ptr<UT::Safety> safety_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StandAndSitSdkNode>();
  rclcpp::spin(node);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
