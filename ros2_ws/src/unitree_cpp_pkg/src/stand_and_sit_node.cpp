#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "unitree_legged_msgs/msg/bms_cmd.hpp"
#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"

namespace {

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

class StandAndSitNode : public rclcpp::Node {
 public:
  StandAndSitNode()
  : Node("stand_and_sit_node"),
    phase_(Phase::INIT),
    step_(0),
    hold_elapsed_(0.0),
    passive_elapsed_(0.0) {
    this->declare_parameter<double>("hold_seconds", 5.0);
    this->declare_parameter<int>("transition_steps", 1000);
    this->declare_parameter<double>("publish_rate", 500.0);
    this->declare_parameter<int>("init_state_samples", 20);
    this->declare_parameter<std::string>("low_cmd_topic", "/unitree_go1/low_cmd");
    this->declare_parameter<std::string>("low_state_topic", "/unitree_go1/low_state");
    this->declare_parameter<bool>("wait_for_low_state", true);

    hold_seconds_ = this->get_parameter("hold_seconds").as_double();
    transition_steps_ = this->get_parameter("transition_steps").as_int();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    init_state_samples_ = this->get_parameter("init_state_samples").as_int();
    low_cmd_topic_ = this->get_parameter("low_cmd_topic").as_string();
    low_state_topic_ = this->get_parameter("low_state_topic").as_string();
    wait_for_low_state_ = this->get_parameter("wait_for_low_state").as_bool();

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
    current_joint_q_.fill(0.0);
    init_q_sum_.fill(0.0);

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.durability_volatile();
    pub_ = this->create_publisher<unitree_legged_msgs::msg::LowCmd>(low_cmd_topic_, qos);
    sub_ = this->create_subscription<unitree_legged_msgs::msg::LowState>(
      low_state_topic_,
      10,
      std::bind(&StandAndSitNode::low_state_cb, this, std::placeholders::_1));

    make_low_cmd();
    log_configuration();

    if (!wait_for_low_state_) {
      RCLCPP_INFO(
        this->get_logger(),
        "wait_for_low_state=false: starting stand-up from KENNEL_POS immediately.");
      start_pos_ = kKennelPos;
      apply_real_stance_gains();
      step_ = 0;
      set_phase(Phase::STAND_UP, "starting from KENNEL_POS");
    }

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&StandAndSitNode::control_loop, this));

    if (wait_for_low_state_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting for first state on '%s'...",
        low_state_topic_.c_str());
    }
  }

 private:
  void log_configuration() {
    std::ostringstream oss;
    oss << "Stand-and-sit configuration:\n"
        << "  publish LowCmd topic: " << low_cmd_topic_ << "\n"
        << "  subscribe LowState topic: " << low_state_topic_ << "\n"
        << "  hold_seconds: " << hold_seconds_ << "s\n"
        << "  transition_steps: " << transition_steps_ << "\n"
        << "  transition_duration: " << (static_cast<double>(transition_steps_) * dt_) << "s\n"
        << "  init_state_samples: " << init_state_samples_ << "\n"
        << "  publish_rate: " << publish_rate_ << " Hz\n"
        << "  passive_duration: " << kPassiveDurationSeconds << "s\n"
        << "  wait_for_low_state: " << (wait_for_low_state_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  void set_phase(Phase new_phase, const std::string & reason) {
    phase_ = new_phase;
    RCLCPP_INFO(this->get_logger(), "Phase -> %d: %s", static_cast<int>(phase_), reason.c_str());
  }

  void make_low_cmd() {
    low_cmd_ = unitree_legged_msgs::msg::LowCmd();
    low_cmd_.head[0] = 0xFE;
    low_cmd_.head[1] = 0xEF;
    low_cmd_.level_flag = 0x01;
    low_cmd_.frame_reserve = 0;
    low_cmd_.sn[0] = 0;
    low_cmd_.sn[1] = 0;
    low_cmd_.version[0] = 1;
    low_cmd_.version[1] = 0;
    low_cmd_.band_width = static_cast<uint16_t>(std::lround(publish_rate_));
    low_cmd_.bms = unitree_legged_msgs::msg::BmsCmd();
    low_cmd_.bms.off = 0;
    low_cmd_.bms.reserve = {0, 0, 0};
    low_cmd_.reserve = 0;
    low_cmd_.crc = 0;

    for (std::size_t i = 0; i < low_cmd_.motor_cmd.size(); ++i) {
      low_cmd_.motor_cmd[i].mode = kPmsmMode;
      low_cmd_.motor_cmd[i].q = 0.0F;
      low_cmd_.motor_cmd[i].dq = 0.0F;
      low_cmd_.motor_cmd[i].tau = 0.0F;
      low_cmd_.motor_cmd[i].kp = 0.0F;
      low_cmd_.motor_cmd[i].kd = 0.0F;
      low_cmd_.motor_cmd[i].reserve = {0U, 0U, 0U};
    }
  }

  void apply_real_stance_gains() {
    for (int leg = 0; leg < 4; ++leg) {
      for (int jt = 0; jt < 3; ++jt) {
        const int idx = leg * 3 + jt;
        low_cmd_.motor_cmd[idx].mode = kPmsmMode;
        low_cmd_.motor_cmd[idx].kp = static_cast<float>(kRealStanceKp[jt]);
        low_cmd_.motor_cmd[idx].kd = static_cast<float>(kRealStanceKd[jt]);
        low_cmd_.motor_cmd[idx].dq = 0.0F;
        low_cmd_.motor_cmd[idx].tau = 0.0F;
      }
    }
  }

  void apply_passive_gains() {
    for (int i = 0; i < 12; ++i) {
      low_cmd_.motor_cmd[i].mode = kPmsmMode;
      low_cmd_.motor_cmd[i].kp = static_cast<float>(kPassiveKp);
      low_cmd_.motor_cmd[i].kd = static_cast<float>(kPassiveKd);
      low_cmd_.motor_cmd[i].q = 0.0F;
      low_cmd_.motor_cmd[i].dq = 0.0F;
      low_cmd_.motor_cmd[i].tau = 0.0F;
    }
  }

  void apply_init_safe_command() {
    for (int i = 0; i < 12; ++i) {
      low_cmd_.motor_cmd[i].mode = kPmsmMode;
      low_cmd_.motor_cmd[i].q = 0.0F;
      low_cmd_.motor_cmd[i].dq = 0.0F;
      low_cmd_.motor_cmd[i].kp = 0.0F;
      low_cmd_.motor_cmd[i].kd = 0.0F;
      low_cmd_.motor_cmd[i].tau = 0.0F;
    }
  }

  void lerp_and_publish(const std::array<double, 12> & start, const std::array<double, 12> & target) {
    const double alpha = std::min(static_cast<double>(step_) / static_cast<double>(transition_steps_), 1.0);
    for (int i = 0; i < 12; ++i) {
      const double q = (1.0 - alpha) * start[i] + alpha * target[i];
      low_cmd_.motor_cmd[i].q = static_cast<float>(q);
    }
    pub_->publish(low_cmd_);
  }

  void publish_position_command(const std::array<double, 12> & target) {
    for (int i = 0; i < 12; ++i) {
      low_cmd_.motor_cmd[i].q = static_cast<float>(target[i]);
    }
    pub_->publish(low_cmd_);
  }

  bool has_valid_sensor_state() {
    if (!has_low_state_) {
      return false;
    }

    if (latest_low_state_.tick == 0U) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "LowState tick is zero; using joint values and skipping duplicate-tick filtering.");
    }

    double max_abs_q = 0.0;
    for (int i = 0; i < 12; ++i) {
      const double q = static_cast<double>(latest_low_state_.motor_state[i].q);
      if (!std::isfinite(q) || std::abs(q) > 10.0) {
        return false;
      }
      max_abs_q = std::max(max_abs_q, std::abs(q));
    }
    return max_abs_q > 0.05;
  }

  void low_state_cb(const unitree_legged_msgs::msg::LowState::SharedPtr msg) {
    latest_low_state_ = *msg;
    has_low_state_ = true;

    for (int i = 0; i < 12; ++i) {
      current_joint_q_[i] = static_cast<double>(latest_low_state_.motor_state[i].q);
    }
  }

  void control_loop() {
    if (phase_ == Phase::DONE) {
      return;
    }

    if (phase_ == Phase::INIT) {
      if (wait_for_low_state_ && !has_low_state_) {
        apply_init_safe_command();
        pub_->publish(low_cmd_);
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Waiting for first LowState while keeping INIT command stream alive.");
        return;
      }

      if (!has_valid_sensor_state()) {
        apply_init_safe_command();
        pub_->publish(low_cmd_);
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Ignoring invalid LowState while initializing start posture.");
        return;
      }

      if (latest_low_state_.tick != 0U) {
        if (latest_low_state_.tick == last_tick_) {
          apply_init_safe_command();
          pub_->publish(low_cmd_);
          return;
        }
        last_tick_ = latest_low_state_.tick;
      }

      for (int i = 0; i < 12; ++i) {
        init_q_sum_[i] += current_joint_q_[i];
      }
      ++init_sample_count_;

      if (init_sample_count_ < init_state_samples_) {
        apply_init_safe_command();
        pub_->publish(low_cmd_);
        return;
      }

      for (int i = 0; i < 12; ++i) {
        start_pos_[i] = init_q_sum_[i] / static_cast<double>(init_sample_count_);
      }

      apply_real_stance_gains();
      publish_position_command(start_pos_);
      step_ = 0;
      set_phase(Phase::STAND_UP, "sensor initialization complete");
      return;
    }

    if (phase_ == Phase::STAND_UP) {
      ++step_;
      lerp_and_publish(start_pos_, kStandPos);
      if (step_ >= transition_steps_) {
        publish_position_command(kStandPos);
        hold_elapsed_ = 0.0;
        set_phase(Phase::HOLD, "stand-up complete");
      }
      return;
    }

    if (phase_ == Phase::HOLD) {
      publish_position_command(kStandPos);
      hold_elapsed_ += dt_;
      if (hold_elapsed_ >= hold_seconds_) {
        for (int i = 0; i < 12; ++i) {
          start_pos_[i] = current_joint_q_[i];
        }
        step_ = 0;
        set_phase(Phase::SIT_DOWN, "hold complete");
      }
      return;
    }

    if (phase_ == Phase::SIT_DOWN) {
      ++step_;
      lerp_and_publish(start_pos_, kKennelPos);
      if (step_ >= transition_steps_) {
        publish_position_command(kKennelPos);
        apply_passive_gains();
        passive_elapsed_ = 0.0;
        pub_->publish(low_cmd_);
        set_phase(Phase::PASSIVE, "sit-down complete");
      }
      return;
    }

    if (phase_ == Phase::PASSIVE) {
      pub_->publish(low_cmd_);
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
  double dt_{0.002};
  std::string low_cmd_topic_;
  std::string low_state_topic_;
  bool wait_for_low_state_{true};

  Phase phase_;
  int step_;
  double hold_elapsed_;
  double passive_elapsed_;
  int init_sample_count_{0};
  uint32_t last_tick_{0};
  bool has_low_state_{false};
  std::array<double, 12> start_pos_;
  std::array<double, 12> current_joint_q_;
  std::array<double, 12> init_q_sum_;
  unitree_legged_msgs::msg::LowState latest_low_state_;

  unitree_legged_msgs::msg::LowCmd low_cmd_;

  rclcpp::Publisher<unitree_legged_msgs::msg::LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<unitree_legged_msgs::msg::LowState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StandAndSitNode>();
  rclcpp::spin(node);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
