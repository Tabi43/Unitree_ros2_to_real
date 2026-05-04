#include <unitree_ros2_interface/face-lights-interface.hpp>

FaceLedNode::FaceLedNode()
  : Node("unitree_face_lights_interface"),
    led_color_{0, 0, 0},
    state_(LED_STATE::STATIC),
    ticks_(0)
    {
        // Parametro: periodo (ms) del messaggio periodico "node alive" pubblicato dentro onTick()
        alive_period_ms_ = this->declare_parameter<int>("alive_period_ms", 1000);
        if (alive_period_ms_ < 0) alive_period_ms_ = 0;

        face_light_client_ = std::make_shared<FaceLightClient>();
        if (!face_light_client_) {
            throw std::runtime_error("Failed to create FaceLightClient");
        }

        // Spegni LED all'avvio
        face_light_client_->setAllLed(face_light_client_->black);
        face_light_client_->sendCmd();

        // Publisher per i log
        pub_log_ = this->create_publisher<std_msgs::msg::String>("face_led_log", 10);

        // Servizi
        srv_color_ = this->create_service<unitree_ros2_interface::srv::SetLedColor>(
            "set_face_color",
            std::bind(&FaceLedNode::onSetLedColor, this, std::placeholders::_1, std::placeholders::_2));

        srv_anim_ = this->create_service<unitree_ros2_interface::srv::SetLedAnimation>(
            "set_face_animation",
            std::bind(&FaceLedNode::onSetFaceAnimation, this, std::placeholders::_1, std::placeholders::_2));

        // Timer a 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FaceLedNode::onTick, this));

        // Inizializza il tempo dell'ultimo "alive"
        last_alive_time_ = this->now();

        publish_log("INFO", "Unitree ROS2 Face LED node started");
    }

FaceLedNode::~FaceLedNode() {
    publish_log("INFO", "Shutting down Unitree ROS2 Face LED node");
    // Spegni LED prima di chiudere
    face_light_client_->setAllLed(face_light_client_->black);
    face_light_client_->sendCmd();
}

void FaceLedNode::onSetLedColor(
    const std::shared_ptr<unitree_ros2_interface::srv::SetLedColor::Request> req,
    std::shared_ptr<unitree_ros2_interface::srv::SetLedColor::Response> res) {
        publish_log("INFO",
        "Service call: set_face_color r=" + std::to_string(req->r) +
        " g=" + std::to_string(req->g) +
        " b=" + std::to_string(req->b) +
        " time=" + std::to_string(req->time));

        const std::array<uint8_t, 3> color{
        static_cast<uint8_t>(req->r),
        static_cast<uint8_t>(req->g),
        static_cast<uint8_t>(req->b)
        };

        const rclcpp::Time now = this->now();

        {
        std::lock_guard<std::mutex> lock(mtx_);
        led_color_ = color;
        state_ = LED_STATE::STATIC;
        current_anim_ = nullptr;
        anim_frame_idx_ = 0;
        anim_frame_elapsed_ms_ = 0;

        if (req->time < 0.0) {
            timed_color_active_ = false;
        } else {
            timed_color_active_ = true;
            timed_color_deadline_ = now + rclcpp::Duration::from_seconds(req->time);
        }
        }

        res->res = true;

        publish_log("INFO", "Service request on set face leds color (R=" + std::to_string(req->r) + " G=" + std::to_string(req->g) + " B=" + std::to_string(req->b) + " time=" + std::to_string(req->time) + ") processed successfully");
    } 

  void FaceLedNode::onSetFaceAnimation(
    const std::shared_ptr<unitree_ros2_interface::srv::SetLedAnimation::Request> req,
    std::shared_ptr<unitree_ros2_interface::srv::SetLedAnimation::Response> res) {
    
    publish_log("INFO", "Service call: set_face_animation id=" + std::to_string(req->id));

    const auto & registry = getAnimationRegistry();
    auto it = registry.find(static_cast<uint32_t>(req->id));

    std::lock_guard<std::mutex> lock(mtx_);

    if (it != registry.end()) {
        current_anim_ = &(it->second);
        anim_frame_idx_ = 0;
        anim_frame_elapsed_ms_ = 0;
        timed_color_active_ = false;
        state_ = LED_STATE::ANIMATION;
        ticks_ = 0;
        res->res = true;
        publish_log("INFO", "Service response: set_face_animation res=true (id=" + std::to_string(req->id) + ")");
    } else {
        publish_log("WARN", "Unknown LED animation ID: " + std::to_string(req->id));
        stopAnimation();
        res->res = false;
        publish_log("INFO", "Service response: set_face_animation res=false");
    }
    }

  // ---- loop 10Hz ----
  void FaceLedNode::onTick() {
    const rclcpp::Time now = this->now();

    // Pub periodica "alive" (semplice) basata su parametro, ma eseguita dentro onTick
    if (alive_period_ms_ > 0) {
        const auto dt = now - last_alive_time_;
        if (dt.nanoseconds() >= static_cast<int64_t>(alive_period_ms_) * 1000000LL) {
        publish_log("DEBUG", "ACK alive (node running)");
        last_alive_time_ = now;
        }
    }

    LED_STATE state_local;
    std::array<uint8_t, 3> color_local;
    bool timed_color_expired = false;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (state_ == LED_STATE::STATIC && timed_color_active_ && now >= timed_color_deadline_) {
            timed_color_active_ = false;
            led_color_ = {0, 0, 0};
            timed_color_expired = true;
        }

        state_local = state_;
        color_local = led_color_;
    }

    if (timed_color_expired) {
        publish_log("INFO", "Timed face color expired: LEDs turned off");
    }

    if (state_local == LED_STATE::STATIC) {
        face_light_client_->setAllLed(color_local.data());
        face_light_client_->sendCmd();
    } 

    if (state_local == LED_STATE::ANIMATION) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (current_anim_ && anim_frame_idx_ < current_anim_->frames.size()) {
            const auto & frame = current_anim_->frames[anim_frame_idx_];
            applyFrame(frame);
            anim_frame_elapsed_ms_ += 100; // tick period
            if (anim_frame_elapsed_ms_ >= frame.duration_ms) {
                anim_frame_elapsed_ms_ = 0;
                ++anim_frame_idx_;
                if (anim_frame_idx_ >= current_anim_->frames.size()) {
                    if (current_anim_->is_loop) {
                        anim_frame_idx_ = 0;
                    } else {
                        stopAnimation();
                    }
                }
            }
        } else {
            stopAnimation();
        }
    }

    ++ticks_;
    }

void FaceLedNode::applyFrame(const LedFrame & frame) {
    for (uint32_t i = 0; i < NUM_FACE_LEDS; ++i) {
        face_light_client_->setLedColor(i, frame.colors[i].data());
    }
    face_light_client_->sendCmd();
}

void FaceLedNode::stopAnimation() {
    current_anim_ = nullptr;
    anim_frame_idx_ = 0;
    anim_frame_elapsed_ms_ = 0;
    timed_color_active_ = false;
    state_ = LED_STATE::STATIC;
    led_color_ = {0, 0, 0};
}

void FaceLedNode::publish_log(const std::string & level, const std::string & msg) {
    const std::string full = "[" + level + "] " + msg;

    // ROS logger
    if (level == "ERROR") {
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
    } else if (level == "WARN") {
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
    } else if (level == "DEBUG") {
        RCLCPP_DEBUG(this->get_logger(), "%s", msg.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
    }

    // Topic log
    std_msgs::msg::String m;
    m.data = full;
    pub_log_->publish(m);
}