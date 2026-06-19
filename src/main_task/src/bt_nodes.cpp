#include "main_task/bt_nodes.h"

/**
 * @brief Retrieves the shared context from the blackboard.
 */
static std::shared_ptr<RobotContext> getCtx(const BT::NodeConfig& cfg) {
    std::shared_ptr<RobotContext> ctx;
    if (!cfg.blackboard->get("robot_context", ctx)) {
        throw BT::RuntimeError("MISSING robot_context on blackboard.");
    }
    return ctx;
}

// --- AllSystemsOK -----------------------------------------------------------

BT::NodeStatus AllSystemsOK::onStart() {
    auto timeout_in = getInput<double>("timeout_s");
    timeout_s_  = timeout_in ? timeout_in.value() : 20.0;
    start_time_ = std::chrono::steady_clock::now();

    auto ctx = getCtx(config());
    RCLCPP_INFO(ctx->node->get_logger(),
                "[AllSystemsOK] Waiting for all systems (timeout %.0f s)...", timeout_s_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AllSystemsOK::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_).count();

    bool all_ok = true;

    if (!ctx->imu_received) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
                             "[AllSystemsOK] Waiting for /imu ... (%.0fs elapsed)", elapsed);
        all_ok = false;
    }

    if (!ctx->zed_ok) {
        RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 2000,
                             "[AllSystemsOK] ZED camera not healthy. (%.0fs elapsed)", elapsed);
        all_ok = false;
    }

    if (all_ok) {
        RCLCPP_INFO(ctx->node->get_logger(),
                    "[AllSystemsOK] All systems OK after %.1fs.", elapsed);
        return BT::NodeStatus::SUCCESS;
    }

    if (elapsed >= timeout_s_) {
        RCLCPP_ERROR(ctx->node->get_logger(),
                     "[AllSystemsOK] Timeout after %.0fs — aborting mission.", timeout_s_);
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void AllSystemsOK::onHalted() {
    RCLCPP_WARN(getCtx(config())->node->get_logger(), "[AllSystemsOK] Halted.");
}

// --- DiveToDepth ------------------------------------------------------------

BT::NodeStatus DiveToDepth::onStart() {
    auto depth_in = getInput<double>("target_depth");
    if (!depth_in) throw BT::RuntimeError("DiveToDepth: missing [target_depth]");
    target_z_ = depth_in.value();

    auto ctx = getCtx(config());
    ctx->target_depth = target_z_;
    RCLCPP_INFO(ctx->node->get_logger(), "[DiveToDepth] Diving to z = %.2f m", target_z_);
    
    ctx->publishToPico(0.0f, 0.0f, (float)target_z_, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DiveToDepth::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double current_z = ctx->latest_altimeter;
    if (std::abs(target_z_ - current_z) < ctx->depth_tolerance) {
        RCLCPP_INFO(ctx->node->get_logger(), "[DiveToDepth] Target depth reached.");
        return BT::NodeStatus::SUCCESS;
    }

    ctx->publishToPico(0.0f, 0.0f, (float)target_z_, 0);
    return BT::NodeStatus::RUNNING;
}

void DiveToDepth::onHalted() { 
    getCtx(config())->stopMotion(); 
}

// --- IsObjectSeen -----------------------------------------------------------

BT::NodeStatus IsObjectSeen::tick() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    auto obj = getInput<std::string>("object");
    if (!obj) throw BT::RuntimeError("IsObjectSeen: missing [object]");

    return ctx->isObjectSeen(obj.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --- Do360Turn --------------------------------------------------------------

BT::NodeStatus Do360Turn::onStart() {
    auto obj = getInput<std::string>("success_when_seen");
    if (!obj) throw BT::RuntimeError("Do360Turn: missing [success_when_seen]");
    target_object_ = obj.value();

    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    prev_yaw_       = ctx->getCurrentYaw();
    accumulated_yaw_ = 0.0;
    confirm_frames_  = 0;

    RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] Searching for %s...", target_object_.c_str());
    ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Do360Turn::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    if (ctx->isObjectSeen(target_object_)) {
        confirm_frames_++;

        if (confirm_frames_ == 1) {
            ctx->publishToPico(ctx->base_yaw_speed * 0.3f, 0.0f, (float)ctx->target_depth, 0);
            RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] %s spotted, slowing to confirm...", target_object_.c_str());
        }

        if (confirm_frames_ >= 4) {
            ctx->stopMotion();
            RCLCPP_INFO(ctx->node->get_logger(), "[Do360Turn] %s confirmed (%d frames).", target_object_.c_str(), confirm_frames_);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    if (confirm_frames_ > 0) {
        RCLCPP_WARN(ctx->node->get_logger(), "[Do360Turn] %s lost after %d confirm frames, resuming spin.", target_object_.c_str(), confirm_frames_);
        confirm_frames_ = 0;
    }

    double current_yaw = ctx->getCurrentYaw();
    accumulated_yaw_ += std::abs(normalizeAngle(current_yaw - prev_yaw_));
    prev_yaw_ = current_yaw;

    if (accumulated_yaw_ >= (2.0 * M_PI)) {
        ctx->stopMotion();
        RCLCPP_WARN(ctx->node->get_logger(), "[Do360Turn] Full rotation complete. %s not found.", target_object_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    ctx->publishToPico(ctx->base_yaw_speed, 0.0f, (float)ctx->target_depth, 0);
    return BT::NodeStatus::RUNNING;
}

void Do360Turn::onHalted() { 
    getCtx(config())->stopMotion(); 
}

// --- DriveThruGate (Align then Surge) ---------------------------------------

BT::NodeStatus DriveThruGate::onStart() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    phase_ = Phase::ALIGN_1;
    gate_lost_frames_ = 0;
    align_confirm_frames_ = 0;
    locked_yaw_ = 0.0;
    filtered_yaw_err_ = 0.0;

    RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Starting DriveThruGate. Entering ALIGN_1 phase.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DriveThruGate::onRunning() {
    auto ctx = getCtx(config());
    rclcpp::spin_some(ctx->node);

    double ox, oy, oz, score = 0.0;
    // Try to locate target object ("shark" first, fallback to "GATE")
    bool seen = ctx->getObjectPosition("shark", ox, oy, oz, &score);
    if (!seen) {
        seen = ctx->getObjectPosition("GATE", ox, oy, oz, &score);
    }

    // --- Phase 1: Initial Alignment (ALIGN_1) ---
    if (phase_ == Phase::ALIGN_1) {
        if (!seen) {
            ctx->publishToPico(0.0f, 0.0f, (float)ctx->target_depth, 0);
            RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 1000,
                                 "[DriveThruGate] ALIGN_1: Target not seen, holding still.");
            return BT::NodeStatus::RUNNING;
        }

        double raw_norm_x = ox / std::max(oz, 0.5);
        constexpr double ALIGN_THRESHOLD = 0.05;

        if (std::abs(raw_norm_x) < ALIGN_THRESHOLD) {
            align_confirm_frames_++;
            if (align_confirm_frames_ >= 5) {
                phase_ = Phase::APPROACH;
                align_confirm_frames_ = 0;
                filtered_yaw_err_ = 0.0;
                RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] ALIGN_1 complete. Entering APPROACH phase.");
                return BT::NodeStatus::RUNNING;
            }
        } else {
            align_confirm_frames_ = 0;
        }

        // Apply EMA filter on raw heading error
        double error = -raw_norm_x;
        constexpr double alpha = 0.25;
        if (filtered_yaw_err_ == 0.0) {
            filtered_yaw_err_ = error;
        } else {
            filtered_yaw_err_ = alpha * error + (1.0 - alpha) * filtered_yaw_err_;
        }

        float yaw_cmd = (float)filtered_yaw_err_;
        yaw_cmd = std::max(-ctx->base_yaw_speed, std::min(ctx->base_yaw_speed, yaw_cmd));

        ctx->publishToPico(yaw_cmd, 0.0f, (float)ctx->target_depth, 0);
        RCLCPP_INFO_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 500,
                             "[DriveThruGate] ALIGN_1: raw_norm_x = %.3f, error = %.3f, filtered_yaw_cmd = %.3f", 
                             raw_norm_x, error, yaw_cmd);
    }
    // --- Phase 2: Approach the target (APPROACH) ---
    else if (phase_ == Phase::APPROACH) {
        if (!seen) {
            // Target lost temporarily — maintain course and move forward slowly
            ctx->publishToPico(0.0f, ctx->base_surge_speed * 0.5f, (float)ctx->target_depth, 0);
            RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 1000,
                                 "[DriveThruGate] APPROACH: Target lost, surging on last heading.");
            return BT::NodeStatus::RUNNING;
        }

        // Distance threshold to stop approaching and re-align
        constexpr double APPROACH_THRESHOLD = 1.5;
        if (oz <= APPROACH_THRESHOLD) {
            phase_ = Phase::ALIGN_2;
            align_confirm_frames_ = 0;
            filtered_yaw_err_ = 0.0;
            ctx->publishToPico(0.0f, 0.0f, (float)ctx->target_depth, 0); // Stop motion
            RCLCPP_INFO(ctx->node->get_logger(), 
                        "[DriveThruGate] APPROACH complete (distance = %.2f m <= %.2f m). Entering ALIGN_2 phase.", 
                        oz, APPROACH_THRESHOLD);
            return BT::NodeStatus::RUNNING;
        }

        // Align dynamically while approaching
        double raw_norm_x = ox / std::max(oz, 0.5);
        double error = -raw_norm_x;
        constexpr double alpha = 0.25;
        if (filtered_yaw_err_ == 0.0) {
            filtered_yaw_err_ = error;
        } else {
            filtered_yaw_err_ = alpha * error + (1.0 - alpha) * filtered_yaw_err_;
        }

        float yaw_cmd = (float)filtered_yaw_err_;
        yaw_cmd = std::max(-ctx->base_yaw_speed, std::min(ctx->base_yaw_speed, yaw_cmd));

        // Command slower forward speed for precise alignment during approach
        ctx->publishToPico(yaw_cmd, ctx->base_surge_speed * 0.6f, (float)ctx->target_depth, 0);
        RCLCPP_INFO_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 500,
                             "[DriveThruGate] APPROACH: distance = %.2f m, yaw_cmd = %.3f", oz, yaw_cmd);
    }
    // --- Phase 3: Secondary Alignment (ALIGN_2) ---
    else if (phase_ == Phase::ALIGN_2) {
        if (!seen) {
            ctx->publishToPico(0.0f, 0.0f, (float)ctx->target_depth, 0);
            RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 1000,
                                 "[DriveThruGate] ALIGN_2: Target not seen, holding still.");
            return BT::NodeStatus::RUNNING;
        }

        double raw_norm_x = ox / std::max(oz, 0.5);
        constexpr double ALIGN_THRESHOLD = 0.04; // Slightly tighter for precision close-up

        if (std::abs(raw_norm_x) < ALIGN_THRESHOLD) {
            align_confirm_frames_++;
            if (align_confirm_frames_ >= 5) {
                // Secondary alignment complete. Lock yaw and switch to SURGE.
                locked_yaw_ = ctx->getCurrentYaw();
                phase_ = Phase::SURGE;
                gate_lost_frames_ = 0;
                ctx->publishToPico(0.0f, ctx->base_surge_speed, (float)ctx->target_depth, 0);
                RCLCPP_INFO(ctx->node->get_logger(), 
                            "[DriveThruGate] ALIGN_2 complete. Locked Yaw to %.2f rad. Switching to SURGE.", locked_yaw_);
                return BT::NodeStatus::RUNNING;
            }
        } else {
            align_confirm_frames_ = 0;
        }

        // Apply EMA filter on raw heading error
        double error = -raw_norm_x;
        constexpr double alpha = 0.25;
        if (filtered_yaw_err_ == 0.0) {
            filtered_yaw_err_ = error;
        } else {
            filtered_yaw_err_ = alpha * error + (1.0 - alpha) * filtered_yaw_err_;
        }

        float yaw_cmd = (float)filtered_yaw_err_;
        yaw_cmd = std::max(-ctx->base_yaw_speed, std::min(ctx->base_yaw_speed, yaw_cmd));

        ctx->publishToPico(yaw_cmd, 0.0f, (float)ctx->target_depth, 0);
        RCLCPP_INFO_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 500,
                             "[DriveThruGate] ALIGN_2: raw_norm_x = %.3f, error = %.3f, filtered_yaw_cmd = %.3f", 
                             raw_norm_x, error, yaw_cmd);
    }
    // --- Phase 4: Final Surge (SURGE) ---
    else if (phase_ == Phase::SURGE) {
        if (!seen) {
            gate_lost_frames_++;
            if (gate_lost_frames_ >= 30) {
                ctx->stopMotion();
                RCLCPP_INFO(ctx->node->get_logger(), "[DriveThruGate] Target lost for %d frames. Gate cleared! SUCCESS.", gate_lost_frames_);
                return BT::NodeStatus::SUCCESS;
            }
        } else {
            gate_lost_frames_ = 0;
        }

        // Hold the locked yaw heading and surge forward
        double cur_yaw = ctx->getCurrentYaw();
        double yaw_err = normalizeAngle(locked_yaw_ - cur_yaw);
        
        ctx->publishToPico((float)yaw_err, ctx->base_surge_speed, (float)ctx->target_depth, 0);
        RCLCPP_INFO_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 500,
                             "[DriveThruGate] SURGE: cur_yaw = %.2f, target = %.2f, error = %.2f, lost_frames = %d", 
                             cur_yaw, locked_yaw_, yaw_err, gate_lost_frames_);
    }

    return BT::NodeStatus::RUNNING;
}

void DriveThruGate::onHalted() {
    getCtx(config())->stopMotion();
}
