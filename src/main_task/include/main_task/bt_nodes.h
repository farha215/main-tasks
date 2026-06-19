#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "auv_msgs/msg/control_command.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

struct RobotContext {
    rclcpp::Node::SharedPtr node;
    std::mutex mtx;
    sensor_msgs::msg::Imu::SharedPtr latest_imu;
    double latest_altimeter = 0.0;
    double target_depth = 0.0;
    vision_msgs::msg::Detection3DArray::SharedPtr latest_detections;

    bool imu_received = false;
    bool altimeter_received = false;
    bool zed_ok = true;
    bool image_received = false;
    double last_image_t = 0.0;

    rclcpp::Publisher<auv_msgs::msg::ControlCommand>::SharedPtr pico_pub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr alt_sub;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr zed_diag_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    // --- MISSION CONFIGURATION (LOADED VIA YAML) ---
    float base_surge_speed = 0.1f;
    float base_yaw_speed = 0.1f;
    float gate_conf_thresh = 0.6f;
    float depth_tolerance = 0.15f;

    double getCurrentYaw() {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_imu)
            return 0.0;
        tf2::Quaternion q(
                latest_imu->orientation.x,
                latest_imu->orientation.y,
                latest_imu->orientation.z,
                latest_imu->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    bool isObjectSeen(const std::string &object) {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_detections)
            return false;
        for (const auto &det : latest_detections->detections) {
            if (det.results.empty())
                continue;
            const auto &hyp = det.results[0].hypothesis;
            const auto &id = hyp.class_id;
            const auto &score = hyp.score;

            if (score < gate_conf_thresh)
                continue;

            if (object == "GATE" && 
                (id == "left_gate_pole" || id == "right_gate_pole" || 
                 id == "preq_gate" || id == "search&rescue" || id == "preq_pole"))
                return true;

            if (object == "shark" && id == "shark")
                return true;
        }
        return false;
    }

    bool getObjectPosition(const std::string &object, double &ox, double &oy,
                           double &oz, double *score_out = nullptr) {
        std::lock_guard<std::mutex> g(mtx);
        if (!latest_detections)
            return false;

        if (object == "GATE") {
            bool has_left = false;
            bool has_right = false;
            double lx = 0.0, ly = 0.0, lz = 0.0, l_score = 0.0;
            double rx = 0.0, ry = 0.0, rz = 0.0, r_score = 0.0;
            double fallback_x = 0.0, fallback_y = 0.0, fallback_z = 0.0, fallback_score = 0.0;
            bool has_fallback = false;

            for (const auto &det : latest_detections->detections) {
                if (det.results.empty())
                    continue;
                const auto &hyp = det.results[0].hypothesis;
                const auto &id = hyp.class_id;
                const auto &score = hyp.score;

                if (score < gate_conf_thresh)
                    continue;

                if (id == "left_gate_pole") {
                    has_left = true;
                    lx = det.bbox.center.position.x;
                    ly = det.bbox.center.position.y;
                    lz = det.bbox.center.position.z;
                    l_score = score;
                } else if (id == "right_gate_pole") {
                    has_right = true;
                    rx = det.bbox.center.position.x;
                    ry = det.bbox.center.position.y;
                    rz = det.bbox.center.position.z;
                    r_score = score;
                } else if (id == "preq_gate" || id == "search&rescue" || id == "preq_pole") {
                    has_fallback = true;
                    fallback_x = det.bbox.center.position.x;
                    fallback_y = det.bbox.center.position.y;
                    fallback_z = det.bbox.center.position.z;
                    fallback_score = score;
                }
            }

            if (has_left && has_right) {
                ox = (lx + rx) / 2.0;
                oy = (ly + ry) / 2.0;
                oz = (lz + rz) / 2.0;
                if (score_out)
                    *score_out = (l_score + r_score) / 2.0;
                return true;
            } else if (has_left) {
                ox = lx + 0.75;
                oy = ly;
                oz = lz;
                if (score_out)
                    *score_out = l_score;
                return true;
            } else if (has_right) {
                ox = rx - 0.75;
                oy = ry;
                oz = rz;
                if (score_out)
                    *score_out = r_score;
                return true;
            } else if (has_fallback) {
                ox = fallback_x;
                oy = fallback_y;
                oz = fallback_z;
                if (score_out)
                    *score_out = fallback_score;
                return true;
            }
        } else if (object == "shark") {
            for (const auto &det : latest_detections->detections) {
                if (det.results.empty())
                    continue;
                const auto &hyp = det.results[0].hypothesis;
                const auto &id = hyp.class_id;
                const auto &score = hyp.score;

                if (id == "shark" && score >= gate_conf_thresh) {
                    ox = det.bbox.center.position.x;
                    oy = det.bbox.center.position.y;
                    oz = det.bbox.center.position.z;
                    if (score_out)
                        *score_out = score;
                    return true;
                }
            }
        }
        return false;
    }

    void publishToPico(float delta_theta, float delta_distance,
                                         float target_depth_val, uint8_t stop_thrusters) {
        auv_msgs::msg::ControlCommand msg;
        msg.delta_theta = delta_theta;
        msg.delta_distance = delta_distance;
        msg.target_depth = target_depth_val;
        msg.stop_thrusters = stop_thrusters;
        pico_pub->publish(msg);
    }

    void stopMotion() {
        publishToPico(0.0f, 0.0f, target_depth, 1);
    }
};

// --- Math Utilities --------------------------------------------------------

inline double clampVal(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

inline double normalizeAngle(double a) {
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a < -M_PI)
        a += 2.0 * M_PI;
    return a;
}

// --- Condition Nodes -------------------------------------------------------

class AllSystemsOK : public BT::StatefulActionNode {
public:
    AllSystemsOK(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("timeout_s", 20.0,
                                                                 "Seconds to wait before giving up")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::chrono::steady_clock::time_point start_time_;
    double timeout_s_ = 20.0;
};

class IsObjectSeen : public BT::ConditionNode {
public:
    IsObjectSeen(const std::string &name, const BT::NodeConfig &config)
            : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("object")};
    }
    BT::NodeStatus tick() override;
};

// --- Action Nodes ----------------------------------------------------------

class DiveToDepth : public BT::StatefulActionNode {
public:
    DiveToDepth(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("target_depth")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    double target_z_ = 0.0;
};

class Do360Turn : public BT::StatefulActionNode {
public:
    Do360Turn(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("success_when_seen")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string target_object_;
    double prev_yaw_ = 0.0;
    double accumulated_yaw_ = 0.0;
    int confirm_frames_ = 0;
};

class DriveThruGate : public BT::StatefulActionNode {
public:
    DriveThruGate(const std::string &name, const BT::NodeConfig &config)
            : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    enum class Phase { ALIGN_1, APPROACH, ALIGN_2, SURGE };
    Phase phase_ = Phase::ALIGN_1;
    int gate_lost_frames_ = 0;
    int align_confirm_frames_ = 0;
    double locked_yaw_ = 0.0;
    double filtered_yaw_err_ = 0.0;
};

inline void registerAllNodes(BT::BehaviorTreeFactory &factory) {
    factory.registerNodeType<AllSystemsOK>("AllSystemsOK");
    factory.registerNodeType<DiveToDepth>("DiveToDepth");
    factory.registerNodeType<IsObjectSeen>("IsObjectSeen");
    factory.registerNodeType<Do360Turn>("Do360Turn");
    factory.registerNodeType<DriveThruGate>("DriveThruGate");
}
// Behavior tree node declarations and RobotContext will go here
