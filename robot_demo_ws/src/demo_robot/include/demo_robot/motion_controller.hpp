#ifndef MOTION_CONTROLLER_CPP__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_CPP__MOTION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "demo_interface/msg/motion_command.hpp"
#include "demo_interface/msg/robot_status.hpp"

namespace motion_controller_cpp
{

    class MotionController : public rclcpp::Node
    {

    public:
        explicit MotionController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        virtual ~MotionController() = default;

    private:
        void motionCommandCallback(const demo_interface::msg::MotionCommand::SharedPtr msg);
        void publishRobotStatus();
        void controlLoop();

        void executeVelocityControl(const geometry_msgs::msg::Vector3 &linear_vel,
                                    const geometry_msgs::msg::Vector3 &angular_val);

        void executePositionControl(const geometry_msgs::msg::Point &target_pos,
                                    const geometry_msgs::msg::Quaternion &target_orient);

        void executeStop();

        void updateRobotPose(double dt);

        void updateBatteryLevel(double dt);

        bool checkSafetyLimits();

        bool isCommandTimeout();

        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters);

        rclcpp::Subscription<demo_interface::msg::MotionCommand>::SharedPtr motion_subscriber_;
        rclcpp::Publisher<demo_interface::msg::RobotStatus>::SharedPtr status_publisher_;
        rclcpp::TimerBase::SharedPtr status_timer_;
        rclcpp::TimerBase::SharedPtr control_timer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        std::string robot_id_;
        double max_linear_velocity_;
        double max_angular_velocity_;
        double max_linear_acceleration_;
        double max_angular_acceleration_;
        double command_timeout_;
        double control_frequency_;
        double status_publish_rate_;

        geometry_msgs::msg::Point current_position_;
        geometry_msgs::msg::Quaternion current_orientation_;
        geometry_msgs::msg::Vector3 current_linear_velocity_;
        geometry_msgs::msg::Vector3 current_angular_velocity_;
        float battery_level_;
        std::string current_mode_;
        uint8_t robot_status_;
        bool is_charging_;
        uint32_t error_code_;
        std::string status_message_;

        // 控制状态
        demo_interface::msg::MotionCommand::SharedPtr current_command_;
        std::chrono::steady_clock::time_point last_command_time_;
        std::chrono::steady_clock::time_point start_time_;

        // 目标状态 (用于位置控制)
        geometry_msgs::msg::Point target_position_;
        geometry_msgs::msg::Quaternion target_orientation_;
        bool position_control_active_;

        // 线程安全
        std::mutex state_mutex_;

        // PID控制器参数 (简化实现)
        struct PIDGains
        {
            double kp, ki, kd;
        };
        PIDGains position_pid_gains_;
        PIDGains orientation_pid_gains_;

        // PID控制器状态
        geometry_msgs::msg::Vector3 position_error_integral_;
        geometry_msgs::msg::Vector3 position_error_previous_;
        geometry_msgs::msg::Vector3 orientation_error_integral_;
        geometry_msgs::msg::Vector3 orientation_error_previous_;
    };

}

#endif // MOTION_CONTROLLER_CPP__MOTION_CONTROLLER_HPP_