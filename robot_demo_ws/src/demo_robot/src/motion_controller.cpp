#include "demo_robot/motion_controller.hpp"

#include <cmath>
#include <functional>

namespace motion_controller_cpp
{

    MotionController::MotionController(const rclcpp::NodeOptions & options)
        : Node("motion_controller", options),
          battery_level_(100.0f),
          current_mode_("idle"),
          robot_status_(demo_interface::msg::RobotStatus::STATUS_IDLE),
          is_charging_(false),
          error_code_(0),
          status_message_("System is valid"),
          start_time_(std::chrono::steady_clock::now())
    {

        this->declare_parameter("robot_id", "robot_001");
        this->declare_parameter("max_linear_velocity", 2.0);
        this->declare_parameter("max_angular_velocity", 1.0);
        this->declare_parameter("max_linear_acceleration", 1.0);
        this->declare_parameter("max_angular_acceleration", 0.5);
        this->declare_parameter("command_timeout", 1.0);
        this->declare_parameter("control_frequency", 50.0);
        this->declare_parameter("status_publish_rate", 10.0);

        this->declare_parameter("position_kp", 1.0);
        this->declare_parameter("position_ki", 0.1);
        this->declare_parameter("position_kd", 0.05);
        this->declare_parameter("orientation_kp", 2.0);
        this->declare_parameter("orientation_ki", 0.2);
        this->declare_parameter("orientation_kd", 0.1);

        robot_id_ = this->get_parameter("robot_id").as_string();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        max_linear_acceleration_ = this->get_parameter("max_linear_acceleration").as_double();
        max_angular_acceleration_ = this->get_parameter("max_angular_acceleration").as_double();
        command_timeout_ = this->get_parameter("command_timeout").as_double();
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        status_publish_rate_ = this->get_parameter("status_publish_rate").as_double();

        position_pid_gains_.kp = this->get_parameter("position_kp").as_double();
        position_pid_gains_.ki = this->get_parameter("position_ki").as_double();
        position_pid_gains_.kd = this->get_parameter("position_kd").as_double();
        orientation_pid_gains_.kp = this->get_parameter("orientation_kp").as_double();
        orientation_pid_gains_.ki = this->get_parameter("orientation_ki").as_double();
        orientation_pid_gains_.kd = this->get_parameter("orientation_kd").as_double();

        current_position_.x = 0.0;
        current_position_.y = 0.0;
        current_position_.z = 0.0;
        current_orientation_.x = 0.0;
        current_orientation_.y = 0.0;
        current_orientation_.z = 0.0;
        current_orientation_.w = 1.0;

        // 创建订阅器和发布器
        motion_subscriber_ = this->create_subscription<demo_interface::msg::MotionCommand>(
            "motion_cmd", 10, std::bind(&MotionController::motionCommandCallback, this, std::placeholders::_1));

        status_publisher_ = this->create_publisher<demo_interface::msg::RobotStatus>(
            "robot_status", 10);

        // 创建定时器
        auto status_period = std::chrono::milliseconds(static_cast<int>(1000.0 / status_publish_rate_));
        status_timer_ = this->create_wall_timer(
            status_period, std::bind(&MotionController::publishRobotStatus, this));

        auto control_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_));
        control_timer_ = this->create_wall_timer(
            control_period, std::bind(&MotionController::controlLoop, this));

        // 设置参数回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MotionController::parametersCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "运动控制器已启动，机器人ID: %s", robot_id_.c_str());
    }

    void MotionController::motionCommandCallback(const demo_interface::msg::MotionCommand::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 检查紧急停止
        if (msg->emergency_stop)
        {
            RCLCPP_WARN(this->get_logger(), "收到紧急停止命令");
            executeStop();
            return;
        }

        // 更新当前命令和时间戳
        current_command_ = msg;
        last_command_time_ = std::chrono::steady_clock::now();

        // 根据控制模式执行相应操作
        switch (msg->control_mode)
        {
        case demo_interface::msg::MotionCommand::MODE_VELOCITY:
            current_mode_ = "velocity_control";
            robot_status_ = demo_interface::msg::RobotStatus::STATUS_MOVING;
            RCLCPP_DEBUG(this->get_logger(), "切换到速度控制模式");
            break;

        case demo_interface::msg::MotionCommand::MODE_POSITION:
            current_mode_ = "position_control";
            robot_status_ = demo_interface::msg::RobotStatus::STATUS_MOVING;
            target_position_ = msg->target_position;
            target_orientation_ = msg->target_orientation;
            position_control_active_ = true;
            RCLCPP_DEBUG(this->get_logger(), "切换到位置控制模式");
            break;

        case demo_interface::msg::MotionCommand::MODE_STOP:
            executeStop();
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "未知的控制模式: %d", msg->control_mode);
            break;
        }
    }

    void MotionController::controlLoop()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        double dt = 1.0 / control_frequency_;

        // 检查命令超时
        if (isCommandTimeout())
        {
            executeStop();
        }

        // 检查安全限制
        if (!checkSafetyLimits())
        {
            executeStop();
            robot_status_ = demo_interface::msg::RobotStatus::STATUS_ERROR;
            error_code_ = 1001;
            status_message_ = "安全限制触发，机器人已停止";
            return;
        }

        // 执行控制逻辑
        if (current_command_)
        {
            switch (current_command_->control_mode)
            {
            case demo_interface::msg::MotionCommand::MODE_VELOCITY:
                executeVelocityControl(current_command_->linear_velocity, current_command_->angular_velocity);
                break;

            case demo_interface::msg::MotionCommand::MODE_POSITION:
                executePositionControl(target_position_, target_orientation_);
                break;
            }
        }

        // 更新机器人状态
        updateRobotPose(dt);
        updateBatteryLevel(dt);
    }

    void MotionController::executeVelocityControl(const geometry_msgs::msg::Vector3 &linear_vel,
                                                  const geometry_msgs::msg::Vector3 &angular_vel)
    {
        // 限制速度在安全范围内
        current_linear_velocity_.x = std::clamp(linear_vel.x, -max_linear_velocity_, max_linear_velocity_);
        current_linear_velocity_.y = std::clamp(linear_vel.y, -max_linear_velocity_, max_linear_velocity_);
        current_linear_velocity_.z = std::clamp(linear_vel.z, -max_linear_velocity_, max_linear_velocity_);

        current_angular_velocity_.x = std::clamp(angular_vel.x, -max_angular_velocity_, max_angular_velocity_);
        current_angular_velocity_.y = std::clamp(angular_vel.y, -max_angular_velocity_, max_angular_velocity_);
        current_angular_velocity_.z = std::clamp(angular_vel.z, -max_angular_velocity_, max_angular_velocity_);
    }

    void MotionController::executePositionControl(const geometry_msgs::msg::Point &target_pos,
                                                  const geometry_msgs::msg::Quaternion &target_orient)
    {

        (void)target_orient;
        // 简化的PID位置控制
        double position_error_x = target_pos.x - current_position_.x;
        double position_error_y = target_pos.y - current_position_.y;
        double position_error_z = target_pos.z - current_position_.z;

        // PID控制计算 (简化版本)
        current_linear_velocity_.x = position_pid_gains_.kp * position_error_x;
        current_linear_velocity_.y = position_pid_gains_.kp * position_error_y;
        current_linear_velocity_.z = position_pid_gains_.kp * position_error_z;

        // 限制速度
        current_linear_velocity_.x = std::clamp(current_linear_velocity_.x, -max_linear_velocity_, max_linear_velocity_);
        current_linear_velocity_.y = std::clamp(current_linear_velocity_.y, -max_linear_velocity_, max_linear_velocity_);
        current_linear_velocity_.z = std::clamp(current_linear_velocity_.z, -max_linear_velocity_, max_linear_velocity_);

        // 检查是否到达目标
        double distance_to_target = std::sqrt(position_error_x * position_error_x +
                                              position_error_y * position_error_y +
                                              position_error_z * position_error_z);

        if (distance_to_target < 0.1)
        { // 10cm容差
            position_control_active_ = false;
            current_mode_ = "idle";
            robot_status_ = demo_interface::msg::RobotStatus::STATUS_IDLE;
            RCLCPP_INFO(this->get_logger(), "已到达目标位置");
        }
    }

    void MotionController::executeStop()
    {
        current_linear_velocity_.x = 0.0;
        current_linear_velocity_.y = 0.0;
        current_linear_velocity_.z = 0.0;
        current_angular_velocity_.x = 0.0;
        current_angular_velocity_.y = 0.0;
        current_angular_velocity_.z = 0.0;

        current_mode_ = "idle";
        robot_status_ = demo_interface::msg::RobotStatus::STATUS_IDLE;
        position_control_active_ = false;
        current_command_.reset();
    }

    void MotionController::updateRobotPose(double dt)
    {
        // 简单的运动学积分
        current_position_.x += current_linear_velocity_.x * dt;
        current_position_.y += current_linear_velocity_.y * dt;
        current_position_.z += current_linear_velocity_.z * dt;
    }

    void MotionController::updateBatteryLevel(double dt)
    {
        // 简单的电池消耗模型
        double velocity_magnitude = std::sqrt(
            current_linear_velocity_.x * current_linear_velocity_.x +
            current_linear_velocity_.y * current_linear_velocity_.y +
            current_linear_velocity_.z * current_linear_velocity_.z);

        double consumption_rate = 0.1 + velocity_magnitude * 0.5;             // 基础消耗 + 运动消耗
        battery_level_ -= static_cast<float>(consumption_rate * dt / 3600.0); // 每小时消耗

        if (battery_level_ < 0.0f)
        {
            battery_level_ = 0.0f;
        }

        // 低电量警告
        if (battery_level_ < 20.0f && robot_status_ != demo_interface::msg::RobotStatus::STATUS_ERROR)
        {
            status_message_ = "电池电量低，请及时充电";
        }
    }

    bool MotionController::checkSafetyLimits()
    {
        // 检查位置限制 (简单的边界检查)
        const double max_x = 10.0, max_y = 10.0, max_z = 2.0;

        if (std::abs(current_position_.x) > max_x ||
            std::abs(current_position_.y) > max_y ||
            current_position_.z < 0.0 || current_position_.z > max_z)
        {
            return false;
        }

        return true;
    }

    bool MotionController::isCommandTimeout()
    {
        if (!current_command_)
        {
            return false;
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - last_command_time_).count();

        return elapsed > command_timeout_;
    }

    void MotionController::publishRobotStatus()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        auto message = demo_interface::msg::RobotStatus();

        message.timestamp = this->get_clock()->now();
        message.robot_id = robot_id_;
        message.position = current_position_;
        message.orientation = current_orientation_;
        message.linear_velocity = current_linear_velocity_;
        message.angular_velocity = current_angular_velocity_;
        message.battery_level = battery_level_;
        message.mode = current_mode_;
        message.status = robot_status_;
        message.is_charging = is_charging_;
        message.error_code = error_code_;
        message.status_message = status_message_;

        status_publisher_->publish(message);
    }

    rcl_interfaces::msg::SetParametersResult MotionController::parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters)
        {
            if (param.get_name() == "max_linear_velocity")
            {
                max_linear_velocity_ = param.as_double();
            }
            else if (param.get_name() == "max_angular_velocity")
            {
                max_angular_velocity_ = param.as_double();
            }
            // 其他参数处理...
        }

        return result;
    }

}