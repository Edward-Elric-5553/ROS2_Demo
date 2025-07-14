#include <rclcpp/rclcpp.hpp>
#include <demo_interface/msg/robot_status.hpp>
#include <demo_interface/srv/move_robot.hpp>
#include <chrono>

using namespace std::chrono_literals;

class RobotNode
{
private:
    float current_pose_ = 0.0;
    float target_pose_ = 0.0;
    int status_ = demo_interface::msg::RobotStatus::STATUS_STOP;

public:
    RobotNode() = default;
    ~RobotNode() = default;

    float move_distance(float distance)
    {
        // start move
        status_ = demo_interface::msg::RobotStatus::STATUS_MOVEING;
        target_pose_ += distance;

        while (fabs(target_pose_ - current_pose_) > 0.01)
        {
            float step = distance / fabs(distance) * fabs(target_pose_ - current_pose_) * 0.1;
            current_pose_ += step;
            std::cout << "move : " << step << " current pose: " << current_pose_ << std::endl;
            std::this_thread::sleep_for(0.5s);
        }

        // stop move
        status_ = demo_interface::msg::RobotStatus::STATUS_STOP;
        return current_pose_;
    }

    float get_current_pose()
    {
        return current_pose_;
    }

    int get_status()
    {
        return status_;
    }
};

class DemoRobotNode : public rclcpp::Node
{
private:
    RobotNode robot;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<demo_interface::srv::MoveRobot>::SharedPtr move_robot_server_;
    rclcpp::Publisher<demo_interface::msg::RobotStatus>::SharedPtr robot_status_publisher_;

    void timer_callback()
    {
        demo_interface::msg::RobotStatus message;
        message.status = robot.get_status();
        message.pose = robot.get_current_pose();

        RCLCPP_INFO(this->get_logger(), "Publishing: %f ", robot.get_current_pose());

        robot_status_publisher_->publish(message);
    }

    void handle_move_robot(const std::shared_ptr<demo_interface::srv::MoveRobot::Request> request, 
                           std::shared_ptr<demo_interface::srv::MoveRobot::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Request Distance: %f, Current Pose: %f", request->distance, response->pose);
        robot.move_distance(request->distance);
        response->pose = robot.get_current_pose();
    }

public:
    DemoRobotNode(const std::string &node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s Node start", node_name.c_str());
        move_robot_server_ = this->create_service<demo_interface::srv::MoveRobot>(
            "move_robot", 
            std::bind(&DemoRobotNode::handle_move_robot, this, std::placeholders::_1, std::placeholders::_2)
        );

        robot_status_publisher_ = this->create_publisher<demo_interface::msg::RobotStatus>(
            "robot_status", 
            10
        );

        timer_ = this->create_wall_timer(0.5s, std::bind(&DemoRobotNode::timer_callback, this));
    }
};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<DemoRobotNode>("Demo_Robot_Node");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}