#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/move_robot.hpp>
#include <demo_interface/msg/robot_status.hpp>

using namespace std::chrono_literals;

class DemoControlNode : public rclcpp::Node
{
private:
    rclcpp::Client<demo_interface::srv::MoveRobot>::SharedPtr client_;
    rclcpp::Subscription<demo_interface::msg::RobotStatus>::SharedPtr robot_status_subscriber_;

    void result_callback_(
        rclcpp::Client<demo_interface::srv::MoveRobot>::SharedFuture result_feature
    ){
        auto response = result_feature.get();
        RCLCPP_INFO(this->get_logger(), "get result: %f", response->pose);
    }

    void robot_status_callback_(
        const demo_interface::msg::RobotStatus::SharedPtr msg
    ){
        RCLCPP_INFO(this->get_logger(), "get pose status: %f, status: %d", msg->pose, msg->status);
    }

public:
    DemoControlNode(const std::string &node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s Node start", node_name.c_str());
        client_ = this->create_client<demo_interface::srv::MoveRobot>(
            "move_robot"
        );

        robot_status_subscriber_ = this->create_subscription<demo_interface::msg::RobotStatus>(
            "robot_status", 
            10, 
            std::bind(&DemoControlNode::robot_status_callback_, this, std::placeholders::_1)
        );
    }

    void move_robot(float distance){
        RCLCPP_INFO(this->get_logger(), "Request distance: %f", distance);

        while(!client_->wait_for_service(1s)){
            if (!rclcpp::ok()){
                RCLCPP_INFO(this->get_logger(), "waiting process be interrupted...");
                return ;
            }
            RCLCPP_INFO(this->get_logger(), "waiting service start...");
        }

        auto request = std::make_shared<demo_interface::srv::MoveRobot::Request>();
        request->distance = distance;

        client_->async_send_request(request, std::bind(&DemoControlNode::result_callback_, this, std::placeholders::_1));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DemoControlNode>("Demo_Control_Node");

    node->move_robot(5.0);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}