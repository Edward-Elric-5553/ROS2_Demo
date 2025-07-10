#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <functional>

using namespace std;

class TopicPublishNode : public rclcpp::Node
{
private:
    rclcpp::WallTimer<function<void()>>::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;

    function<void()> Callback = [this](){
        std_msgs::msg::String message;
        message.data = "One Message";
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
    };

public:
    TopicPublishNode(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已启动", name.c_str());
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("Command", 10);
        timer_ = this->create_wall_timer(chrono::milliseconds(500), Callback);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TopicPublishNode>("TopicPublisher");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}