#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>

using namespace std;

class TopicSubscribeNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;

    function<void(const std_msgs::msg::String::SharedPtr msgs)> Callback = [this](const std_msgs::msg::String::SharedPtr msgs){
        RCLCPP_INFO(this->get_logger(), "Subscribing: %s", msgs->data.c_str());
    };

public:
    TopicSubscribeNode(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已启动", name.c_str());
        command_subscriber_ = this->create_subscription<std_msgs::msg::String>("Command", 10, Callback);
        //timer_ = this->create_wall_timer(chrono::milliseconds(500), Callback);
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TopicSubscribeNode>("TopicSubScriber");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
