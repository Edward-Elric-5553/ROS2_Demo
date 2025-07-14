#include <rclcpp/rclcpp.hpp>

using namespace std;

class ServiceServer : public rclcpp::Node
{
private:
public:
    ServiceServer(const string &node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s node start", node_name.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = make_shared<ServiceServer>("Demo_Server");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}