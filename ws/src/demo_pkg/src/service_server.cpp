#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using namespace std;

class ServiceServer : public rclcpp::Node
{
private:
    // 创建 <example_interfaces::srv::AddTwoInts> 模板接口的共享指针
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr add_int_server_;

public:
    ServiceServer(const string &node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s node start", node_name.c_str());

        // 创建 <example_interfaces::srv::AddTwoInts> 模板的service 实例
        // 使用 Lambda函数 创建回调函数
        add_int_server_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints_srv", [this](
            const shared_ptr<example_interfaces::srv::AddTwoInts::Request> request, 
            shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
        )->void{
            response->sum = request->a + request->b;
        });
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    //创建 Service Server节点
    auto node = make_shared<ServiceServer>("Demo_Server");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}