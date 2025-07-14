#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <chrono>

using namespace std;
using namespace std::chrono_literals;

class ServiceClient : public rclcpp::Node
{
private:
    // 创建 <example_interfaces::srv::AddTwoInts> 模板接口的共享指针
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_int_client_;

public:
    ServiceClient(const string &node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s node start", node_name.c_str());

        //创建 client service 实例
        add_int_client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints_srv");
    }

    void send_request(int a, int b){
        RCLCPP_INFO(this->get_logger(), "calculate %d + %d", a, b);

        // 等待 server service 启动
        while(!add_int_client_->wait_for_service(1s)){
            if (!rclcpp::ok()){
                RCLCPP_INFO(this->get_logger(), "waiting service be interrupted .......");
                return ;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for service start ......");
        }

        // 创建 request 消息实例
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
        request->a = a;
        request->b = b;

        // 异步发送请求， 并获取返回
        add_int_client_->async_send_request(request, [this](
            rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_feaure
        )->void{
            auto response = result_feaure.get();
            RCLCPP_INFO(this->get_logger(), "calculate result: %ld", response->sum);
        });
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 创建 service client 节点
    auto node = std::make_shared<ServiceClient>("Demo_Client");

    // 发送 client 请求
    node->send_request(5, 6);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}