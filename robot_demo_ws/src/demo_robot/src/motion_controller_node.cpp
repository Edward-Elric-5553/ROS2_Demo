#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "demo_robot/motion_controller.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<motion_controller_cpp::MotionController>();
    RCLCPP_INFO(node->get_logger(), "运动控制器节点已启动");
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("motion_controller_node"), 
                 "节点启动失败: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}