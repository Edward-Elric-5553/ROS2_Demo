#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>

using namespace std::chrono_literals;

class DynamicTFBroadCaster : public rclcpp::Node{
    private:
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;

    public:
        DynamicTFBroadCaster():Node("dynamic_tf_broadcaster"){
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            timer_ = create_wall_timer(100ms, std::bind(&DynamicTFBroadCaster::publishTransform, this));
        }

        void publishTransform(){
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->get_clock()->now();
            transform.header.frame_id = "map";
            transform.child_frame_id = "base_link";
            transform.transform.translation.x = 2.0;
            transform.transform.translation.y = 3.0;
            transform.transform.translation.z = 4.0;
            tf2::Quaternion quat;
            quat.setRPY(0, 0, 30 * M_PI / 180.0);
            transform.transform.rotation = tf2::toMsg(quat);
            tf_broadcaster_->sendTransform(transform);
        }
};






int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DynamicTFBroadCaster>();

    rclcpp::spin(node);


    rclcpp::shutdown();
    return 0;
}