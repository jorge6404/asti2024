#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/set_velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

rclcpp::WallRate loop_rate(100us);


class VelController : public rclcpp::Node {
public:
    VelController()
    : Node("vel_controller") { // Defines a /set_velocity publisher associated to a timer
        vel_publisher_ = this->create_publisher
            <custom_interfaces::msg::SetVelocity>("set_velocity", 10);
        vel_subscriber_ = this->create_subscription
            <geometry_msgs::msg::Twist>("cmd_vel", 10, 
            std::bind(&VelController::vel_callback, this, std::placeholders::_1));
    }

private:
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        input_linear_vel_ = msg->linear.x;
        input_angular_vel_ = msg->angular.z;

        if (input_angular_vel_ == 0){
            output_vel_right = input_linear_vel_*unit_distance;
            output_vel_left = input_linear_vel_*unit_distance;
        }
        else{
            output_vel_right = (input_angular_vel_*(input_linear_vel_+0.086))*unit_distance;
            output_vel_left = ((input_angular_vel_*(input_linear_vel_-0.086)))*unit_distance;
        }

        msg_.id = 1;
        msg_.velocity = output_vel_right;
        vel_publisher_->publish(msg_);
        msg_.id = 2;
        msg_.velocity = output_vel_left;
        vel_publisher_->publish(msg_);
    }

    int output_vel=0,
        output_vel_right = 0, // ID 1
        output_vel_left = 0; // ID 2
    // Distance traveled in a motor unit: 
    const float unit_distance = 1223.3613;
    float input_linear_vel_ = 0,
          input_angular_vel_ = 0,
          angular_vel = 0;
    rclcpp::Publisher<custom_interfaces::msg::SetVelocity>::SharedPtr vel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
    custom_interfaces::msg::SetVelocity msg_;
};

int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<VelController>());
   rclcpp::shutdown();
   return 0;
}