#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "pd/pd.cpp"

using namespace std::chrono_literals;

class TurtleFollowerNode : public rclcpp::Node {
public:
  TurtleFollowerNode() : Node("turtle_follower_node") {
    using namespace std::placeholders;

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    params = initialize_params();
    x_controller = PD(params);
    z_controller = PD(params);

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    updateTimer =
        this->create_wall_timer(10ms, std::bind(
            &TurtleFollowerNode::timer_callback, this));
  }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    geometry_msgs::msg::Twist vel_msg;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    PDParameters params;
    PD x_controller{PD::defaultParams()};
    PD z_controller{PD::defaultParams()};

    double dt{0.01};
    double x_e{0}, y_e{0}, ang_e{0};

    void timer_callback() {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                "base_link", "turtle",tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                "base_link", "turtle", ex.what());
            return;
        }

        x_e = transform.transform.translation.x;
        y_e = transform.transform.translation.y;
        ang_e = atan2(y_e, x_e);

        RCLCPP_INFO(
            this->get_logger(), "Errors: %f, %f, %f",
            x_e, y_e, ang_e);

        vel_msg.linear.x = -x_controller.update(x_e, 0.);
        vel_msg.angular.z = -z_controller.update(ang_e, 0.);
        vel_pub_->publish(vel_msg);
    }

    PDParameters initialize_params() {
        this->declare_parameter("Kp", 1.0);
        this->declare_parameter("Kd", 0.2);
        this->declare_parameter("U_MAX", 1.0);
        this->declare_parameter("U_MIN", -1.0);
        this->declare_parameter("enable_ramp", false);
        this->declare_parameter("ramp_rate", 0.3);

        PDParameters p;
        p.kP = this->get_parameter("Kp").as_double();
        p.kD = this->get_parameter("Kd").as_double();
        p.kDt = dt;
        p.kUMax = this->get_parameter("U_MAX").as_double();
        p.kUMin = this->get_parameter("U_MIN").as_double();
        p.enable_ramp_rate_limit = this->get_parameter("enable_ramp").as_bool();
        p.ramp_rate = this->get_parameter("ramp_rate").as_double();
        return p;
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleFollowerNode>());
  rclcpp::shutdown();
  return 0;
}