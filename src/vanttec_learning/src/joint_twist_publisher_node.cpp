#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointTwistPublisherNode : public rclcpp::Node {
 public:
  JointTwistPublisherNode() : Node("joint_twist_publisher_node") {
    using namespace std::placeholders;

    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist &msg) { 
            /*
            vx = (v_l + v_r) * r / 2
            vz = (v_l - v_r) * r / (2*l)
            (vl + vr) = 2*vx/r
            (vl - vr) = 2*l*vz/r
            2vl = 2*l*vz/r + 2*vx/r
            vl = l*vz/r + vx/r = (l*vz + vx)/r
            vr = vl - 2*l*vz/r
            */
            double vx = msg.linear.x;
            double vz = -msg.angular.z;
            l_vel = (l*vz + vx)/r;
            r_vel = l_vel - 2*l*vz/r;
            });

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/j_s", 10);

    joint_msg.header.frame_id="base_link";
    joint_msg.name = std::vector<std::string>{"base_to_lwheel", "base_to_rwheel"};

    updateTimer =
        this->create_wall_timer(10ms, std::bind(&JointTwistPublisherNode::timer_callback, this));
  }

 private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    sensor_msgs::msg::JointState joint_msg;

    rclcpp::TimerBase::SharedPtr updateTimer;

    double l_wheel{0.}, r_wheel{0.};
    double l_vel{0.}, r_vel{0.};
    double r{0.046}; // wheel radius
    double l{0.23}; // wheel separation left to right

    double dt{0.01};

    void timer_callback() {
        l_wheel+=dt*l_vel;
        r_wheel+=dt*r_vel;

        l_wheel = std::fmod((l_wheel + M_PI), 2*M_PI) - M_PI;
        r_wheel = std::fmod((r_wheel + M_PI), 2*M_PI) - M_PI;

        joint_msg.header.stamp = JointTwistPublisherNode::now();
        joint_msg.position = std::vector<double>{-l_wheel, r_wheel};
        joint_pub_->publish(joint_msg);
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTwistPublisherNode>());
  rclcpp::shutdown();
  return 0;
}