#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class RobotStatePublisherNode : public rclcpp::Node {
public:
  RobotStatePublisherNode() : Node("robot_state_publisher_node") {
    using namespace std::placeholders;

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
            vx = msg.linear.x;
            vz = msg.angular.z;
            l_vel = (-l*vz + vx)/r;
            r_vel = l_vel + 2*l*vz/r;
            });

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/j_s", 10);

    joint_msg.header.frame_id="base_link";
    joint_msg.name = std::vector<std::string>{"base_to_lwheel", "base_to_rwheel"};

    updateTimer =
        this->create_wall_timer(10ms, std::bind(&RobotStatePublisherNode::timer_callback, this));
  }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    sensor_msgs::msg::JointState joint_msg;

    rclcpp::TimerBase::SharedPtr updateTimer;

    double l_wheel{0.}, r_wheel{0.};
    double l_vel{0.}, r_vel{0.};
    double vx{0.}, vz{0.};
    double pose[3]{0., 0., 0.};
    double r{0.046}; // wheel radius
    double l{0.23}; // wheel separation left to right

    double dt{0.01};

    void timer_callback() {
        l_wheel+=l_vel*dt;
        r_wheel+=r_vel*dt;

        l_wheel = std::fmod((l_wheel + M_PI), 2*M_PI) - M_PI;
        r_wheel = std::fmod((r_wheel + M_PI), 2*M_PI) - M_PI;

        joint_msg.header.stamp = RobotStatePublisherNode::now();
        joint_msg.position = std::vector<double>{-l_wheel, r_wheel};
        joint_pub_->publish(joint_msg);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "base_link";

        pose[0] += vx*cos(pose[2])*dt;
        pose[1] += vx*sin(pose[2])*dt;
        pose[2] += vz*dt;
        pose[2] = std::fmod((pose[2] + M_PI), 2*M_PI) - M_PI;

        t.transform.translation.x = pose[0];
        t.transform.translation.y = pose[1];
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, pose[2]);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotStatePublisherNode>());
  rclcpp::shutdown();
  return 0;
}