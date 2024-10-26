#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class JointStatePublisher : public rclcpp::Node 
{
public:
	JointStatePublisher() : Node("joint_state_publisher")
    {
        // Parámetros del robot
        radio_wheels_ = 0.046;   // Radio de las llantas en metros
        dist_wheels_ = 0.16;     // Distancia entre las llantas en metros
        publish_frequency = 1.0; // Frecuencia para publicar los mensajes

        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&JointStatePublisher::callbackPublisherPosition, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds((int) (1000.0 / publish_frequency)),
                                        std::bind(&JointStatePublisher::publish_joint_state, this));

//        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
//                                         std::bind(&JointStatePublisher::publish_joint_state, this));

        RCLCPP_INFO(this->get_logger(), "Joint State Publisher Node has been started.");
	}
    
private:

    void callbackPublisherPosition(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Extraer velocidades lineales y angulares
        double vel_linear = msg->linear.x;
        double vel_angular = msg->angular.z;

        // Calcular velocidades de las llantas
        double vel_ang_der = (vel_linear + (vel_angular * dist_wheels_ / 2)) / radio_wheels_;
        double vel_ang_izq = (vel_linear - (vel_angular * dist_wheels_ / 2)) / radio_wheels_;

        
    }


    void publish_joint_state() {

    }

    double publish_frequency;
    double radio_wheels_;
    double dist_wheels_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

};
    
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<JointStatePublisher>(); // MODIFY NAME
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
