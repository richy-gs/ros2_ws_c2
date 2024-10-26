#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DifferentialDriveNode : public rclcpp::Node
{
public:
    DifferentialDriveNode()
    : Node("differential_drive_node"),
      wheel_radius(0.046),     // Radio de la llanta en metros
      wheel_base(0.16)        // Distancia entre las llantas en metros
    {
        // Suscribirse al tópico /cmd_vel
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DifferentialDriveNode::cmdVelCallback, this, std::placeholders::_1)
        );

        // Publicar en el tópico /joint_states
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10
        );

        // Crear timer para publicar mensajes
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DifferentialDriveNode::publishJointStates, this)
        );
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Guardar las velocidades deseadas
        linear_x = msg->linear.x;
        angular_z = msg->angular.z;
    }

    void publishJointStates()
    {
        sensor_msgs::msg::JointState joint_state_msg;

        // Calcular las velocidades de las llantas
        double vel_ang_izq = (linear_x - (angular_z * wheel_base / 2)) / wheel_radius;
        double vel_ang_der = (linear_x + (angular_z * wheel_base / 2)) / wheel_radius;

        // Llenar el mensaje de JointState
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.name.push_back("base_to_lwheel");
        joint_state_msg.name.push_back("base_to_rwheel");
        joint_state_msg.position.push_back(0.0); // Inicialmente en 0
        joint_state_msg.position.push_back(0.0); // Inicialmente en 0
        joint_state_msg.velocity.push_back(vel_ang_izq);
        joint_state_msg.velocity.push_back(vel_ang_der);
        joint_state_msg.effort.push_back(0.0); // Por simplicidad, no se usa esfuerzo
        joint_state_msg.effort.push_back(0.0);

        // Publicar el mensaje
        joint_state_publisher_->publish(joint_state_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double linear_x = 0.0;
    double angular_z = 0.0;

    const double wheel_radius;
    const double wheel_base;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialDriveNode>());
    rclcpp::shutdown();
    return 0;
}
