#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsStationNode : public rclcpp::Node
{
public:
	RobotNewsStationNode() : Node("robot_news_station"), robot_name_("R2D2")
	{
        this->declare_parameter("robot_name", "R2D2");
        robot_name_ = this->get_parameter("robot_name").as_string();

        // Inicialize the publisher
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        // Inicializar timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                                         std::bind(&RobotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started.");
	}
        
private:
    // Create a function
    void publishNews(){
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the Robot News Station");
        publisher_->publish(msg);
    }

    std::string robot_name_;
    // Declare the publisher
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
    
int main(int argc, char **argv)
{
    // ROS communication is inicialize
	rclcpp::init(argc, argv);
    // Create a Node
	auto node = std::make_shared<RobotNewsStationNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
