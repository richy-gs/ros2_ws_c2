#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
	AddTwoIntsServerNode() : Node("add_two_ints_server")
	{
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints", 
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Service serve has been started.");
	}
    
private:
    // Create a callback function
    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b; //Automaticly send to client
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", request->a, request->b, response->sum);
    }

    // Create a service
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};
    
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<AddTwoIntsServerNode>(); // MODIFY NAME
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
