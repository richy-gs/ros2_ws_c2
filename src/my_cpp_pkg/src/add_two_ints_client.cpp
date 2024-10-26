#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

/*
    Puede usar como template:
        - Replace interface
        - Change the name of the server
        - Change the name of the request
        - What you want with the result
*/


class AddTwoIntsClientNode : public rclcpp::Node
{
public:
	AddTwoIntsClientNode() : Node("add_two_ints_client")
	{
        // Start a thread and it will directly exit
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsServie, this, 1, 4)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsServie, this, 6, 24)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsServie, this, 8, 2)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsServie, this, 5, 9)));
	}
    
    void callAddTwoIntsServie(int a, int b)
    {
        // Create a client
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        // Waiting until the server has created the service
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }
        
        // Ccreate a request
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        // Send the request to the server
        auto future = client->async_send_request(request);

        try 
        {
            // Obtain the response
            // In this point, the node is spinning
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, response->sum);
        } 
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
        
    }

private:
    // Create a thread object
    std::vector<std::thread> threads_;
};
    
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<AddTwoIntsClientNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
