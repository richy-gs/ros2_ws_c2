#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
    public:
        MyNode() : Node("cpp_test_node"), counter_(0)
        {
            RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");
            timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                std::bind(&MyNode::timerCallBack, this));
        }
    private:
        void timerCallBack()
        {
            counter_++;
            RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
            
        }

        rclcpp::TimerBase::SharedPtr timer_;
        int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Create a shared pointer
    auto node = std::make_shared<MyNode>();
    // Print a message
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}