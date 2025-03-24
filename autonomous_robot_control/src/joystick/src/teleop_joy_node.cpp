#include "rclcpp/rclcpp.hpp" 
#include "sensor_msgs/msg/joy.hpp" 
#include "geometry_msgs/msg/twist.hpp" // Include the Twist message type for velocity commands

//This code is a basic teleoperation node that reads joystick input and
// publishes velocity commands to control a robot, General code for any robot 
class MyJoy : public rclcpp::Node
{
public:
    MyJoy() // Constructor for the MyJoy class
    : Node("teleop_joy_node") // Initialize the node with the name "teleop_joy_node"
    {
        // Create a publisher for geometry_msgs::msg::Twist messages on the "/cmd_vel_joy" topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_joy", 20);
        
        // Create a subscription to the "/joy" topic for sensor_msgs::msg::Joy messages
        subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 20, std::bind(&MyJoy::callback, this, std::placeholders::_1)); // Bind the callback function

        declare_parameter<double>("linearK", 0.8f); // Blend factor
        get_parameter("linearK", k);
    }

private:
    double k;
    geometry_msgs::msg::Twist twist_message; // Create a Twist message to hold velocity commands
    void callback(const sensor_msgs::msg::Joy::SharedPtr msg) // Callback function for joystick input
    {
        
        if((abs(msg->axes[1])) + abs(msg->axes[0])<0.1)
        {
            twist_message.linear.x = 0;
            twist_message.angular.z = 0;
        }
        else{
            //Set rotation speed based on joystick input
            twist_message.linear.x = (msg->axes[1])*k + twist_message.linear.x*(1-k);
            twist_message.angular.z = msg->axes[0];
            RCLCPP_INFO(this->get_logger(), "Linear-axes: '%f'", msg->axes[1]);
        }

        // Publish the Twist message to the "/cmd_vel" topic
        publisher_->publish(twist_message);

    }

    // Define publisher and subscriber shared pointers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // Publisher for Twist messages
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_; // Subscriber for Joy messages
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<MyJoy>()); // Create and spin the MyJoy node
    rclcpp::shutdown(); // Shutdown the ROS 2 system
    return 0; // Exit the program
}
