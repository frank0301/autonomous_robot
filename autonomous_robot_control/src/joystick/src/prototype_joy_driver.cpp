#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"     
#include "joystick/joy_compute_control.hpp"
#include "robot_common_interfaces/msg/ser_com_struct.hpp"

// Define a class for the joy driver node, inheriting from rclcpp::Node
class PrototypeJoy : public rclcpp::Node
{
public:
    // Constructor for the PrototypeJoy class
    PrototypeJoy()
    : Node("prototype_joy_driver") // Initialize the node with a name
    {
        // Create a subscription to the /joy topic to receive joystick messages
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&PrototypeJoy::callback_func, this, std::placeholders::_1));

        // Create a publisher for sending motor control commands
        publisher_ = this->create_publisher<robot_common_interfaces::msg::SerComStruct>(
            "motor_throttle_control", 10);
    }

private:
    // Callback function to process incoming joystick messages
    void callback_func(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Comments refer to the buttons on Sony's PlayStation controller 
        if(msg->buttons[0]) // pressing 'X' shifts to low gear
            is_low_gear_ = true;
        else if(msg->buttons[3]) // pressing 'square' shifts to high gear
            is_low_gear_ = false;

        // Calculate throttle based on joystick axes for forward motion
        if(msg->axes[5] >= 0){
            throttle_ = -(msg->axes[5]) / 2.0 + 0.5; // Adjust throttle range for forward motion
        }
        else{
            throttle_ = (-msg->axes[5] + 1.0) / 2.0; // Adjust throttle range for backward motion
        }

        // Reverse (only works if we're not going forward)
        if(throttle_ <= 0){
            if(msg->axes[2] >= 0){
                throttle_ = (msg->axes[2]) / 2.0 - 0.5; // Adjust throttle for backward movement
            }
            else{
                throttle_ = (msg->axes[2] - 1.0) / 2.0; // Adjust throttle for full backward input
            }
        }
        
        // Create an instance of the control class to compute motor commands
        PrototypeControl control;
        RCLCPP_INFO(this->get_logger(), "Steering : '%f'", msg->axes[0]); // Log steering information
        SerialCom ser_com = control.compute_control(throttle_, msg->axes[0], is_low_gear_); // Compute control based on throttle and steering

        // Prepare a message to send motor commands
        auto pwm_struct = robot_common_interfaces::msg::SerComStruct();
        pwm_struct.pwm_l = ser_com.pwm_L * 0.594177; // Scale left motor PWM
        pwm_struct.pwm_r = ser_com.pwm_R * 0.594177; // Scale right motor PWM
        pwm_struct.dir_l = ser_com.dir_L; // Set reverse direction for left motor
        pwm_struct.dir_r = ser_com.dir_R; // Set reverse direction for right motor
        pwm_struct.gear = ser_com.gear; // Set gear status
        publisher_->publish(pwm_struct); // Publish motor command message

        // Additional logging statements (commented out)
        // RCLCPP_INFO(this->get_logger(), "Right wheel speed : '%f'", vel_message.data[0]);
        // RCLCPP_INFO(this->get_logger(), "Left wheel speed : '%f'", vel_message.data[1]);
        // RCLCPP_INFO(this->get_logger(), "Right wheel PWM : '%d'", phi_right_);
        // RCLCPP_INFO(this->get_logger(), "Left wheel PWM : '%d'", phi_left_);
        // RCLCPP_INFO(this->get_logger(), "High gear : '%s'", ser_com.gear? "true" : "false");
    }

    // Publisher for sending motor commands
    rclcpp::Publisher<robot_common_interfaces::msg::SerComStruct>::SharedPtr publisher_;
    // Subscription for receiving joystick messages
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    bool is_low_gear_ = true; // Default gear configuration set to low (true)
    double throttle_; // Variable to store the current throttle value
    int phi_right_; // Variable for right wheel PWM value (not used in this snippet)
    int phi_left_; // Variable for left wheel PWM value (not used in this snippet)
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<PrototypeJoy>()); // Spin the joy driver node
    rclcpp::shutdown(); // Shutdown the ROS 2 system
    return 0; // Exit the program
}
