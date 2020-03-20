#include <joy_to_twist/joy_to_twist_component.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<joy_to_twist::JoyToTwistComponent>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}