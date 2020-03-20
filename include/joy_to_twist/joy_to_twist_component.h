#ifndef JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_H_INCLUDED
#define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_EXPORT __attribute__ ((dllexport))
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_IMPORT __attribute__ ((dllimport))
    #else
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_EXPORT __declspec(dllexport)
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_IMPORT __declspec(dllimport)
    #endif
    #ifdef JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_BUILDING_DLL
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_PUBLIC JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_EXPORT
    #else
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_PUBLIC JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_IMPORT
    #endif
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_PUBLIC_TYPE JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_PUBLIC
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_LOCAL
#else
    #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_EXPORT __attribute__ ((visibility("default")))
    #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_IMPORT
    #if __GNUC__ >= 4
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
    #else
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_PUBLIC
        #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_LOCAL
    #endif
    #define JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace joy_to_twist
{
    class JoyToTwistComponent: public rclcpp::Node
    {
    public:
        JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_PUBLIC
        explicit JoyToTwistComponent(const rclcpp::NodeOptions & options);
    private:
        rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & params);
        void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        double longitudal_input_ratio_;
        double lateral_input_ratio_;
    };
}

#endif  //JOY_TO_TWIST_JOY_TO_TWIST_COMPONENT_H_INCLUDED