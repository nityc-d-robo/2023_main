#include <rclcpp/rclcpp.hpp>
#include <drobo_interfaces/msg/solenoid_state_msg.hpp>

class MecanumAirRiser : public rclcpp::Node{
    private:
        rclcpp::Subscription<drobo_interfaces::msg::SolenoidStateMsg>::SharedPtr _subscription;
        void _topic_callback(const drobo_interfaces::msg::SolenoidStateMsg::SharedPtr msg);
    public:
        MecanumAirRiser(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
        MecanumAirRiser(
            const std::string& name_space,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
};