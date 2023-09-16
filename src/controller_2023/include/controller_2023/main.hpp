#pragma once

#include <string>
#include <memory>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>

#include <p9n_interface/p9n_interface.hpp>
#include "drobo_interfaces/msg/solenoid_state_msg.hpp"
#include "drobo_interfaces/msg/md_lib_msg.hpp"

struct JoystickState{
    bool Square = false;
    bool Circle = false;
    bool Triangle = false;
    bool Cross = false;
    bool L1 = false;
    bool R1 = false;
    bool DPadUp = false;
    bool DPadDown = false;
    bool DPadLeft = false;
    bool DPadRight = false;
    bool Start = false;
    bool Select = false;
    bool PS = false;
};

struct SolenoidState{
    bool IsFrontSolenoidUp = true;
    bool IsMidSolenoidUp = true;
    bool IsRearSolenoidUp = true;
};

class Controller2023 : public rclcpp::Node{
    private:
        p9n_interface::HW_TYPE _hw_type;
        std::unique_ptr<p9n_interface::PlayStationInterface> _p9n_if;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;
        JoystickState _joy_before_state;
        SolenoidState _solenoid_state;
        void onJoy(sensor_msgs::msg::Joy::ConstSharedPtr);

        rclcpp::Publisher<drobo_interfaces::msg::SolenoidStateMsg>::SharedPtr _riser_publisher;
        std::shared_ptr<drobo_interfaces::msg::SolenoidStateMsg> front_solenoid_msg = std::make_shared<drobo_interfaces::msg::SolenoidStateMsg>();
        std::shared_ptr<drobo_interfaces::msg::SolenoidStateMsg> mid_solenoid_msg = std::make_shared<drobo_interfaces::msg::SolenoidStateMsg>();
        std::shared_ptr<drobo_interfaces::msg::SolenoidStateMsg> rear_solenoid_msg = std::make_shared<drobo_interfaces::msg::SolenoidStateMsg>();

        rclcpp::Publisher<drobo_interfaces::msg::MdLibMsg>::SharedPtr _md_publisher;

    public:
        Controller2023(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
        Controller2023(
            const std::string& name_space,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Controller2023);
