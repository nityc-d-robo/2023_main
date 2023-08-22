#include "mecanum_air_riser/main.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "drobo_interfaces/msg/solenoid_state_msg.hpp"
#include <motor_lib/motor_lib.hpp>

void MecanumAirRiser::_topic_callback(const drobo_interfaces::msg::SolenoidStateMsg::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "%uを%d", msg->axle_position, msg->state);
    int solenoind_power = msg->state ? 0 : 999;
    MotorLib::sd.sendPowers(msg->axle_position, NULL, solenoind_power, solenoind_power, 5000);
}

MecanumAirRiser::MecanumAirRiser(
    const rclcpp::NodeOptions& options
): MecanumAirRiser("", options){}

MecanumAirRiser::MecanumAirRiser(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("mecanum_air_riser", name_space, options){
    _subscription = this->create_subscription<drobo_interfaces::msg::SolenoidStateMsg>(
        "solenoid_order",
        rclcpp::QoS(10),
        std::bind(&MecanumAirRiser::_topic_callback, this, std::placeholders::_1)
    );
}

int main(int argc, char* argv[]){
    MotorLib::usb_config.vendor_id = 0x483;
    MotorLib::usb_config.product_id = 0x5740;
    MotorLib::usb_config.b_interface_number = 0;

    MotorLib::usb.setUsb(&MotorLib::usb_config);
    MotorLib::usb.openUsb();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumAirRiser>());
    rclcpp::shutdown();
    return 0;
}