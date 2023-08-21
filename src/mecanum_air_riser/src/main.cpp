#include "mecanum_air_riser/main.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "drobo_interfaces/msg/solenoid_state_msg.hpp"

void MecanumAirRiser::_topic_callback(const drobo_interfaces::msg::SolenoidStateMsg::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "%uを%d", msg->axle_position, msg->state);
    //motor_libを叩く
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
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumAirRiser>());
    rclcpp::shutdown();
    return 0;
}