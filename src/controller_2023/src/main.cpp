#include "controller_2023/main.hpp"

Controller2023::Controller2023(const rclcpp::NodeOptions& options) : rclcpp::Node("controller_2023", options){
    const std::string hw_name = this->declare_parameter<std::string>(
        "hw_type",
        p9n_interface::HW_NAME::DUALSENSE
    );

    try {
        this->_hw_type = p9n_interface::getHwType(hw_name);
    }catch (std::runtime_error& e){
        RCLCPP_ERROR(this->get_logger(), e.what());
        RCLCPP_ERROR(this->get_logger(), "Please select hardware from %s,", p9n_interface::getAllHwName().c_str());
        rclcpp::shutdown();
        return;
    }

    this->_p9n_if = std::make_unique<p9n_interface::PlayStationInterface>(this->_hw_type);
    this->_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
        std::bind(&Controller2023::onJoy, this, std::placeholders::_1)
    );
}

void Controller2023::onJoy(sensor_msgs::msg::Joy::ConstSharedPtr joy_msg){
    this->_p9n_if->setJoyMsg(joy_msg);

    //押された一回にのみ発火
    if(this->_p9n_if->pressedCircle() && !_joy_before_state.Circle){
        _joy_before_state.Circle = true;
        //ここから色々な処理
        RCLCPP_INFO(this->get_logger(), "○を押したね");
    }
    //離された一回にのみ発火
    if(!this->_p9n_if->pressedCircle() && _joy_before_state.Circle){
        _joy_before_state.Circle = false;
        //ここから色々な処理
        RCLCPP_INFO(this->get_logger(), "○を離したね");
    }
}
