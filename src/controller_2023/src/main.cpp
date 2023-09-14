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
    if(this->_p9n_if->pressedL2() && !_joy_before_state.L2){
        _joy_before_state.L2 = true;
        RCLCPP_INFO(this->get_logger(), "上昇！");
        auto msg = std::make_shared<example_interfaces::msg::Int8>();
        msg->data = true;
        _demeter_publisher->publish(*msg);
    }
    if(!this->_p9n_if->pressedL2() && _joy_before_state.L2){
        _joy_before_state.L2 = false;
        RCLCPP_INFO(this->get_logger(), "上昇ストップ！");
        auto msg = std::make_shared<example_interfaces::msg::Int8>();
        msg->data = -1;
        _demeter_publisher->publish(*msg);
    }
    if(this->_p9n_if->pressedL2() && !_joy_before_state.R2){
        _joy_before_state.R2 = true;
        RCLCPP_INFO(this->get_logger(), "下降！");
        auto msg = std::make_shared<example_interfaces::msg::Int8>();
        msg->data = true;
        _demeter_publisher->publish(*msg);
    }
    if(!this->_p9n_if->pressedR2() && _joy_before_state.R2){
        _joy_before_state.R2 = false;
        RCLCPP_INFO(this->get_logger(), "下降ストップ！");
        auto msg = std::make_shared<example_interfaces::msg::Int8>();
        msg->data = -1;
        _demeter_publisher->publish(*msg);
    }
}
