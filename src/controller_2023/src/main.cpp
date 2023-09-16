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
    this->_riser_publisher = create_publisher<drobo_interfaces::msg::SolenoidStateMsg>("solenoid_order", rclcpp::QoS(10));
    this->_md_publisher = create_publisher<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", rclcpp::QoS(10));
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
    if(this->_p9n_if->pressedDPadRight() && !_joy_before_state.DPadRight){
        _joy_before_state.DPadRight = true;
        RCLCPP_INFO(this->get_logger(), "FrontSolenoid_%s!", _solenoid_state.IsFrontSolenoidUp ? "上昇" : "下降");
        front_solenoid_msg->axle_position = 0;
        front_solenoid_msg->state = _solenoid_state.IsFrontSolenoidUp;
        _riser_publisher->publish(*front_solenoid_msg);
        _solenoid_state.IsFrontSolenoidUp = !_solenoid_state.IsFrontSolenoidUp;
    }
    if(!this->_p9n_if->pressedDPadRight() && _joy_before_state.DPadRight) _joy_before_state.DPadRight = false;

    if(this->_p9n_if->pressedDPadUp() && !_joy_before_state.DPadUp){
        _joy_before_state.DPadUp = true;
        RCLCPP_INFO(this->get_logger(), "MiddleSolenoid_%s!", _solenoid_state.IsMidSolenoidUp ? "上昇" : "下降");
        mid_solenoid_msg->axle_position = 1;
        mid_solenoid_msg->state = _solenoid_state.IsMidSolenoidUp;
        _riser_publisher->publish(*mid_solenoid_msg);
        _solenoid_state.IsMidSolenoidUp = !_solenoid_state.IsMidSolenoidUp;
    }
    if(!this->_p9n_if->pressedDPadUp() && _joy_before_state.DPadUp) _joy_before_state.DPadUp = false;

    if(this->_p9n_if->pressedDPadLeft() && !_joy_before_state.DPadLeft){
        _joy_before_state.DPadLeft = true;
        RCLCPP_INFO(this->get_logger(), "RearSolenoid_%s!", _solenoid_state.IsRearSolenoidUp ? "上昇" : "下降");
        rear_solenoid_msg->axle_position = 2;
        rear_solenoid_msg->state = _solenoid_state.IsRearSolenoidUp;
        _riser_publisher->publish(*rear_solenoid_msg);
        _solenoid_state.IsRearSolenoidUp = !_solenoid_state.IsRearSolenoidUp;
    }
    if(!this->_p9n_if->pressedDPadLeft() && _joy_before_state.DPadLeft) _joy_before_state.DPadLeft = false;
}
