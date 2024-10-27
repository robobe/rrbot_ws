#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <vector>

namespace arduino_hw
{
class PWMMotorHardware : public hardware_interface::SystemInterface
{
public:
    PWMMotorHardware(){

    }

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) 
    {
        // Initialize motor configuration from 'info' (YAML parameters)
        RCLCPP_INFO(rclcpp::get_logger("PWMMotorHardware"), "Configuring PWM Motor Hardware");
        // Initialize PWM signal controls here (e.g., setting up GPIO or other hardware interfaces)
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() 
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        // Export state interfaces such as position, velocity, current, etc.
        state_interfaces.emplace_back(hardware_interface::StateInterface("joint1", hardware_interface::HW_IF_POSITION, &position_));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() 
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // Export command interfaces such as setting PWM values to control motor speed.
        command_interfaces.emplace_back(hardware_interface::CommandInterface("joint1", hardware_interface::HW_IF_POSITION, &position_command_));
        return command_interfaces;
    }

    hardware_interface::return_type start() 
    {
        // Start the motor (e.g., send the initial PWM signal to set speed to 0)
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type stop() 
    {
        // Stop the motor (set PWM to 0)
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period) 
    {
        // Update the current state of the motor (e.g., read encoder values, calculate velocity)
        position_ = position_command_;//read_motor_velocity();
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Write the current command (PWM signal) to the motor
        // set_motor_pwm(position_command_);
        // RCLCPP_INFO(rclcpp::get_logger(""), "write %f pos", position_command_);
        return hardware_interface::return_type::OK;
    }

private:
    double position_command_; // Commanded velocity (which can be translated into PWM)
    double position_;         // Actual motor velocity

    // void set_motor_pwm(double pwm_value)
    // {
    //     // Implement logic to send PWM signal to motor
    //     RCLCPP_INFO(rclcpp::get_logger("PWMMotorHardware"), "Setting PWM value: %f", pwm_value);
    // }

    // double read_motor_velocity()
    // {
    //     // Implement logic to read motor velocity (e.g., from an encoder or sensor)
    //     return 0.0;  // Placeholder
    // }
};
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arduino_hw::PWMMotorHardware, hardware_interface::SystemInterface)
