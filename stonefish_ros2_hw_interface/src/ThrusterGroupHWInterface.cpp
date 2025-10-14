#include <stonefish_hw_interface/ThrusterGroupHWInterface.hpp>

namespace stonefish_hw_interface
{
    ThrusterGroupHWInterface::ThrusterGroupHWInterface(/* args */)
    {
        std::cout << "constructing hw interface\n";
    }

    ThrusterGroupHWInterface::~ThrusterGroupHWInterface()
    {
    }

    hardware_interface::CallbackReturn ThrusterGroupHWInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        // Store joint names and initialize storage for commands/states
        joint_names_.clear();

        for (const auto &joint : info.joints)
        {
            RCLCPP_INFO(this->get_logger(), "Getting joint interface: %s", joint.name.c_str());
            joint_names_.push_back(joint.name);
        }

        velocity_commands_.resize(joint_names_.size(), 0.0);
        position_states_.resize(joint_names_.size(), 0.0);
        velocity_states_.resize(joint_names_.size(), 0.0);
        torque_states_.resize(joint_names_.size(), 0.0);

        // Check for thruster write topic
        auto write_topic_it = info.hardware_parameters.find("thruster_group_write_topic");
        if (write_topic_it == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Thruster write topic not defined");
            return hardware_interface::CallbackReturn::ERROR;
        }

        auto thruster_group_write_topic_ = write_topic_it->second;
        RCLCPP_INFO(this->get_logger(), "Thruster write topic: %s", thruster_group_write_topic_.c_str());
        thruster_group_publisher_ = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(thruster_group_write_topic_, 10);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThrusterGroupHWInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // parameter are read here
        std::cout << "on_configure thruster hw interface\n";
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ThrusterGroupHWInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint_names_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]));

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint_names_[i], hardware_interface::HW_IF_EFFORT, &torque_states_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ThrusterGroupHWInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::return_type ThrusterGroupHWInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        for (size_t i = 0; i < velocity_states_.size(); ++i)
        {
            velocity_states_[i] = 0.0; // Placeholder: Replace with actual sensor data (e.g., RPM)
            position_states_[i] = 0.0;
            torque_states_[i] = 0.0;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ThrusterGroupHWInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Publish velocity commands to thruster topic
        std_msgs::msg::Float64MultiArray msg;
        msg.data = velocity_commands_;
        thruster_group_publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published thruster commands: %zu values", msg.data.size());
        return hardware_interface::return_type::OK;
    }

} // namespace stonefish_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(stonefish_hw_interface::ThrusterGroupHWInterface, hardware_interface::SystemInterface)
