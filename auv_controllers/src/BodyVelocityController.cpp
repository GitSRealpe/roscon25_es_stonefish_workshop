#include <auv_controllers/BodyVelocityController.hpp>
#include "hardware_interface/system_interface.hpp"

using std::placeholders::_1;
namespace auv_controllers
{
    BodyVelocityController::BodyVelocityController(/* args */)
    {
        std::cout << "constructing controller\n";
    }

    BodyVelocityController::~BodyVelocityController()
    {
    }

    controller_interface::CallbackReturn BodyVelocityController::on_init()
    {
        try
        {
            // Create the parameter listener and get the parameters
            param_listener_ = std::make_shared<auv_velocity_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BodyVelocityController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Vel state topic on: %s", params_.body_velocity_state_topic.c_str());
        // specify namespace in node launch
        state_sub = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
            params_.body_velocity_state_topic, rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
            {
                const auto cmd = *msg;
                rt_state_.set(cmd);
            });

        RCLCPP_INFO(get_node()->get_logger(), "Vel command topic on: ~/body_velocity_command");
        // specify namespace in node launch
        twist_sub = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
            "~/body_velocity_command", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
            {
                const auto cmd = *msg;
                rt_command_.set(cmd);
            });

        // Allocate reference interfaces, 6 DOF, 0.0 initialization
        reference_interfaces_.resize(6, 0.0);
        // Allocate state interfaces, 6 DOF, 0.0 initialization
        state_interfaces_values_.resize(6, 0.0);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration BodyVelocityController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (auto &dof : params_.dof_names)
        {
            command_interfaces_config.names.push_back(dof + "/" + params_.command_interface);
        }

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration BodyVelocityController::state_interface_configuration() const
    {
        // if (params_.open_loop)
        if (true)
        {
            return {controller_interface::interface_configuration_type::NONE, {}};
        }

        std::vector<std::string> conf_names;

        conf_names.push_back(get_node()->get_name() + std::string("/x/") + hardware_interface::HW_IF_VELOCITY);
        conf_names.push_back(get_node()->get_name() + std::string("/yaw/") + hardware_interface::HW_IF_VELOCITY);

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};

        // return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::CallbackReturn BodyVelocityController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BodyVelocityController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool BodyVelocityController::on_set_chained_mode(bool /*chained_mode*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Controller is now in chained mode");
        twist_sub.reset();
        return true;
    }

    std::vector<hardware_interface::StateInterface> BodyVelocityController::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(state_interfaces_values_.size());
        std::string export_prefix = get_node()->get_name();

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("x/") + hardware_interface::HW_IF_VELOCITY,
                &state_interfaces_values_[0]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("y/") + hardware_interface::HW_IF_VELOCITY,
                &state_interfaces_values_[1]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("z/") + hardware_interface::HW_IF_VELOCITY,
                &state_interfaces_values_[2]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("roll/") + hardware_interface::HW_IF_VELOCITY,
                &state_interfaces_values_[3]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("pitch/") + hardware_interface::HW_IF_VELOCITY,
                &state_interfaces_values_[4]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("yaw/") + hardware_interface::HW_IF_VELOCITY,
                &state_interfaces_values_[5]));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> BodyVelocityController::on_export_reference_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.reserve(reference_interfaces_.size());
        std::string export_prefix = get_node()->get_name();

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("x/") + hardware_interface::HW_IF_VELOCITY,
                &reference_interfaces_[0]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("y/") + hardware_interface::HW_IF_VELOCITY,
                &reference_interfaces_[1]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("z/") + hardware_interface::HW_IF_VELOCITY,
                &reference_interfaces_[2]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("roll/") + hardware_interface::HW_IF_VELOCITY,
                &reference_interfaces_[3]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("pitch/") + hardware_interface::HW_IF_VELOCITY,
                &reference_interfaces_[4]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("yaw/") + hardware_interface::HW_IF_VELOCITY,
                &reference_interfaces_[5]));

        return reference_interfaces;
    }

    controller_interface::return_type BodyVelocityController::update_reference_from_subscribers(
        const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {
        auto twist_command_op = rt_command_.try_get();
        if (twist_command_op.has_value())
            twist_command = twist_command_op.value();

        // watchdog looking thing
        if ((time - twist_command.header.stamp).seconds() > 2.0)
            twist_command = geometry_msgs::msg::TwistStamped();
        // RCLCPP_INFO(get_node()->get_logger(), "activate successful");
        reference_interfaces_[0] = twist_command.twist.linear.x;
        reference_interfaces_[1] = twist_command.twist.linear.y;
        reference_interfaces_[2] = twist_command.twist.linear.z;
        reference_interfaces_[3] = twist_command.twist.angular.x;
        reference_interfaces_[4] = twist_command.twist.angular.y;
        reference_interfaces_[5] = twist_command.twist.angular.z;

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type BodyVelocityController::update_and_write_commands(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        auto twist_state_op = rt_state_.try_get();
        if (twist_state_op.has_value())
            twist_state = twist_state_op.value();
        // Write exported states
        state_interfaces_values_[0] = twist_state.twist.linear.x;
        state_interfaces_values_[1] = twist_state.twist.linear.y;
        state_interfaces_values_[2] = twist_state.twist.linear.z;
        state_interfaces_values_[3] = twist_state.twist.angular.x;
        state_interfaces_values_[4] = twist_state.twist.angular.y;
        state_interfaces_values_[5] = twist_state.twist.angular.z;

        // Write commands to the hardware interface
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            // casting to void to avoid compiler warning
            static_cast<void>(command_interfaces_[i].set_value(reference_interfaces_[i]));
        }

        return controller_interface::return_type::OK;
    }

} // namespace auv_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(auv_controllers::BodyVelocityController, controller_interface::ChainableControllerInterface)
