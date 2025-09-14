#include <auv_controllers/BodyVelocityController.hpp>
#include "hardware_interface/system_interface.hpp"

using std::placeholders::_1;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
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
            param_listener_ = std::make_shared<ParamListener>(get_node());
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
        Eigen::MatrixXd tam(6, params_.num_thrusters);
        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < params_.num_thrusters; col++)
            {
                tam(row, col) = params_.thruster_allocation_matrix[row * params_.num_thrusters + col];
            }
        }
        // Eigen::MatrixXd tam(6, 2);
        // tam << cos(0.785398), cos(0.785398),
        //     sin(0.785398), -sin(0.785398),
        //     0, 0,
        //     0, 0,
        //     0, 0,
        //     -0.495, 0.495;
        std::cout << tam.format(CleanFmt) << "\n";

        tam_inv_ = tam.completeOrthogonalDecomposition().pseudoInverse();
        tam_inv_ = tam_inv_.unaryExpr([](double x)
                                      { return (abs(x) < 1e-4) ? 0.0 : x; });

        // parameter are read here
        twist_sub = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "~/body_velocity_command", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                const auto cmd = *msg;
                rt_command_.set(cmd);
            });

        // Allocate reference interfaces if needed
        reference_interfaces_.resize(2, std::numeric_limits<double>::quiet_NaN());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration BodyVelocityController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces_config.names = {"auv/thruster3_joint/velocity",
                                           "auv/thruster4_joint/velocity"};
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
        RCLCPP_INFO(get_node()->get_logger(), "activate successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BodyVelocityController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool BodyVelocityController::on_set_chained_mode(bool /*chained_mode*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "controller is now in chained mode");
        return true;
    }

    std::vector<hardware_interface::StateInterface> BodyVelocityController::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> exported_state_interfaces;

        std::string export_prefix = get_node()->get_name();
        twist_state.linear.x = 3.0;
        twist_state.angular.z = 1.0;

        exported_state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("x/") + hardware_interface::HW_IF_VELOCITY, &twist_state.linear.x));
        exported_state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("yaw/") + hardware_interface::HW_IF_VELOCITY, &twist_state.angular.z));

        return exported_state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> BodyVelocityController::on_export_reference_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.reserve(reference_interfaces_.size());

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                get_node()->get_name() + std::string("/x"), hardware_interface::HW_IF_VELOCITY,
                &reference_interfaces_[0]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                get_node()->get_name() + std::string("/yaw"), hardware_interface::HW_IF_VELOCITY,
                &reference_interfaces_[1]));

        return reference_interfaces;
    }

    controller_interface::return_type BodyVelocityController::update_reference_from_subscribers(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        auto twist_command_op = rt_command_.try_get();
        if (twist_command_op.has_value())
        {
            twist_command = twist_command_op.value();
        }

        command = Eigen::VectorXd::Zero(6);
        command << twist_command.linear.x,
            twist_command.linear.y,
            twist_command.linear.z,
            twist_command.angular.x,
            twist_command.angular.y,
            twist_command.angular.z;

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type BodyVelocityController::update_and_write_commands(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        Eigen::VectorXd setpoints = tam_inv_ * command;

        // Write commands to the hardware interface
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            // casting to void to avoid compiler warning
            static_cast<void>(command_interfaces_[i].set_value(setpoints[i]));
        }

        return controller_interface::return_type::OK;
    }

} // namespace auv_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(auv_controllers::BodyVelocityController, controller_interface::ChainableControllerInterface)
