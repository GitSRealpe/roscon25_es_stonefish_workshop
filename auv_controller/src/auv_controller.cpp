#include <auv_controller/auv_controller.hpp>

using std::placeholders::_1;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
namespace auv_controller
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
        std::cout << "on_init controller\n";
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BodyVelocityController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // parameter are read here
        twist_sub = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "~/body_velocity_command", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                const auto cmd = *msg;
                rt_command_.set(cmd);
            });

        Eigen::MatrixXd tam(6, 2);
        tam << cos(0.785398), cos(0.785398),
            sin(0.785398), -sin(0.785398),
            0, 0,
            0, 0,
            0, 0,
            -0.495, 0.495;

        std::cout << tam.format(CleanFmt) << "\n";

        tam_inv_ = tam.completeOrthogonalDecomposition().pseudoInverse();
        tam_inv_ = tam_inv_.unaryExpr([](double x)
                                      { return (abs(x) < 1e-4) ? 0.0 : x; });
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
        return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
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

    controller_interface::return_type BodyVelocityController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        auto twist_command_op = rt_command_.try_get();
        if (twist_command_op.has_value())
        {
            twist_command = twist_command_op.value();
        }

        Eigen::VectorXd command = Eigen::VectorXd::Zero(6);
        command << twist_command.linear.x,
            twist_command.linear.y,
            twist_command.linear.z,
            twist_command.angular.x,
            twist_command.angular.y,
            twist_command.angular.z;

        Eigen::VectorXd setpoints = tam_inv_ * command;

        // Write commands to the hardware interface
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            // casting to void to avoid compiler warning
            static_cast<void>(command_interfaces_[i].set_value(setpoints[i]));
        }

        return controller_interface::return_type::OK;
    }

} // namespace auv_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(auv_controller::BodyVelocityController, controller_interface::ControllerInterface)
