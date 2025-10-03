#include <auv_controllers/BodyPoseController.hpp>
#include "hardware_interface/system_interface.hpp"
// #include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
namespace auv_controllers
{
    BodyPoseController::BodyPoseController(/* args */)
    {
    }

    BodyPoseController::~BodyPoseController()
    {
    }

    controller_interface::CallbackReturn BodyPoseController::on_init()
    {
        try
        {
            // Create the parameter listener and get the parameters
            param_listener_ = std::make_shared<auv_pose_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BodyPoseController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Current Pose state topic on: %s", params_.body_pose_state_topic.c_str());
        // specify namespace in node launch
        state_sub = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
            params_.body_pose_state_topic, rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                const auto cmd = *msg;
                rt_state_.set(cmd);
            });

        RCLCPP_INFO(get_node()->get_logger(), "Pose command topic on: ~/body_pose_command");
        // specify namespace in node launch
        pose_sub = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/body_pose_command", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                const auto cmd = *msg;
                rt_command_.set(cmd);
            });

        // Allocate reference interfaces, 6 DOF, 0.0 initialization
        reference_interfaces_.resize(7, 0.0);
        // Allocate state interfaces, 6 DOF, 0.0 initialization
        state_interfaces_values_.resize(7, 0.0);

        // initialize pid vars
        pid_err.setZero();
        prev_t = rclcpp::Time(0);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration BodyPoseController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (auto &dof : params_.dof_names)
        {
            command_interfaces_config.names.push_back(dof + "/" + params_.command_interface);
        }

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration BodyPoseController::state_interface_configuration() const
    {
        // if (params_.open_loop)
        if (true)
        {
            return {controller_interface::interface_configuration_type::NONE, {}};
        }

        std::vector<std::string> conf_names;

        conf_names.push_back(get_name() + std::string("/x/") + hardware_interface::HW_IF_VELOCITY);
        conf_names.push_back(get_name() + std::string("/yaw/") + hardware_interface::HW_IF_VELOCITY);

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};

        // return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::CallbackReturn BodyPoseController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BodyPoseController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool BodyPoseController::on_set_chained_mode(bool /*chained_mode*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Controller is now in chained mode");
        pose_sub.reset();
        return true;
    }

    std::vector<hardware_interface::StateInterface> BodyPoseController::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(state_interfaces_values_.size());
        std::string export_prefix = get_node()->get_name();

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("position/x"), &state_interfaces_values_[0]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("position/y"), &state_interfaces_values_[1]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("position/z"), &state_interfaces_values_[2]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("orientation/x"), &state_interfaces_values_[3]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("orientation/y"), &state_interfaces_values_[4]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("orientation/z"), &state_interfaces_values_[5]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                export_prefix, std::string("orientation/w"), &state_interfaces_values_[6]));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> BodyPoseController::on_export_reference_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.reserve(reference_interfaces_.size());
        std::string export_prefix = get_node()->get_name();

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("position/x"), &reference_interfaces_[0]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("position/y"), &reference_interfaces_[1]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("position/z"), &reference_interfaces_[2]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("orientation/x"), &reference_interfaces_[3]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("orientation/y"), &reference_interfaces_[4]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("orientation/z"), &reference_interfaces_[5]));

        reference_interfaces.push_back(
            hardware_interface::CommandInterface(
                export_prefix, std::string("orientation/w"), &reference_interfaces_[6]));

        return reference_interfaces;
    }

    controller_interface::return_type BodyPoseController::update_reference_from_subscribers(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        auto pose_command_op = rt_command_.try_get();
        if (pose_command_op.has_value())
            pose_command = pose_command_op.value();

        reference_interfaces_[0] = pose_command.pose.position.x;
        reference_interfaces_[1] = pose_command.pose.position.y;
        reference_interfaces_[2] = pose_command.pose.position.z;
        reference_interfaces_[3] = pose_command.pose.orientation.x;
        reference_interfaces_[4] = pose_command.pose.orientation.y;
        reference_interfaces_[5] = pose_command.pose.orientation.z;
        reference_interfaces_[6] = pose_command.pose.orientation.w;

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type BodyPoseController::update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {
        auto pose_state_op = rt_state_.try_get();
        if (pose_state_op.has_value())
            pose_state = pose_state_op.value();
        // Write exported states
        state_interfaces_values_[0] = pose_state.pose.position.x;
        state_interfaces_values_[1] = pose_state.pose.position.y;
        state_interfaces_values_[2] = pose_state.pose.position.z;
        state_interfaces_values_[3] = pose_state.pose.orientation.x;
        state_interfaces_values_[4] = pose_state.pose.orientation.y;
        state_interfaces_values_[5] = pose_state.pose.orientation.z;
        state_interfaces_values_[6] = pose_state.pose.orientation.w;

        // watchdog looking thing
        if ((time.seconds() - pose_command.header.stamp.sec) < 2.0)
        {
            tf2::fromMsg(pose_state.pose, mat_current);
            tf2::fromMsg(pose_command.pose, mat_goal);
            // see the goal w.r.t current frame at every iteration
            error = mat_current.inverseTimes(mat_goal);
            // get orientation as rpy
            tf2::Matrix3x3 m(error.getRotation());
            m.getRPY(err_roll, err_pitch, err_yaw, 1);

            dt = (time.seconds() - prev_t.seconds());

            for (size_t i = 0; i < 3; i++)
            {
                // proportional
                pid_err(i, 0) = params_.gains.dof_names_map[params_.dof_names[i]].p * error.getOrigin()[i];
                // derivative smooths
                pid_err(i, 2) = params_.gains.dof_names_map[params_.dof_names[i]].d * (error.getOrigin()[i] - last_error_x) / dt;
            }
            // pid_err(0, 0) = params_.gains.dof_names_map.[params_.dof_names[0]].d * error.getOrigin()[0];
            // pid_err(1, 0) = params_.gains.dof_names_map[params_.dof_names[1]].d * error.getOrigin().y();
            // pid_err(2, 0) = params_.gains.dof_names_map.at(0).d * error.getOrigin().z();

            // // integral for steady state error,
            // integral_x += error.getOrigin().x() * dt;
            // integral_y += error.getOrigin().y() * dt;
            // integral_z += error.getOrigin().z() * dt;
            // pid_err(0, 1) = 0.1 * integral_x;
            // pid_err(1, 1) = 0.1 * integral_y;
            // pid_err(2, 1) = 0.1 * integral_z;
            // // derivative smooths
            // pid_err(0, 2) = 0.3 * (error.getOrigin().x() - last_error_x) / dt;
            // pid_err(1, 2) = 0.5 * (error.getOrigin().y() - last_error_y) / dt;
            // pid_err(2, 2) = 0.5 * (error.getOrigin().z() - last_error_z) / dt;
            // last_error_x = error.getOrigin().x();
            // last_error_y = error.getOrigin().y();
            // last_error_z = error.getOrigin().z();

            // std::cout << "x action:" << std::clamp(pid_err.row(0).sum(), -max_vel, max_vel) << "\n";
            // std::cout << "y action:" << std::clamp(pid_err.row(1).sum(), -max_vel, max_vel) << "\n";
            // std::cout << "z action:" << std::clamp(pid_err.row(2).sum(), -max_vel, max_vel) << "\n";
            // std::cout << "yaw action:" << std::clamp(0.7 * err_yaw, -max_rot_vel, max_rot_vel) << "\n";

            // Write commands to the hardware interface
            for (size_t i = 0; i < command_interfaces_.size(); ++i)
            {
                // casting to void to avoid compiler warning
                static_cast<void>(command_interfaces_[i].set_value(pid_err.row(i).sum()));
            }
        }
        else
        {
            for (size_t i = 0; i < command_interfaces_.size(); ++i)
                static_cast<void>(command_interfaces_[i].set_value(0.0));
        }

        prev_t = time;

        return controller_interface::return_type::OK;
    }

} // namespace auv_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(auv_controllers::BodyPoseController, controller_interface::ChainableControllerInterface)
