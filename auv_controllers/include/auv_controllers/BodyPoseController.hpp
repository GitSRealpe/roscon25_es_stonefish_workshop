#ifndef AUV_POS_CTRL_HPP
#define AUV_POS_CTRL_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Core>
#include <auv_controllers/auv_pose_controller_params.hpp>
namespace auv_controllers
{
    class BodyPoseController : public controller_interface::ChainableControllerInterface
    {
    private:
        geometry_msgs::msg::PoseStamped pose_command;
        geometry_msgs::msg::PoseStamped pose_state;
        realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::PoseStamped> rt_command_;
        realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::PoseStamped> rt_state_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr state_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;

        // Parameters from ROS for
        std::shared_ptr<auv_pose_controller::ParamListener> param_listener_;
        auv_pose_controller::Params params_;

        tf2::Transform mat_current;
        tf2::Transform mat_goal;
        tf2::Transform error;
        double err_roll, err_pitch, err_yaw, integral_x, integral_y, integral_z, last_error_x, last_error_y, last_error_z, dt = 0;
        rclcpp::Time prev_t;
        Eigen::Matrix3d pid_err;

    public:
        BodyPoseController(/* args */);
        ~BodyPoseController();

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // call when controller
        bool on_set_chained_mode(bool activate) override;

        std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

        // Chainable controller replaces update() with the following two functions
        controller_interface::return_type update_reference_from_subscribers(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;
        // configure the interface to write commands to (HW)
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        // configure the interface to read state from (HW)
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    };

};

#endif // AUV_POS_CTRL_HPP
