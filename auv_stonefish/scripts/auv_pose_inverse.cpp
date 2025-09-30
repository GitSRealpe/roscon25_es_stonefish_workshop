#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>

class AUVGoalInverse : public rclcpp::Node
{
public:
    AUVGoalInverse() : Node("auv_feedback_node")
    {

        // std::string frame_id = declare_parameter("frame_id", "empty");
        // rclcpp::Parameter param = get_parameter("frame_id");
        // pose_msg_.header.frame_id = param.as_string();

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pub_pose_topic", 10);

        sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/auv/navigator/body_pose", 10, std::bind(&AUVGoalInverse::currentCallback, this, std::placeholders::_1));

        sub_goal = create_subscription<geometry_msgs::msg::Pose>(
            "/auv/goal", 10, std::bind(&AUVGoalInverse::goalCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_goal;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    geometry_msgs::msg::Pose current_msg_;
    geometry_msgs::msg::Pose goal_msg_;

    tf2::Transform mat_current;
    tf2::Transform mat_goal;
    tf2::Transform error;
    double roll, pitch, err_yaw = 0;

    void currentCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        std::cout << "got some poses" << "\n";
        current_msg_ = *msg;
    }

    void goalCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        std::cout << "got some goals" << "\n";
        goal_msg_ = *msg;
    }

    void computeInverse()
    {
        tf2::fromMsg(current_msg_, mat_current);
        tf2::fromMsg(goal_msg_, mat_goal);

        // see the goal w.r.t current frame at every iteration
        error = mat_current.inverseTimes(mat_goal);

        // get orientation as rpy
        tf2::Matrix3x3 m(error.getRotation());
        m.getRPY(roll, pitch, err_yaw, 1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AUVGoalInverse>());
    rclcpp::shutdown();
    return 0;
}