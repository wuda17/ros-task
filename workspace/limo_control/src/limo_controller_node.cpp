#include "limo_controller_node.hpp"

namespace limo
{
    LimoControllerNode::LimoControllerNode() : Node("limo_controller"),
                                               controller_(1.0, 2.0, -0.5, 1.0, 1.0)
    {
        // Publish new control inputs on a timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&LimoControllerNode::control_loop, this));

        // Initialize publishers and subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&LimoControllerNode::odom_callback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&LimoControllerNode::goal_callback, this, std::placeholders::_1));

        l2_error_pub_ = this->create_publisher<std_msgs::msg::Float64>("l2_error", 10);
        angular_error_pub_ = this->create_publisher<std_msgs::msg::Float64>("angular_error", 10);
    }

    void LimoControllerNode::control_loop()
    {
        if (!has_goal_ || goal_reached_)
            return;

        UnicycleKinematicState error = compute_error_body_frame();

        // Publish error diagnostics
        std_msgs::msg::Float64 l2_error_msg{};
        l2_error_msg.data = std::sqrt(error.x * error.x + error.y * error.y);
        l2_error_pub_->publish(l2_error_msg);
        std_msgs::msg::Float64 angular_error_msg{};
        angular_error_msg.data = error.theta;
        angular_error_pub_->publish(angular_error_msg);

        UnicycleControlInput u = controller_.error_to_control_input(error);

        geometry_msgs::msg::Twist twist_msg{};
        twist_msg.linear.x = u.v;
        twist_msg.angular.z = u.omega;
        cmd_vel_pub_->publish(twist_msg);

        // If command output empty goal is reached. Still publish to stop robot when goal reached.
        if (u.v == 0.0 && u.omega == 0.0)
            goal_reached_ = true;
    }

    void LimoControllerNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        UnicycleKinematicState goal;
        goal.x = msg->pose.position.x;
        goal.y = msg->pose.position.y;
        goal.theta = yaw_from_quat(msg->pose.orientation);

        goal_state_ = goal;
        goal_reached_ = false;
        has_goal_ = true;
    }

    void LimoControllerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Odom position: (%.2f, %.2f)",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y);

        UnicycleKinematicState odom;
        odom.x = msg->pose.pose.position.x;
        odom.y = msg->pose.pose.position.y;
        odom.theta = yaw_from_quat(msg->pose.pose.orientation);

        odom_state_ = odom;
    }

    UnicycleKinematicState LimoControllerNode::compute_error_body_frame() const
    {
        // Transform goal into robot body frame
        double dx = goal_state_.x - odom_state_.x;
        double dy = goal_state_.y - odom_state_.y;
        double theta = odom_state_.theta;

        UnicycleKinematicState error;
        error.x = std::cos(theta) * dx + std::sin(theta) * dy;
        error.y = -std::sin(theta) * dx + std::cos(theta) * dy;

        // angular error wrapped to [-pi, pi]
        error.theta = goal_state_.theta - odom_state_.theta;
        while (error.theta > M_PI)
            error.theta -= 2 * M_PI;
        while (error.theta < -M_PI)
            error.theta += 2 * M_PI;

        return error;
    }

    double LimoControllerNode::yaw_from_quat(const geometry_msgs::msg::Quaternion &q)
    {
        return std::atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z));
    }

} // namespace limo

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<limo::LimoControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}