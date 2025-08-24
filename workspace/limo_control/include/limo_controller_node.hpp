#pragma once

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/float32.hpp>

#include "unicycle_model.hpp"

namespace limo
{
    class LimoControllerNode : public rclcpp::Node
    {
    public:
        LimoControllerNode();

    private:
        void control_loop();
        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        UnicycleKinematicState compute_error_body_frame() const;
        double yaw_from_quat(const geometry_msgs::msg::Quaternion &q);

        // Publishers and subscribers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // For error diagnostics
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr l2_error_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angular_error_pub_;

        // Internal state
        UnicycleKinematicState goal_state_;
        UnicycleKinematicState odom_state_;
        bool has_goal_{false};
        bool goal_reached_{false};

        // Unicycle controller
        UnicycleController controller_;
    };
} // namespace limo