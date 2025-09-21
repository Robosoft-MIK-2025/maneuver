#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("send_goal_node");

    auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 2.0;
    goal.pose.position.z = 0.5;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    RCLCPP_INFO(node->get_logger(), "Publishing goal pose...");
    rclcpp::Rate rate(1);
    while (rclcpp::ok()) {
        publisher->publish(goal);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
