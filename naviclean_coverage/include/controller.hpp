#ifndef SEND_GOAL_HPP
#define SEND_GOAL_HPP

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class SendGoal 
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    SendGoal(rclcpp::Node::SharedPtr node);

    void set_goal(const float x, const float y);

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

    NavigateToPose::Goal goal_msg_;
    
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options_;

    NavigateToPose::Feedback feedback_;

    rclcpp::Node::SharedPtr node_;

    void feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                           const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
};

#endif  // SEND_GOAL_HPP
