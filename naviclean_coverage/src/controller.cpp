#include "controller.hpp"

SendGoal::SendGoal(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // Create an action client
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

    // Wait until the action server is available
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        return;
    }

    send_goal_options_ = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options_.feedback_callback = std::bind(&SendGoal::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback = std::bind(&SendGoal::result_callback, this, std::placeholders::_1);
}

void SendGoal::feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    RCLCPP_INFO(node_->get_logger(), "Feedback received: Current robot position: (%f, %f)", feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
    feedback_ = *feedback;
}

void SendGoal::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            break;
    }
    rclcpp::shutdown();
}

void SendGoal::set_goal(const float x, const float y){
    goal_msg_.pose.pose.position.x = x;
    goal_msg_.pose.pose.position.y = y;
    goal_msg_.pose.pose.orientation.w = 1.0; // No rotation, facing forward
    goal_msg_.pose.header.frame_id = "map"; // Assuming the robot uses the "map" frame
    goal_msg_.pose.header.stamp = node_->get_clock()->now();

    // Send the goal
    RCLCPP_INFO(node_->get_logger(), "Sending goal...");

    client_->async_send_goal(goal_msg_, send_goal_options_);
}

// void SendGoal::get_feedback(){

// }
