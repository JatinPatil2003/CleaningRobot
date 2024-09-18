#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import time

def make_pose(x, y, z, w):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose

def main():
    rclpy.init()

    # Create a BasicNavigator object to interact with the Nav2 stack
    navigator = BasicNavigator()

    # Wait for Nav2 to fully launch and be ready
    navigator.waitUntilNav2Active()

    init_pose = make_pose(0.0, 0.0, 0.0, 1.0)
    goal_pose = make_pose(1.0, 0.0, 0.0, 1.0)

    path = navigator.getPath(init_pose, goal_pose, use_start=True)
    
    smooth_plan = navigator.smoothPath(path)


    print(path)
    # print(smooth_plan)
    print()

    navigator.followPath(path)

    i = 0
    while not navigator.isTaskComplete():

        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated distance remaining to goal position: ' +
                  '{0:.3f}'.format(feedback.distance_to_goal) +
                  '\nCurrent speed of the robot: ' +
                  '{0:.3f}'.format(feedback.speed))

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # Shutdown the navigator and clean up
    # navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
