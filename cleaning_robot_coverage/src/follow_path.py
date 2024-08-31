#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import actionlib
from  move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
import math
import ast

class PathSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('path_subscriber', anonymous=True)
        rospy.Subscriber("coverage_points", String, self.callback)
        self.path_publisher = rospy.Publisher("coverage_path", Path,  queue_size=10)
        self.path_points = []
        self.client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        self.rate = rospy.Rate(20)

    def callback(self, data):
        # rospy.loginfo(f"Received Points")
        self.path_points = ast.literal_eval(data.data)

    def spin(self):
        received = 0
        while received < 5:
            received += 1
            self.publish_path()
            self.rate.sleep()

        self.follow_path()
        # self.client.wait_for_server()
        # self.send_goal(2.0, 2.0)

    def send_goal(self, x, y):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation.w = 1.0
        rospy.loginfo(f"Goal Published: [{x}, {y}]")
        self.client.send_goal(self.goal,done_cb=self.result_callback, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()

    def result_callback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal Reached")

    def feedback_callback(self, feedback):
        # self.client.cancel_goal()
        x_1 = self.goal.target_pose.pose.position.x
        y_1 = self.goal.target_pose.pose.position.y
        x_2 = feedback.base_position.pose.position.x
        y_2 = feedback.base_position.pose.position.y
        distance = self.get_euclidian(x_1, y_1, x_2, y_2)
        # rospy.loginfo(f"Received feedback: {distance}")
        if distance < 0.2:
            self.client.cancel_goal()

    def get_euclidian(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def follow_path(self):
        self.client.wait_for_server()
        if self.path_points is not None:
            for path in self.path_points:
                for point in path:
                    self.send_goal(point[0], point[1])

    def publish_path(self):
        if self.path_points is not None:
            for path in self.path_points:
                msg = Path()
                msg.header.frame_id = 'map'
                msg.header.stamp = rospy.Time.now()
                for point in path:
                    point_msg = PoseStamped()
                    point_msg.header.frame_id = 'map'
                    point_msg.header.stamp = rospy.Time.now()
                    point_msg.pose.position.x = point[0]
                    point_msg.pose.position.y = point[1]
                    point_msg.pose.orientation.w = 1
                    msg.poses.append(point_msg)
                self.path_publisher.publish(msg)
                self.rate.sleep()
                rospy.loginfo("Published Plan")

if __name__ == '__main__':
    path_subscriber = PathSubscriber()
    path_subscriber.spin()
