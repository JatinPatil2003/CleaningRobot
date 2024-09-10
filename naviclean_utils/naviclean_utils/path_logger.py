#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        
        # Publisher to publish the Path
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)
        
        # Subscriber to the /amcl_pose topic to get the robot's current pose
        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)

        # Initialize the Path message
        self.path = Path()
        self.path.header.frame_id = "map"  # Set to 'map' since amcl_pose provides pose relative to the map frame

        # Timer to publish the path at a regular interval
        self.timer = self.create_timer(0.5, self.publish_path)

    def amcl_pose_callback(self, msg):
        # Create a PoseStamped message for the current robot pose
        pose_stamped = PoseStamped()
        
        # Set the header
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        
        # Set the position from the amcl_pose message
        pose_stamped.pose.position.x = msg.pose.pose.position.x
        pose_stamped.pose.position.y = msg.pose.pose.position.y
        pose_stamped.pose.position.z = msg.pose.pose.position.z

        # Set the orientation from the amcl_pose message
        pose_stamped.pose.orientation = msg.pose.pose.orientation
        
        # Append the current pose to the path
        self.path.poses.append(pose_stamped)

    def publish_path(self):
        # Update the path header's timestamp
        self.path.header.stamp = self.get_clock().now().to_msg()

        # Publish the path
        self.path_publisher.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
