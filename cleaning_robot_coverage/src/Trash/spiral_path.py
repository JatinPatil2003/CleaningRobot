#!/usr/bin/env python3

import cv2
import yaml
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult

class MapLoader:
    def __init__(self, map_file, yaml_file):
        rospy.init_node('Coverage_Planner_Node')
        self.map_file = map_file
        self.yaml_file = yaml_file
        self.map_data = None
        self.map_image = None
        self.x_offset = 0.15
        self.y_offset = 0.7
        self.move_base_goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_result_callback)

    def load_map(self):
        # Load map parameters from YAML file
        with open(self.yaml_file, 'r') as file:
            self.map_data = yaml.safe_load(file)

        # Load map image using OpenCV
        self.map_image = cv2.imread(self.map_file, cv2.IMREAD_GRAYSCALE)

    def display_map_parameters(self):
        if self.map_data is not None:
            rospy.loginfo("Map Parameters:")
            for key, value in self.map_data.items():
                rospy.loginfo(f"{key}: {value}")
        else:
            rospy.loginfo("Map parameters not available. Call load_map() first.")

    def pixel_to_map_coordinates(self, x_pixel, y_pixel):
        # Convert pixel coordinates to map coordinates
        resolution = self.map_data['resolution']
        origin = self.map_data['origin']

        y_map = -(origin[0] + (y_pixel * resolution) + self.y_offset)
        x_map = origin[1] + (x_pixel * resolution) + self.x_offset

        return x_map, y_map

    def distance_to_nearest_black(self, y, x, radius=7):
        # Check distance to the nearest black pixel
        black_mask = self.map_image < 250
        y_range, x_range = self.map_image.shape

        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                if 0 <= y + i < y_range and 0 <= x + j < x_range and black_mask[y + i, x + j]:
                    return False  # Too close to a black pixel

        return True
    
    def publish_goal(self, position):
        goal_msg = MoveBaseActionGoal()
        goal_msg.goal.target_pose.header.frame_id = "map"
        goal_msg.goal.target_pose.pose.position.x = position[0]
        goal_msg.goal.target_pose.pose.position.y = position[1]
        goal_msg.goal.target_pose.pose.orientation.w = 1.0  # Assuming you want to keep the orientation constant

        rospy.loginfo(f"Publishing goal: {position}")
        self.move_base_goal_pub.publish(goal_msg)

    def move_base_result_callback(self, result_msg):
        # This callback will be called when the move_base action result is received
        # You can add additional logic here if needed
        rospy.loginfo("MoveBase Action Result Received")

    def display_modified_map(self):
        if self.map_image is not None:
            # Convert the grayscale image to RGB
            color_map = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2RGB)
            
            print(self.map_image.shape)
            print(color_map.shape)

            # Create mask for pixels below 250
            black_mask = self.map_image < 250

            # Set pixels to black based on the mask
            color_map[black_mask] = [0, 0, 0]  # Set black

            # Create mask for pixels above 250
            white_mask = self.map_image >= 250

            # Lists to store coordinates and goal indexes
            goal_positions = []
            goal_pixel_positions = []

            # Draw red points on the white part and store goal positions
            goal_index = 1
            max_index = len(range(0, self.map_image.shape[0], 10)) * len(range(0, self.map_image.shape[1], 10))
            for x in range(0, self.map_image.shape[1], 10):  # Adjust the step size as needed
                # Determine whether to iterate from top to bottom or bottom to top based on x-coordinate
                y_range = range(0, self.map_image.shape[0], 10) if x % 20 == 0 else reversed(range(0, self.map_image.shape[0], 10))
                
                for y in y_range:  # Adjust the step size as needed
                    if white_mask[y, x] and self.distance_to_nearest_black(y, x):
                        if goal_index == 1:
                            color_map[y, x] = [255, 0, 0]  # Blue for the first red dot
                        elif goal_index == max_index:
                            color_map[y, x] = [0, 255, 0]  # Green for the last red dot
                        else:
                            color_map[y, x] = [0, 0, 255]  # Red for other red dots

                        goal_pixel_positions.append(((x, y)))
                        x_map, y_map = self.pixel_to_map_coordinates(x, y)
                        goal_positions.append(((round(x_map, 2), round(y_map, 2)), goal_index))  # Store coordinates and goal index
                        goal_index += 1

            # Draw green lines between consecutive red dots
            for i in range(1, len(goal_positions)):
                cv2.line(color_map, (int(goal_pixel_positions[i-1][0]), int(goal_pixel_positions[i-1][1])),
                         (int(goal_pixel_positions[i][0]), int(goal_pixel_positions[i][1])), (0, 255, 0), 1)

                cv2.circle(color_map, (int(goal_pixel_positions[i-1][0]), int(goal_pixel_positions[i-1][1])), radius=1, color=(0, 0, 255), thickness=-1)  # -1 for filled circle

            rospy.loginfo("Goal Positions (Map Coordinates):")
            # for position, index in goal_positions:
            #     rospy.loginfo(f"Goal {index}: {position}")
            #     self.publish_goal(position)  # Publish the goal to move_base/goal

            #     # Wait for the robot to reach the goal before proceeding to the next one
            #     rospy.wait_for_message('/move_base/result', MoveBaseActionResult)

            cv2.imshow('Modified Map', color_map)
            print(color_map.shape)
            # cv2.imwrite('result.jpg', color_map)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            rospy.loginfo("Map image not available. Call load_map() first.")

if __name__ == "__main__":
    # Replace these paths with your actual paths
    map_file_path = '/home/jatin/catkin_ws/src/CleaningRobot/cleaning_robot_navigation/maps/gazebo_world.pgm'
    yaml_file_path = '/home/jatin/catkin_ws/src/CleaningRobot/cleaning_robot_navigation/maps/gazebo_world.yaml'

    map_file_path = '/home/jatin/catkin_ws/src/CleaningRobot/cleaning_robot_navigation/maps/house_map.pgm'
    yaml_file_path = '/home/jatin/catkin_ws/src/CleaningRobot/cleaning_robot_navigation/maps/house_map.yaml'

    # Create MapLoader instance
    map_loader = MapLoader(map_file_path, yaml_file_path)

    # Load and display map parameters
    map_loader.load_map()
    map_loader.display_map_parameters()

    # Display modified map
    map_loader.display_modified_map()