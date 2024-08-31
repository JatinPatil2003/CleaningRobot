#!/usr/bin/env python3

import cv2
import yaml
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult


class MapLoader:
    def __init__(self, map_file, yaml_file):
        rospy.init_node("Coverage_Planner_Node")
        self.map_file = map_file
        self.yaml_file = yaml_file
        self.map_data = None
        self.map_image = None
        self.x_offset = 0.15
        self.y_offset = 0.7
        self.move_base_goal_pub = rospy.Publisher(
            "/move_base/goal", MoveBaseActionGoal, queue_size=1
        )
        rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.move_base_result_callback
        )

    def load_map(self):
        with open(self.yaml_file, "r") as file:
            self.map_data = yaml.safe_load(file)
        self.map_image = cv2.imread(self.map_file, cv2.IMREAD_GRAYSCALE)

    def display_map_parameters(self):
        if self.map_data is not None:
            rospy.loginfo("Map Parameters:")
            for key, value in self.map_data.items():
                rospy.loginfo(f"{key}: {value}")
        else:
            rospy.loginfo("Map parameters not available. Call load_map() first.")

    def pixel_to_map_coordinates(self, x_pixel, y_pixel):
        resolution = self.map_data["resolution"]
        origin = self.map_data["origin"]

        y_map = -(origin[0] + (y_pixel * resolution) + self.y_offset)
        x_map = origin[1] + (x_pixel * resolution) + self.x_offset

        return x_map, y_map

    def distance_to_nearest_black(self, y, x, radius=7):
        black_mask = self.map_image < 250
        y_range, x_range = self.map_image.shape

        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                if (
                    0 <= y + i < y_range
                    and 0 <= x + j < x_range
                    and black_mask[y + i, x + j]
                ):
                    return False  # Too close to a black pixel

        return True

    def publish_goal(self, position):
        goal_msg = MoveBaseActionGoal()
        goal_msg.goal.target_pose.header.frame_id = "map"
        goal_msg.goal.target_pose.pose.position.x = position[0]
        goal_msg.goal.target_pose.pose.position.y = position[1]
        goal_msg.goal.target_pose.pose.orientation.w = (
            1.0  # Assuming you want to keep the orientation constant
        )

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

            black_mask = self.map_image < 250

            color_map[black_mask] = [0, 0, 0]  # Set black

            white_mask = self.map_image >= 250

            goal_positions = []
            goal_pixel_positions = []

            goal_index = 1
            x_range = range(0, self.map_image.shape[1], 10)
            y_range = range(0, self.map_image.shape[0], 10)
            max_index = len(y_range) * len(x_range)
            # Get the shape of the resulting array
            result_shape = (len(x_range), len(y_range), 4)

            # Initialize the result array
            result_array = np.ones(result_shape, dtype=np.int64)
            print(len(y_range), len(x_range), max_index)
            print("Shape of result_array:", result_array.shape, result_shape)

            number_count = 1

            for x_ind, x in enumerate(x_range):  # Adjust the step size as needed
                # Determine whether to iterate from top to bottom or bottom to top based on x-coordinate
                # y_range = y_range if x % 20 == 0 else reversed(y_range)

                for y_ind, y in enumerate(y_range):  # Adjust the step size as needed
                    if white_mask[y, x] and self.distance_to_nearest_black(y, x):
                        if goal_index == 1:
                            color_map[y, x] = [255, 0, 0]  # Blue for the first red dot
                            print('start', x_ind, y_ind, number_count)
                        elif goal_index == max_index:
                            color_map[y, x] = [0, 255, 0]  # Green for the last red dot
                        else:
                            color_map[y, x] = [0, 0, 255]  # Red for other red dots

                        result_array[x_ind, y_ind, 0] = number_count
                        result_array[x_ind, y_ind, 1] = x
                        result_array[x_ind, y_ind, 2] = y
                        result_array[x_ind, y_ind, 3] = 255
                        img = color_map
                        self.map = color_map
                        goal_pixel_positions.append(((x, y)))
                        x_map, y_map = self.pixel_to_map_coordinates(x, y)
                        goal_positions.append(
                            ((round(x_map, 2), round(y_map, 2)), goal_index)
                        )  # Store coordinates and goal index
                        goal_index += 1
                    else:
                        result_array[x_ind, y_ind, 0] = number_count
                        result_array[x_ind, y_ind, 1] = x
                        result_array[x_ind, y_ind, 2] = y
                        result_array[x_ind, y_ind, 3] = 0
                    number_count += 1
            # print(result_array[1:10,:])
                    
            print('goal index', goal_index)
            # result = self.spiral_dfs_with_blocked(result_array, (4,3))

            self.get_path(result_array)

            goal_pixel_positions = self.goal_points
            
            print(goal_pixel_positions)

            # Draw green lines between consecutive red dots
            for i in range(1, len(goal_pixel_positions)):
                cv2.line(
                    img,
                    (int(goal_pixel_positions[i - 1][0]), int(goal_pixel_positions[i - 1][1])),
                    (int(goal_pixel_positions[i][0]), int(goal_pixel_positions[i][1])),
                    (255, 0, 0),
                    1
                )

                cv2.circle(color_map,(int(goal_pixel_positions[i - 1][0]),int(goal_pixel_positions[i - 1][1])),radius=1,color=(0, 0, 255),thickness=-1)

            rospy.loginfo("Goal Positions (Map Coordinates):")
            # for position, index in goal_positions:
            #     rospy.loginfo(f"Goal {index}: {position}")
            #     self.publish_goal(position)  # Publish the goal to move_base/goal

            #     # Wait for the robot to reach the goal before proceeding to the next one
            #     rospy.wait_for_message('/move_base/result', MoveBaseActionResult)

            cv2.imshow("Modified Map", img)
            print(img.shape)
            # cv2.imwrite('result.jpg', color_map)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            rospy.loginfo("Map image not available. Call load_map() first.")


    def spiral_scan_with_blocked(self, matrix, start_node=(0, 0), blocked_value=0):
        result = []
        cols, rows, _ = matrix.shape
        col, row = start_node
        print(rows, cols, row, col)
        
        while True:
            # Move elements from top col
            # print(matrix[col, row, 1])
            while row < rows and matrix[row, col, 1] != blocked_value:
                result.append(matrix[row, col, 0])
                matrix[row, col, 1] = blocked_value
                # print(matrix[row, col])
                print('1')
                row += 1
            
            row -= 1
            col += 1
            
            if col >= cols or matrix[row, col, 1] == blocked_value:
                break
            
            # Move elements from rightmost rowumn
            while col < cols and matrix[row, col, 1] != blocked_value:
                result.append(matrix[row, col, 0])
                matrix[row, col, 1] = blocked_value
                col += 1
                print('2')
            col -= 1
            row -= 1
            
            if row < 0 or matrix[row, col, 1] == blocked_value:
                break
            
            # Move elements from bottom col
            while row >= 0 and matrix[row, col, 1] != blocked_value:
                result.append(matrix[row, col, 0])
                matrix[row, col, 1] = blocked_value
                row -= 1
                print('3')
            row += 1
            col -= 1
            
            if col < 0 or matrix[row, col, 1] == blocked_value:
                break
            
            # Move elements from leftmost rowumn
            while col >= 0 and matrix[row, col, 1] != blocked_value:
                result.append(matrix[row, col, 0])
                matrix[row, col, 1] = blocked_value
                col -= 1
                print('4')
            col += 1
            row += 1
            
            if row >= rows or matrix[row, col, 1] == blocked_value:
                break
            
        return result
    
    def spiral_dfs_with_blocked(self, matrix, start_node=(0, 0), blocked_value=0):
        result = []
        cols, rows, _ = matrix.shape
        col, row = start_node
        visited = set()

        def is_valid_move(x, y):
            return 0 <= x < cols and 0 <= y < rows and matrix[y, x, 1] != blocked_value and (x, y) not in visited

        def dfs(x, y, direction):
            visited.add((x, y))
            result.append((matrix[y, x, 0],y,x))

            # Define the clockwise directions (right, down, left, up)
            directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
            next_direction = (direction + 1) % 4

            for _ in range(4):
                dx, dy = directions[direction]
                nx, ny = x + dx, y + dy

                if is_valid_move(nx, ny):
                    dfs(nx, ny, next_direction)
                direction = (direction + 1) % 4

        dfs(col, row, 0)
        return result

    def get_path(self, matrix):
        self.goal_points = []
        self.matrix = matrix
        self.generate()

    def generate(self):
        x, y = self.get_index()
        print(x,y)
        if x is None and y is None:
            return 
        self.goal_points.append((self.matrix[x, y, 1], self.matrix[x, y, 2]))
        self.mark_visited(x,y)
        
        # print(self.matrix[x,y,0])
        # print(self.matrix[:,0,:])
        # print(self.matrix[1,7,:])
        i = 1
        while True:
            # print(self.matrix[x, y-5:y+5])
            # print(self.matrix[x, y+i, 3])
            if self.matrix[x, y+i, 3] == 255:
                y += i
                if x > 0 and self.matrix[x-1, y, 3] == 255:
                    return self.generate()
                self.goal_points.append((self.matrix[x, y, 1], self.matrix[x, y, 2]))
                self.mark_visited(x,y)
                print(x,y)
            else:
                # print(self.matrix[x+1, y, 3])
                if self.matrix[x+1, y, 3] == 255:
                    x += 1
                    self.goal_points.append((self.matrix[x+1, y, 1], self.matrix[x+1, y, 2]))
                    self.mark_visited(x,y)
                    print(x,y)
                    if i == 1:
                        i = -1
                    else:
                        i = 1
                else:
                    return self.generate()
            
            goal_pixel_positions = self.goal_points
            for k in range(1, len(goal_pixel_positions)):
                cv2.line(
                    self.map,
                    (
                        int(goal_pixel_positions[k - 1][0]),
                        int(goal_pixel_positions[k - 1][1]),
                    ),
                    (int(goal_pixel_positions[k][0]), int(goal_pixel_positions[k][1])),
                    (0, 255, 0),
                    1,
                )

                # cv2.circle(self.map,(int(goal_pixel_positions[k - 1][0]),int(goal_pixel_positions[k - 1][1]),),radius=1,color=(0, 0, 255),thickness=-1,
                # )  # -1 for filled circle
            cv2.imshow("Modified Maps", self.map)
            # cv2.imwrite('result.jpg', color_map)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        return

    def mark_visited(self, x, y):
        self.matrix[x,y,3] = 0


    def get_index(self):
        for i, array in enumerate(self.matrix):
            for j, value in enumerate(array):
                if value[3] == 255:
                    return i, j
        
        print("finished")
        return None, None
    
    def check_valid(self, x,y):
        if x >= 0 and x <= self.matrix.shape[1] or  y >= 0 and y <= self.matrix.shape[0]:
            if self.matrix[x, y, 1] == 255:
                return True
        return False

if __name__ == "__main__":
    # Replace these paths with your actual paths
    map_file_path = "/home/jatin/catkin_ws/src/CleaningRobot/cleaning_robot_navigation/maps/gazebo_world.pgm"
    yaml_file_path = "/home/jatin/catkin_ws/src/CleaningRobot/cleaning_robot_navigation/maps/gazebo_world.yaml"

    # map_file_path = '/home/jatin/catkin_ws/src/CleaningRobot/cleaning_robot_navigation/maps/L412.pgm'
    # yaml_file_path = '/home/jatin/catkin_ws/src/CleaningRobot/cleaning_robot_navigation/maps/L412.yaml'

    # Create MapLoader instance
    map_loader = MapLoader(map_file_path, yaml_file_path)

    # Load and display map parameters
    map_loader.load_map()
    map_loader.display_map_parameters()

    # Display modified map
    map_loader.display_modified_map()
