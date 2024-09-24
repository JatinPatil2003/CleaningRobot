#!/usr/bin/env python3
import math

def calculate_total_distance(file_path):
    coordinates = []
    
    # Read the file and extract the coordinates
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line:  # Check if the line is not empty
                # Split the line by comma and convert to float
                try:
                    coord = list(map(float, line[1:-1].split(',')))
                    coordinates.append(coord)
                except ValueError:
                    print(f"Skipping invalid line: {line}")
    
    if len(coordinates) < 2:
        print("Not enough valid points to calculate distance.")
        return 0
    
    total_distance = 0.0
    
    # Loop through each consecutive pair of points
    for i in range(1, len(coordinates)):
        x1, y1 = coordinates[i - 1]
        x2, y2 = coordinates[i]
        # Calculate Euclidean distance between consecutive points
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        total_distance += distance
    
    return round(total_distance, 2)

def get_time(file_path):
    count = 0
    with open(file_path, 'r') as file:
        for line in file:
            count += 1
    return count * 0.5

tool_radius = 0.225

# coverage_path_file = 'coverage_path.txt'
# robot_path_file = 'robot_path.txt'
# t1 = 1726935885.393285920
# t2 = 1726935885.393508833

coverage_path_file = 'coverage_path_1.txt'
robot_path_file = 'robot_path_1.txt'
t1 = 1727089597.203518175
t2 = 1727089597.203663152

coverage_distance = calculate_total_distance(coverage_path_file)
robot_distance = calculate_total_distance(robot_path_file)

coverage_area = round(coverage_distance * 2 * tool_radius, 2)
robot_area = round(robot_distance * 2 * tool_radius, 2)



print(f"\nCoverage Planner Disatnce: {coverage_distance} m")
print(f"Robot Covered Disatnce: {robot_distance} m")

print(f"\nCoverage Planner Area: {coverage_area} m^2")
print(f"Robot Covered Area: {robot_area} m^2")

print(f"\nTime Required to Compute Coverage Plan: {round((t2 - t1) * 1000, 3)} ms")
print(f"Time Required to Clean: {get_time(robot_path_file)} seconds\n")