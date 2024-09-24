#!/usr/bin/env python3
import matplotlib.pyplot as plt

def plot_coordinates(file_paths):
    # Define the colors for the two files
    colors = ['red', 'green']
    lables = ['Coverage Path', 'Robot Path']
    linestyle = ['-', '--']
    
    for idx, file_path in enumerate(file_paths):
        coordinates = []
        
        # Read the file and extract the coordinates
        with open(file_path, 'r') as file:
            for line in file:
                line = line.strip()
                if line:  # Check if the line is not empty
                    
                    try:
                        coord = list(map(float, line[1:-1].split(',')))
                        coordinates.append(coord)
                    except ValueError:
                        print(f"Skipping invalid line in {file_path}: {line}")
                else:
                    if len(coordinates) < 2:
                        print(f"Not enough valid points in {file_path} to plot.")
                        continue

                    # Separate the coordinates into X and Y lists
                    x_coords = [coord[0] for coord in coordinates]
                    y_coords = [-coord[1] for coord in coordinates]
                    coordinates = []
                    # Plot the points with a dotted line and the specific color
                    plt.plot(y_coords, x_coords, linestyle=linestyle[idx], color=colors[idx], label=lables[idx])   

    
    # Add labels and title
    # plt.xlabel('X Coordinate')
    # plt.ylabel('Y Coordinate')
    plt.title('Plot of Coverage Path and Robot Path')
    # plt.legend()
    
    # Show the plot
    # plt.grid(True)
    plt.show()

# Example usage coverage_path_1
file_paths = ['coverage_path.txt', 'robot_path.txt']  # Update these to your actual file paths
file_paths = ['coverage_path_1.txt', 'robot_path_1.txt']  # Update these to your actual file paths
plot_coordinates(file_paths)
