""" 
Detects and normalizes LEGO brick coordinates for robotic arm manipulation.

1. Reads the detected label number, name, and coordinates (YOLOv5 scale) of LEGO bricks 
   from the detection results file.

2. Normalizes the YOLOv5 coordinates by dividing by 640 and scales them to real-world sizes (cm). 
   Converts these values to the robot's coordinate system using the LEGO brick's center.

3. Allows the user to select the desired class ID for the robot to pick up.

4. Displays the selected class name and its coordinates in the robot's coordinate system.
"""

import numpy as np

# Load recognized LEGO color categories and their positions in YOLOv5 from a txt file
def load_results(file_path):
    with open(file_path, 'r') as f:
        return [line.strip().split(',') for line in f]

# Convert recognition results to integers and organize them into a 2D list
def organize_results(result):
    label_num_coord = [[0, 0, 0, 0] for _ in range(4)]
    for i in range(len(result) - 1):
        index_num = int(float(result[-1][i]))
        label_num_coord[index_num] = [int(float(x)) if idx != 0 else x for idx, x in enumerate(result[i])]
    return label_num_coord

# Calculate side lengths and center coordinates
def calculate_dimensions_and_center(label_num_coord):
    dimensions = []
    centers = []
    for i in range(len(label_num_coord)):
        if label_num_coord[i][0] != 0:
            a, b, c, d = label_num_coord[i][1:5]
            short_side, long_side = min(d - b, c - a), max(d - b, c - a)
            center_x, center_y = (a + c) / 2, (b + d) / 2
            dimensions.append((short_side, long_side))
            centers.append((center_x, center_y))
        else:
            dimensions.append((None, None))
            centers.append((None, None))
    return dimensions, centers

# Find the slope and intercept
def find_linear_equation(len_x, len_y):
    slope, intercept = np.polyfit(len_x, len_y, 1)
    return slope, intercept

# Convert real coordinates to robot coordinate system
def real_to_robot_coord(n):
    return 32.3 - n

# Use the linear equation to calculate Y coordinates
def apply_linear_equation(x, slope, intercept):
    return x * slope + intercept

# Main function
def main():
    result = load_results('label_coordinate.txt')
    print("Recognition results:", result)

    label_num_coord = organize_results(result)
    print("Organized label coordinates:", label_num_coord)

    # Calculate the side lengths and center for each LEGO
    dimensions, centers = calculate_dimensions_and_center(label_num_coord)

    # Actual sizes of LEGO (in cm)
    real_sizes = [2.85, 5.8, 8.7]  # PURPLE, PINK & BLUE, GREEN
    YOLO_X_list = [640, 0]
    Real_X_list = [64.6, 0]
    YOLO_Y_list = [640, 0]
    Real_Y_list = [48.5, 0]

    # Calculate slope and intercept
    YOLO_X_Slope, YOLO_X_Intercept = find_linear_equation(YOLO_X_list, Real_X_list)
    YOLO_Y_Slope, YOLO_Y_Intercept = find_linear_equation(YOLO_Y_list, Real_Y_list)

    real_coords = []
    for i in range(4):
        if label_num_coord[i][0] != 0:
            center_x, center_y = centers[i]
            real_x = apply_linear_equation(center_x, YOLO_X_Slope, YOLO_X_Intercept)
            real_y = apply_linear_equation(center_y, YOLO_Y_Slope, YOLO_Y_Intercept)
            real_coords.append((real_x, real_y))
            print(f"{label_num_coord[i][0]} Center coordinates: {real_coords[-1]}")
        else:
            real_coords.append((None, None))

    # Allow user to choose which object to capture
    available_indices = [i for i in range(4) if label_num_coord[i][0] != 0]
    for idx in available_indices:
        print(f"{label_num_coord[idx][0]} : {label_num_coord[idx]}")

    if available_indices:
        index = int(input("Please enter the number of the object to be captured: "))
        if index in available_indices:
            coord = real_coords[index]
            print(f"{label_num_coord[index][0]} : {coord}")
            final_coord = (coord[1], real_to_robot_coord(coord[0]))
            print(f"(X = {final_coord[0]}, Z = {final_coord[1]})")
            return final_coord
        else:
            print("The entered number is incorrect.")
    else:
        print("No objects detected.")
    

if __name__ == "__main__":
    main()
