import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import deque
from scipy.ndimage import binary_dilation

def load_pgm(filename):
    with open(filename, 'r') as f:
        # Read header
        assert f.readline().strip() == 'P2', "Only P2 PGM format is supported."
        # Skip comments
        line = f.readline()
        while line.startswith('#'):
            line = f.readline()
        # Read width and height
        width, height = map(int, line.strip().split())
        # Read max gray value
        max_gray = int(f.readline().strip())
        # Read pixel data
        pixels = []
        for line in f:
            pixels.extend([int(i) for i in line.strip().split()])
            if len(pixels) >= width * height:
                break
        image = np.array(pixels).reshape((height, width))
    return image, max_gray

def save_pgm(filename, image, max_gray):
    height, width = image.shape
    with open(filename, 'w') as f:
        f.write('P2\n')
        f.write(f'{width} {height}\n')
        f.write(f'{max_gray}\n')
        for row in image:
            row_values = ' '.join(map(str, row.tolist()))
            f.write(f'{row_values}\n')

def trim_empty_edges(pgm_map):
    """Trims empty space from the left, top, right, and bottom edges of the map and returns the offset."""
    non_empty_columns = np.any(pgm_map < 128, axis=0)
    left_trim_index = np.argmax(non_empty_columns)
    right_trim_index = len(non_empty_columns) - np.argmax(non_empty_columns[::-1]) - 1

    non_empty_rows = np.any(pgm_map < 128, axis=1)
    top_trim_index = np.argmax(non_empty_rows)
    bottom_trim_index = len(non_empty_rows) - np.argmax(non_empty_rows[::-1]) - 1

    # Crop the map to remove empty space on all edges
    trimmed_map = pgm_map[top_trim_index:bottom_trim_index + 1, left_trim_index:right_trim_index + 1]
    offset = (left_trim_index, top_trim_index)
    return trimmed_map, offset

def load_waypoints(csv_filename):
    # Load waypoints from CSV without headers, and assign column names
    column_names = ['X', 'Y', 'Z', 'Proximity', 'Action']
    df = pd.read_csv(csv_filename, header=None, names=column_names)
    return df

def bresenham_line(start, end):
    """Generates the points along a line from start to end using Bresenham's algorithm."""
    x0, y0 = start[1], start[0]  # Note: columns correspond to x, rows correspond to y
    x1, y1 = end[1], end[0]
    points = []

    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy  # error value e_xy

    while True:
        points.append((y0, x0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return points

def is_straight_line_clear(start, goal, grid):
    """Checks if the straight line between start and goal is unobstructed."""
    line_points = bresenham_line(start, goal)
    for point in line_points:
        row, col = point
        if grid[row, col]:
            return False  # Obstacle detected
    return True

def bfs_pathfinding(grid, start, goal, proximity_radius):
    # Perform BFS to find a path to within proximity_radius of the goal
    queue = deque()
    queue.append((start, [start]))
    visited = set()
    visited.add(start)
    goal_row, goal_col = goal
    while queue:
        (current, path) = queue.popleft()
        row, col = current
        # Check if current position is within proximity_radius of goal
        distance = np.hypot(row - goal_row, col - goal_col)
        if distance <= proximity_radius:
            return path
        neighbors = [
            (row + 1, col), (row - 1, col),
            (row, col + 1), (row, col - 1),
            (row + 1, col + 1), (row - 1, col - 1),
            (row + 1, col - 1), (row - 1, col + 1)
        ]
        for n_row, n_col in neighbors:
            if 0 <= n_row < grid.shape[0] and 0 <= n_col < grid.shape[1]:
                if not grid[n_row, n_col] and (n_row, n_col) not in visited:
                    queue.append(((n_row, n_col), path + [(n_row, n_col)]))
                    visited.add((n_row, n_col))
    return None  # No path found

def save_all_paths(paths, filename):
    with open(filename, 'w') as f:
        for path in paths:
            for point in path:
                f.write(f"{point[0]}, {point[1]}\n")
            f.write('\n')  # Empty line between paths

def main():
    # Step 1: Load PGM file
    pgm_filename = 'map.pgm'
    image, max_gray = load_pgm(pgm_filename)

    # Step 2: Trim edges using the provided function
    image, (left_offset, top_offset) = trim_empty_edges(image)

    # Save the dimensions after initial trimming
    trimmed_height, trimmed_width = image.shape

    # Update max_gray if trimming affects pixel values
    max_gray = image.max()

    # Step 3: Load waypoints
    waypoints_df = load_waypoints('waypoints.csv')

    # Convert waypoints from meters to grid coordinates
    scale = 0.05  # Each pixel represents 0.05m
    waypoints = []
    proximities = []
    for index, wp in waypoints_df.iterrows():
        x_meters, y_meters = wp['X'], wp['Y']
        col = int(round(x_meters / scale))
        row = int(round(y_meters / scale))
        # Adjust row to match image coordinate system (invert Y-axis)
        row = trimmed_height - row
        waypoints.append((row, col))
        proximities.append(wp['Proximity'])

    # Adjust waypoints for initial trimming
    waypoints = [(row - top_offset, col - left_offset) for row, col in waypoints]

    # Validate waypoints are within the map bounds
    image_height, image_width = image.shape
    valid_waypoints = []
    valid_proximities = []
    for waypoint, proximity in zip(waypoints, proximities):
        row, col = waypoint
        if 0 <= row < image_height and 0 <= col < image_width:
            valid_waypoints.append((row, col))
            valid_proximities.append(proximity)
        else:
            print(f"Waypoint {waypoint} is out of bounds after processing and will be skipped.")
    waypoints = valid_waypoints
    proximities = valid_proximities

    if len(waypoints) < 2:
        print("Not enough valid waypoints to calculate paths.")
        return

    # Step 4: Create inflated obstacle grid for path planning
    obstacle_grid = (image == 0)
    inflation_radius = 6
    structuring_element = np.ones((2 * inflation_radius + 1, 2 * inflation_radius + 1), dtype=bool)
    inflated_obstacle_grid = binary_dilation(obstacle_grid, structure=structuring_element)

    # Step 5: Calculate paths between waypoints
    paths = []
    for i in range(len(waypoints)-1):
        start = waypoints[i]
        goal = waypoints[i+1]
        proximity = proximities[i+1]  # Use the proximity of the goal waypoint
        # Determine proximity radius in pixels
        if isinstance(proximity, str):
            proximity = proximity.lower()
        if proximity == 'soft':
            proximity_radius_pixels = 6  # 30 cm
        elif proximity == 'hard':
            proximity_radius_pixels = 3  # 15 cm
        else:
            print(f"Unknown proximity '{proximity}' at waypoint {i+1}, defaulting to 'hard' (15 cm).")
            proximity_radius_pixels = 3

        # Check if straight line path is clear
        if is_straight_line_clear(start, goal, inflated_obstacle_grid):
            path = [start, goal]
        else:
            # Perform pathfinding using BFS
            path = bfs_pathfinding(inflated_obstacle_grid, start, goal, proximity_radius_pixels)
            if path is None:
                print(f"No path found between waypoint {i} and {i+1}.")
                continue
        paths.append(path)

    # Step 6: Save all paths to one text file with empty lines between paths
    save_all_paths(paths, 'all_paths.txt')

    # Step 7: Draw paths on the map (do not draw waypoints)
    # Create a copy of the image to draw on
    output_image = image.copy()

    # Define pixel value for paths
    path_value = int(max_gray * 0.5)  # Mid-gray value

    # Draw paths
    for path in paths:
        for row, col in path:
            if 0 <= row < output_image.shape[0] and 0 <= col < output_image.shape[1]:
                output_image[row, col] = path_value

    # Save the output image with paths to 'output.pgm'
    output_map_filename = 'output.pgm'
    save_pgm(output_map_filename, output_image.astype(int), int(max_gray))

    # Optional: Display the map with paths
    plt.imshow(output_image, cmap='gray', origin='upper')
    plt.title('Map with Paths')
    plt.show()

if __name__ == '__main__':
    main()
