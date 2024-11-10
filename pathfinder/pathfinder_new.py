import numpy as np
import heapq
import csv

def load_pgm_map(file_path):
    """Loads and preprocesses a PGM map (P2 or P5) file, trimming empty space from the left and top edges."""
    with open(file_path, 'rb') as f:
        header = f.readline().strip()
        if header not in [b'P2', b'P5']:
            raise ValueError("Unsupported format: only binary PGM (P5) and ASCII PGM (P2) supported")
        
        dimensions = f.readline().strip()
        width, height = map(int, dimensions.split())
        
        max_value = int(f.readline().strip())
        if max_value != 255:
            raise ValueError("Only 8-bit PGM files supported")
        
        if header == b'P5':
            pixel_data = f.read()
            pgm_map = np.frombuffer(pixel_data, dtype=np.uint8).reshape((height, width))
        elif header == b'P2':
            pixel_data = []
            for line in f:
                pixel_data.extend(map(int, line.split()))
            pgm_map = np.array(pixel_data, dtype=np.uint8).reshape((height, width))
        
        trimmed_map, offset = trim_empty_edges(pgm_map)
        return trimmed_map, offset

def trim_empty_edges(pgm_map):
    """Trims empty space from the left and top edges of the map and returns the offset of the new origin."""
    non_empty_columns = np.any(pgm_map < 128, axis=0)
    left_trim_index = np.argmax(non_empty_columns)
    
    non_empty_rows = np.any(pgm_map < 128, axis=1)
    top_trim_index = np.argmax(non_empty_rows)

    trimmed_map = pgm_map[top_trim_index:, left_trim_index:]
    offset = (top_trim_index, left_trim_index)
    return trimmed_map, offset

def adjust_waypoints(waypoints, offset):
    """Adjust waypoints based on the trimming offset."""
    return [(y - offset[0], x - offset[1]) for y, x in waypoints]

def inflate_obstacles(pgm_map, inflation_radius):
    """Inflates obstacles in the map by the given radius."""
    inflated_map = pgm_map.copy()
    for i in range(pgm_map.shape[0]):
        for j in range(pgm_map.shape[1]):
            if pgm_map[i, j] < 128:  # Obstacle
                for dx in range(-inflation_radius, inflation_radius + 1):
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        ni, nj = i + dx, j + dy
                        if 0 <= ni < pgm_map.shape[0] and 0 <= nj < pgm_map.shape[1]:
                            inflated_map[ni, nj] = 0  # Mark inflated obstacle
    return inflated_map

def a_star(start, goal, pgm_map):
    """A* algorithm for finding the shortest path."""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < pgm_map.shape[0] and 0 <= neighbor[1] < pgm_map.shape[1]:
                if is_obstacle(neighbor, pgm_map):
                    continue

                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # Return empty list if no path is found

def is_obstacle(position, pgm_map):
    """Checks if the given position is an obstacle."""
    x, y = position
    if pgm_map[x, y] < 128:  # Check if obstacle (assuming obstacle value is less than 128)
        return True
    return False

def heuristic(a, b):
    """Calculates the Manhattan distance heuristic for A*."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    """Reconstructs the path from the goal to the start."""
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]

def parse_waypoints(file_path):
    """Loads waypoints from a file and converts meters to pixels."""
    waypoints = []
    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            x, y = int(float(row[0]) * 20), int(float(row[1]) * 20)  # Convert to pixels
            waypoints.append((y, x))  # y = height, x = width (order corrected)
    return waypoints

def inflate_waypoint(waypoint, inflation_radius, pgm_map):
    """Inflates the waypoint by marking a square area around it."""
    inflated_waypoints = []
    x, y = waypoint  # Correct order: x is width, y is height
    for dy in range(-inflation_radius, inflation_radius + 1):
        for dx in range(-inflation_radius, inflation_radius + 1):
            ni, nj = y + dy, x + dx  # Correct application of x (width) and y (height)
            if 0 <= ni < pgm_map.shape[0] and 0 <= nj < pgm_map.shape[1]:
                inflated_waypoints.append((ni, nj))
    return inflated_waypoints

def save_map_as_pgm(file_path, pgm_map, path, waypoints, offset, inflation_radius=2):
    """Saves the PGM map with the path and inflated waypoints marked, applying the offset for consistency."""
    output_map = pgm_map.copy()
    
    # Mark the path
    for (y, x) in path:
        adjusted_y, adjusted_x = y + offset[0], x + offset[1]  # Apply offset to the path coordinates
        if 0 <= adjusted_y < pgm_map.shape[0] and 0 <= adjusted_x < pgm_map.shape[1]:
            output_map[adjusted_y, adjusted_x] = 0  # Mark path as black

    # Mark the inflated waypoints
    for waypoint in waypoints:
        adjusted_waypoint = (waypoint[0] + offset[0], waypoint[1] + offset[1])  # Apply offset to waypoints
        inflated_points = inflate_waypoint(adjusted_waypoint, inflation_radius, pgm_map)
        for (y, x) in inflated_points:
            if 0 <= y < pgm_map.shape[0] and 0 <= x < pgm_map.shape[1]:
                output_map[y, x] = 128  # Mark waypoints as a different color (e.g., gray)

    height, width = output_map.shape
    with open(file_path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f"{width} {height}\n".encode())
        f.write(b'255\n')
        f.write(output_map.tobytes())

def save_path_as_txt(file_path, meter_paths):
    """Saves each path in meter coordinates to a .txt file with an empty line after each path."""
    with open(file_path, 'w') as f:
        for path in meter_paths:
            for x, y in path:
                f.write(f"{x:.2f},{y:.2f}\n")
            f.write("\n")

def convert_path_to_meters(path):
    """Converts the path coordinates from pixels back to meters."""
    return [(x / 20.0, y / 20.0) for y, x in path]  # Reversed order to meters

def main(pgm_file, waypoints_file, output_file):
    pgm_map, offset = load_pgm_map(pgm_file)
    waypoints = parse_waypoints(waypoints_file)
    adjusted_waypoints = adjust_waypoints(waypoints, offset)

    inflated_map = inflate_obstacles(pgm_map, inflation_radius=4)

    all_meter_paths = []
    full_path = []
    for i in range(len(adjusted_waypoints) - 1):
        start = adjusted_waypoints[i]
        goal = adjusted_waypoints[i + 1]
        path = a_star(start, goal, inflated_map)

        if path:
            full_path.extend(path)
            print(f"Path found between points {waypoints[i]} and {waypoints[i + 1]}: {path}")
            meter_path = convert_path_to_meters(path)
            all_meter_paths.append(meter_path)
        else:
            print(f"No path exists between points {waypoints[i]} and {waypoints[i + 1]}!")

    # Save the map with the path and waypoints
    # Save the map with the path and waypoints
    save_map_as_pgm(output_file, inflated_map, full_path, adjusted_waypoints, offset)

    
    # Save the paths as a .txt file
    save_path_as_txt(output_file.replace('.pgm', '.txt'), all_meter_paths)

    print("Process completed.")

if __name__ == "__main__":
    pgm_file = 'map_1.pgm'  # Input PGM file path
    waypoints_file = 'waypoints.csv'  # CSV file with waypoints
    output_file = 'output.pgm'  # Output PGM file path
    main(pgm_file, waypoints_file, output_file)

