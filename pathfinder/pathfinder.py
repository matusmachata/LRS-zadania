import numpy as np
import heapq

from numpy.ma import equal


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
    # Find the first non-empty column from the left
    non_empty_columns = np.any(pgm_map < 128, axis=0)
    left_trim_index = np.argmax(non_empty_columns)
    
    # Find the first non-empty row from the top
    non_empty_rows = np.any(pgm_map < 128, axis=1)
    top_trim_index = np.argmax(non_empty_rows)

    # Crop the map to remove empty space on the left and top edges
    trimmed_map = pgm_map[top_trim_index:, left_trim_index:]
    offset = (left_trim_index, top_trim_index)  # Offset matches (x, y) structure
    return trimmed_map, offset

def adjust_waypoints(waypoints, offset):
    """Adjust waypoints based on the trimming offset."""
    return [(x - offset[0], y - offset[1]) for x, y in waypoints]

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

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4 neighboring points
            neighbor = (current[0] + dx, current[1] + dy)

            # Check map boundaries and avoid obstacles
            if (0 <= neighbor[0] < pgm_map.shape[1]) and (0 <= neighbor[1] < pgm_map.shape[0]):
                if is_obstacle(neighbor, pgm_map):
                    continue

                tentative_g_score = g_score[current] + 1  # Path cost is 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # Return empty list if no path is found

def is_obstacle(position, pgm_map):
    """Checks if the given position is an obstacle considering inflated dimensions."""
    inflate_radius = 4  # Inflate obstacles by 4 pixels
    x, y = position
    for dx in range(-inflate_radius, inflate_radius + 1):
        for dy in range(-inflate_radius, inflate_radius + 1):
            neighbor = (x + dx, y + dy)
            if (0 <= neighbor[0] < pgm_map.shape[1]) and (0 <= neighbor[1] < pgm_map.shape[0]):
                if pgm_map[neighbor[1], neighbor[0]] < 128:  # Adjusted for (x, y)
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
    return total_path[::-1]  # Reverse the path

def parse_waypoints(file_path):
    """Loads waypoints from a file and converts meters to pixels."""
    waypoints = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            x, y = int(float(parts[0]) * 20), int(float(parts[1]) * 20)  # Convert to pixels (1 m = 20 px)
            waypoints.append((x, y))  # Maintain (x, y) format
    return waypoints

def mark_waypoints_on_map(pgm_map, waypoints):
    """Marks waypoints on the map with grey pixels (value 128)."""
    pgm_map = pgm_map.copy()  # Ensure the map is writable
    for (x, y) in waypoints:
        if 0 <= x < pgm_map.shape[1] and 0 <= y < pgm_map.shape[0]:
            pgm_map[y, x] = 128  # Grey color for waypoints
    return pgm_map


def save_map_as_pgm(file_path, pgm_map, path):
    """Saves the PGM map with the path marked as PGM."""
    output_map = pgm_map.copy()

    # Mark the path on the map with black (0)
    for (x, y) in path:
        output_map[y, x] = 0  # Mark path as black

    # Save as PGM
    height, width = output_map.shape
    with open(file_path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f"{width} {height}\n".encode())
        f.write(b'255\n')
        f.write(output_map.tobytes())

def convert_path_to_meters(path):
    """Converts the path coordinates from pixels back to meters."""
    meter_path = [(x / 20.0, y / 20.0) for x, y in path]  # Maintain (x, y) format in meters
    return meter_path

def save_path_as_txt(file_path, meter_paths):
    """Saves each path in meter coordinates to a .txt file with an empty line after each path."""
    with open(file_path, 'w') as f:
        for path in meter_paths:
            for x, y in path:
                f.write(f"{x:.2f},{y:.2f}\n")
            f.write("\n")  # Add an empty line after each path

def main(pgm_file, waypoints_file, output_file):
    pgm_map, offset = load_pgm_map(pgm_file)
    waypoints = parse_waypoints(waypoints_file)
    adjusted_waypoints = adjust_waypoints(waypoints, offset)

    # Mark waypoints on the map
    pgm_map = mark_waypoints_on_map(pgm_map, waypoints)

    all_meter_paths = []
    full_path = []
    # print(len(waypoints))
    for i in range(len(waypoints) - 1):
        # print(i)
        start = waypoints[i]
        goal = waypoints[i + 1]
        if (i == 0):
            start = (int(start[0]) - 200, start[1])
            print(start)
            print(goal)
        if (i == 5):
            goal = (int(goal[0] - 200), int(goal[1]))
            print(start)
            print(goal)
        path = a_star(start, goal, pgm_map)



        if path:
            full_path.extend(path)
            print(f"Path found between points {waypoints[i]} and {waypoints[i + 1]}: {path}")
            meter_path = convert_path_to_meters(path)
            all_meter_paths.append(meter_path)  # Save each path separately
        else:
            print(f"No path exists between points {waypoints[i]} and {waypoints[i + 1]}!")

    # Save the path to the output PGM file
    save_map_as_pgm(output_file, pgm_map, full_path)
    
    # Save all paths with empty lines between them
    save_path_as_txt(path_txt_file, all_meter_paths)

if __name__ == "__main__":
    pgm_file = 'map_1.pgm'  # Input PGM file path
    waypoints_file = 'waypoints.csv'  # CSV file with waypoints
    output_file = 'output.pgm'  # Output PGM file path
    path_txt_file = 'path.txt'
    main(pgm_file, waypoints_file, output_file)

