import numpy as np
import heapq

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
    offset = (top_trim_index, left_trim_index)
    return trimmed_map, offset

def adjust_waypoints(waypoints, offset):
    """Adjust waypoints based on the trimming offset."""
    return [(y - offset[0], x - offset[1]) for y, x in waypoints]

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
            if (0 <= neighbor[0] < pgm_map.shape[0]) and (0 <= neighbor[1] < pgm_map.shape[1]):
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
            if (0 <= neighbor[0] < pgm_map.shape[0]) and (0 <= neighbor[1] < pgm_map.shape[1]):
                if pgm_map[neighbor] < 128:  # Assume values < 128 are obstacles
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
            waypoints.append((y, x))  # Switch x and y
    return waypoints

def main(pgm_file, waypoints_file, output_file):
    pgm_map, offset = load_pgm_map(pgm_file)
    waypoints = parse_waypoints(waypoints_file)
    adjusted_waypoints = adjust_waypoints(waypoints, offset)

    full_path = []
    for i in range(len(adjusted_waypoints) - 1):
        start = adjusted_waypoints[i]
        goal = adjusted_waypoints[i + 1]
        path = a_star(start, goal, pgm_map)

        if path:
            full_path.extend(path)
            print(f"Path found between points {waypoints[i]} and {waypoints[i + 1]}: {path}")
        else:
            print(f"No path exists between points {waypoints[i]} and {waypoints[i + 1]}!")

    # Save the path to the output PGM file
    save_map_as_pgm(output_file, pgm_map, full_path)

if __name__ == "__main__":
    pgm_file = 'map.pgm'  # Input PGM file path
    waypoints_file = 'waypoints.csv'  # CSV file with waypoints
    output_file = 'output.pgm'  # Output PGM file path
    main(pgm_file, waypoints_file, output_file)

