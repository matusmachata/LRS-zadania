import numpy as np
import heapq
from numpy.ma import equal
from scipy.constants import point


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

        rotated_map = np.rot90(pgm_map, 2)
        trimmed_map, offset = trim_empty_edges(rotated_map)
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

def mark_waypoints_on_map(pgm_map, waypoints):
    """Marks waypoints on the map with grey pixels (value 128)."""
    pgm_map = pgm_map.copy()  # Ensure the map is writable
    for (x, y) in waypoints:
        if 0 <= x < pgm_map.shape[1] and 0 <= y < pgm_map.shape[0]:
            pgm_map[y, x] = 128  # Grey color for waypoints
            pgm_map[y + 1, x + 1] = 128
            pgm_map[y + 1, x - 1] = 128
            pgm_map[y - 1, x + 1] = 128
            pgm_map[y - 1, x - 1] = 128
    return pgm_map


def parse_waypoints(file_path):
    """Loads waypoints from a file and converts meters to pixels."""
    waypoints = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            x, y = int(float(parts[0]) * 20), int(float(parts[1]) * 20)  # Convert to pixels (1 m = 20 px)
            waypoints.append((y, x))  # Maintain (x, y) format
    return waypoints

def save_map_as_pgm(file_path, pgm_map):
    """Saves the PGM map with the path marked as PGM."""
    output_map = pgm_map.copy()


    # Save as PGM
    height, width = output_map.shape
    with open(file_path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f"{width} {height}\n".encode())
        f.write(b'255\n')
        f.write(output_map.tobytes())

def main(pgm_file, waypoints_file, output_file):
    waypoints = parse_waypoints(waypoints_file)
    print(waypoints)

    pgm_map, offset = load_pgm_map(pgm_file)
    print(pgm_map.shape)
    pgm_map = mark_waypoints_on_map(pgm_map, waypoints)

    save_map_as_pgm(output_file, pgm_map)



if __name__ == "__main__":
    pgm_file = 'map.pgm'  # Input PGM file path
    waypoints_file = 'waypoints.csv'  # CSV file with waypoints
    output_file = 'output.pgm'  # Output PGM file path
    path_txt_file = 'path.txt'
    main(pgm_file, waypoints_file, output_file)