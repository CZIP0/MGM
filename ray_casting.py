import numpy as np
import random
import matplotlib.pyplot as plt
from math import sin, cos, radians

# Méretek
SIZE = 20

# Négyzetrács inicializálása
grid = np.zeros((SIZE, SIZE))

# Véletlenszerű falak generálása, ahol a falak csak éllel csatlakozhatnak
def generate_walls(grid, wall_count=60):
    def is_valid_wall_position(x, y):
        if grid[x][y] == 1:
            return False
        diagonals = [(x-1, y-1), (x-1, y+1), (x+1, y-1), (x+1, y+1)]
        for dx, dy in diagonals:
            if 0 <= dx < SIZE and 0 <= dy < SIZE and grid[dx][dy] == 1:
                return False
        return True

    for _ in range(wall_count):
        while True:
            x, y = random.randint(1, SIZE-2), random.randint(1, SIZE-2)
            if is_valid_wall_position(x, y):
                grid[x][y] = 1
                break
    return grid

# Bresenham vonal algoritmus
def bresenham(x1, y1, x2, y2):
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy
    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    return points

# Robot inicializálása egy random pozícióban (nem fal és nem a szélén)
def place_robot(grid, existing_positions):
    while True:
        x, y = random.randint(1, SIZE-2), random.randint(1, SIZE-2)
        if grid[x][y] == 0:
            if all(abs(x - ex) > 1 or abs(y - ey) > 1 for ex, ey in existing_positions):
                return x, y

# Szimuláció beállítása
grid = generate_walls(grid)

# Térképezés
def map_environment(grid, robot_x, robot_y):
    detection_map = np.zeros_like(grid)
    lidar_data = {}
    for angle in range(0, 360, 5):
        rad_angle = radians(angle)
        end_x = int(robot_x + SIZE * cos(rad_angle))
        end_y = int(robot_y + SIZE * sin(rad_angle))
        line_points = bresenham(robot_x, robot_y, end_x, end_y)
        distance = 0
        for x, y in line_points:
            if 0 <= x < SIZE and 0 <= y < SIZE:
                distance += 1
                if grid[x][y] == 1:
                    detection_map[x][y] = 1
                    break
            else:
                break
        lidar_data[angle] = distance
    return detection_map, lidar_data

# Többszörös szimuláció
all_lidar_data = []
robot_positions = []
for _ in range(10):
    robot_x, robot_y = place_robot(grid, robot_positions)
    detection_map, lidar_data = map_environment(grid, robot_x, robot_y)
    all_lidar_data.append(lidar_data)
    robot_positions.append((robot_x, robot_y))

# Rajzolás
def plot_grids(grid, robot_positions, detection_maps, occupancy_map):
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))

    # Eredeti térkép
    axs[0].imshow(grid, cmap='gray_r')  # Invert colors
    for robot_x, robot_y in robot_positions:
        axs[0].scatter(robot_y, robot_x, color='green', s=100)
    for detection_map in detection_maps:
        for x in range(SIZE):
            for y in range(SIZE):
                if detection_map[x, y] == 1:
                    axs[0].scatter(y, x, color='red', s=10)
    axs[0].set_title('Eredeti térkép')

    # Occupancy map
    axs[1].imshow(occupancy_map, cmap='gray_r')  # Invert colors
    for robot_x, robot_y in robot_positions:
        axs[1].scatter(robot_y, robot_x, color='green', s=100)
    axs[1].set_title('Occupancy map')

    plt.show()

# Eredmény megjelenítése
detection_maps = [map_environment(grid, x, y)[0] for x, y in robot_positions]

# Occupancy map generálása bresenham algoritmus alapján
occupancy_map = np.ones((SIZE, SIZE))
for lidar_data, (robot_x, robot_y) in zip(all_lidar_data, robot_positions):
    for angle, distance in lidar_data.items():
        rad_angle = radians(angle)
        # Végpont kiszámítása a távolság és a szög alapján
        end_x = int(robot_x + distance * cos(rad_angle))
        end_y = int(robot_y + distance * sin(rad_angle))
        
        # Használjuk a bresenham algoritmust a robot és a végpont között
        line_points = bresenham(robot_x, robot_y, end_x, end_y)
        
        # Vonal pontok kezelése
        for x, y in line_points:
            if 0 <= x < SIZE and 0 <= y < SIZE:
                occupancy_map[x][y] = 0

                if (x, y) == (end_x, end_y):  # Az utolsó pont, ahol az akadály van
                    occupancy_map[x][y] = 1
                    break  # A vonal véget ér az akadálynál

# Ensure robot positions are marked as free in the occupancy map
for robot_x, robot_y in robot_positions:
    occupancy_map[robot_x][robot_y] = 0

plot_grids(grid, robot_positions, detection_maps, occupancy_map)

# Lidar adatok kiírása
for i, lidar_data in enumerate(all_lidar_data):
    print(f"Robot pozíció {i+1}: {robot_positions[i]}")
    for angle, distance in lidar_data.items():
        print(f"Szög: {angle}°, Távolság: {distance}")
