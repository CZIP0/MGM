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
        # Ellenőrizzük, hogy a négyzet sarkai nem érintkeznek-e más falakkal
        if grid[x][y] == 1:
            return False  # Ha már fal van itt, akkor nem rakunk új falat
        # Ellenőrizzük a diagonális szomszédokat (sarkok)
        diagonals = [(x-1, y-1), (x-1, y+1), (x+1, y-1), (x+1, y+1)]
        for dx, dy in diagonals:
            if 0 <= dx < SIZE and 0 <= dy < SIZE and grid[dx][dy] == 1:
                return False
        return True

    for _ in range(wall_count):
        while True:
            x, y = random.randint(1, SIZE-2), random.randint(1, SIZE-2)
            if is_valid_wall_position(x, y):
                grid[x][y] = 1  # 1-es érték a fal
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
def place_robot(grid):
    while True:
        x, y = random.randint(1, SIZE-2), random.randint(1, SIZE-2)
        if grid[x][y] == 0:  # Csak olyan helyre teheti a robotot, ahol nincs fal
            return x, y

# Szimuláció beállítása
grid = generate_walls(grid)
robot_x, robot_y = place_robot(grid)

# Térképezés
def map_environment(grid, robot_x, robot_y):
    detection_map = np.zeros_like(grid)
    for angle in range(0, 360, 5):  # 5 fokonként mérünk
        rad_angle = radians(angle)
        end_x = int(robot_x + SIZE * cos(rad_angle))
        end_y = int(robot_y + SIZE * sin(rad_angle))
        line_points = bresenham(robot_x, robot_y, end_x, end_y)
        found_wall = False  # Nyomon követjük, hogy találkoztunk-e már falnak
        for x, y in line_points:
            if 0 <= x < SIZE and 0 <= y < SIZE:
                if grid[x][y] == 1:  # Falat találtunk
                    detection_map[x][y] = 1
                    found_wall = True  # Jelöljük, hogy falat találtunk
                    break  # Azonnal megállunk a fal első érzékelése után
                if found_wall:  # Ha már falat találtunk, további részek nem számítanak
                    break
            else:
                break  # Ha kilépünk a térképből, megállunk
    return detection_map

# Környezet feltérképezése
detection_map = map_environment(grid, robot_x, robot_y)

# Rajzolás
def plot_grid(grid, robot_x, robot_y, detection_map):
    plt.imshow(grid, cmap='gray')
    plt.scatter(robot_y, robot_x, color='green', s=100)  # Robot jelölése
    for x in range(SIZE):
        for y in range(SIZE):
            if detection_map[x, y] == 1:
                plt.scatter(y, x, color='red', s=10)  # Falérzékelések jelölése
    plt.show()

# Eredmény megjelenítése
plot_grid(grid, robot_x, robot_y, detection_map)
