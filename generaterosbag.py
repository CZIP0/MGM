import numpy as np
import random
from math import sin, cos, radians
import rospy
import rosbag
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from time import time

# Méretek
SIZE = 20
CM_PER_GRID_UNIT = 100  # 1 grid unit = 100 cm

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

# Robot inicializálása egy random pozícióban (nem fal és nem a szélén)
def place_robot(grid):
    while True:
        x, y = random.randint(1, SIZE-2), random.randint(1, SIZE-2)
        if grid[x][y] == 0:
            return x, y

# Szimuláció beállítása
grid = generate_walls(grid)

# Térképezés
def map_environment(grid, robot_x, robot_y):
    lidar_data = LaserScan()
    lidar_data.header = Header(stamp=rospy.Time.from_sec(time()), frame_id="lidar_frame")
    lidar_data.angle_min = 0.0
    lidar_data.angle_max = 2 * np.pi
    lidar_data.angle_increment = np.pi / 180  # 1 degree in radians
    lidar_data.range_min = 0.0
    lidar_data.range_max = float(SIZE)  # Maximum range in meters
    lidar_data.ranges = []

    for angle in range(360):
        rad_angle = radians(angle)
        distance = 0.0
        while True:
            end_x = robot_x + distance * cos(rad_angle)
            end_y = robot_y + distance * sin(rad_angle)
            grid_x, grid_y = int(end_x), int(end_y)
            if 0 <= grid_x < SIZE and 0 <= grid_y < SIZE:
                if grid[grid_x][grid_y] == 1:
                    break
                distance += 0.01  # Increment by 1 cm
            else:
                break
        lidar_data.ranges.append(distance)  # Export in meters
    return lidar_data 

# ROS node inicializálása
rospy.init_node('robot_simulation')

# Véletlenszerű robot pozíció
robot_x, robot_y = place_robot(grid)

# Odometria üzenet
odom_msg = Odometry()
odom_msg.header = Header(stamp=rospy.Time.from_sec(time()), frame_id="odom")
odom_msg.pose.pose.position.x = robot_x
odom_msg.pose.pose.position.y = robot_y
odom_msg.pose.pose.position.z = 0.0
odom_msg.pose.pose.orientation.w = 1.0

# Lidar adatok gyűjtése
lidar_data = map_environment(grid, robot_x, robot_y)

# Rosbag fájlba mentés
bag = rosbag.Bag('robot_data.bag', 'w')
try:
    bag.write('robot_odom', odom_msg)
    bag.write('lidar_data', lidar_data)
finally:
    bag.close()

# Adatok kiírása
print(f"Robot pozíció: ({robot_x}, {robot_y})")
print(f"Odometry: {odom_msg}")
for angle, distance in enumerate(lidar_data.ranges):
    print(f"Szög: {angle}°, Távolság: {distance:.2f} méter")

    # Rajzolás
def plot_grid(grid, robot_x, robot_y):
    plt.imshow(grid, cmap='gray_r')
    plt.scatter(robot_y, robot_x, color='green', s=100)  # Robot jelölése
    plt.show()

# Eredeti térkép megjelenítése
plot_grid(grid, robot_x, robot_y)