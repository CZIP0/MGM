#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import cos, sin

# Grid map inicializálása
GRID_SIZE = 20
GRID_RESOLUTION = 1  # 1 méter per cella
grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)

# Robot pozíció és orientáció inicializálása
robot_x, robot_y = GRID_SIZE // 2, GRID_SIZE // 2  # Kezdeti pozíció a rács közepén

def bresenham(x0, y0, x1, y1):
    """Bresenham vonal rajzoló algoritmus."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return points

def lidar_callback(scan_data):
    global grid, robot_x, robot_y
    angle_min = scan_data.angle_min
    angle_increment = scan_data.angle_increment

    for i, distance in enumerate(scan_data.ranges):
        if distance > 0:  # Csak érvényes mérések
            angle = angle_min + i * angle_increment
            
            # A LiDAR mérések alapján célpont koordináták meghatározása a robot pozíciójához képest
            x_end = robot_x + int((distance * cos(angle)) / GRID_RESOLUTION)
            y_end = robot_y + int((distance * sin(angle)) / GRID_RESOLUTION)

            # Bresenham algoritmus használata az aktuális pozíciótól a célpontig
            points = bresenham(robot_x, robot_y, x_end, y_end)
            for j, (x, y) in enumerate(points):
                x = int(x)
                y = int(y)
                if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                    if j < len(points) - 1:
                        grid[x][y] = 0   # 0 a vonalon, 1 a célpontban
                    else:
                        grid[x][y]= 1

    # Grid megjelenítése
    plt.imshow(grid, cmap='gray_r')
    plt.scatter(robot_y, robot_x, color='red', s=100, label="Robot Position")  # Robot pozíció piros ponttal
    plt.title("20x20 Grid Map")
    plt.pause(0.05)
    plt.clf()

def odometry_callback(odom_data):
    global robot_x, robot_y
    # Robot pozíciójának kiolvasása és rács pozícióra konvertálása
    position = odom_data.pose.pose.position
    # robot_x = int(position.x / GRID_RESOLUTION) + GRID_SIZE // 2
    robot_x = position.x
    # robot_y = int(position.y / GRID_RESOLUTION) + GRID_SIZE // 2
    robot_y = position.y

def main():
    rospy.init_node('lidar_grid_mapper', anonymous=True)

    # Feliratkozás az odometriai és LiDAR topikokra
    rospy.Subscriber("/robot_odom", Odometry, odometry_callback)
    rospy.Subscriber("/lidar_data", LaserScan, lidar_callback)

    # Megjelenítési beállítások
    plt.ion()
    plt.figure(figsize=(5, 5))

    rospy.spin()

if __name__ == "__main__":
    main()
