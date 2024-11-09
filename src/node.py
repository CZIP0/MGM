#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from math import cos, sin
import message_filters
import tf2_ros
import tf2_geometry_msgs

# Grid map inicializálása
GRID_SIZE = 1000  # Példa méret (szabadon változtatható)
GRID_RESOLUTION = 0.02
grid = np.full((GRID_SIZE, GRID_SIZE), 0.5)

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

class LidarGridMapper:
    def __init__(self):
        rospy.init_node('lidar_grid_mapper', anonymous=True)

        # TF2 buffer és listener inicializálása
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Feliratkozás az AMCL pozícióra és LiDAR topikokra a message_filters segítségével
        lidar_sub = message_filters.Subscriber("/scan", LaserScan)
        amcl_sub = message_filters.Subscriber("/amcl_pose", PoseWithCovarianceStamped)
        ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, amcl_sub], queue_size=100, slop=0.1)
        ts.registerCallback(self.callback)

        # Grid map publikáló RVIZ számára
        self.grid_publisher = rospy.Publisher("/grid_map", OccupancyGrid, queue_size=1)
        self.position_publisher = rospy.Publisher("/robot_position", PoseStamped, queue_size=1)

        rospy.spin()

    def callback(self, lidar_data, amcl_data):
        global grid

        # Transformáció lekérése a TF2 bufferből
        try:
            transform = self.tfBuffer.lookup_transform("map", lidar_data.header.frame_id, lidar_data.header.stamp, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform error: {e}")
            return

        # A robot pozíciójának transzformációja a globális koordinátarendszerbe
        robot_position = self.transform_to_global(amcl_data.pose.pose.position, transform)
        robot_x = int(robot_position[0] / GRID_RESOLUTION) + GRID_SIZE // 2
        robot_y = int(robot_position[1] / GRID_RESOLUTION) + GRID_SIZE // 2

        # LiDAR adatainak feldolgozása
        angle_min = lidar_data.angle_min
        angle_increment = lidar_data.angle_increment

        for i, distance in enumerate(lidar_data.ranges):
            if distance < lidar_data.range_min or distance > lidar_data.range_max:
                continue  # Skip invalid measurements

            angle = angle_min + i * angle_increment
            x = distance * cos(angle)
            y = distance * sin(angle)

            # Lokális pont transzformálása globális koordinátarendszerbe
            local_point = PoseStamped()
            local_point.header = lidar_data.header
            local_point.pose.position.x = x
            local_point.pose.position.y = y
            local_point.pose.orientation.w = 1.0

            transformed_point = tf2_geometry_msgs.do_transform_pose(local_point, transform)
            x_end = int(transformed_point.pose.position.x / GRID_RESOLUTION) + GRID_SIZE // 2
            y_end = int(transformed_point.pose.position.y / GRID_RESOLUTION) + GRID_SIZE // 2

            # Bresenham algoritmus használata az akadályig vezető vonal megrajzolásához
            points = bresenham(robot_x, robot_y, x_end, y_end)
            for j, (x, y) in enumerate(points):
                if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                    if j < len(points) - 1:
                        if grid[x][y] == 1:
                            continue
                        else:
                            grid[x][y] = 0  # Szabad terület
                    else:
                        grid[x][y] = 1  # Akadály

        # OccupancyGrid üzenet létrehozása és küldése RVIZ-be
        self.publish_occupancy_grid()

        # Robot pozíciójának küldése
        current_state = PoseStamped()
        current_state.header.frame_id = "map"
        current_state.header.stamp = rospy.Time.now()
        current_state.pose.position.x = amcl_data.pose.pose.position.x
        current_state.pose.position.y = amcl_data.pose.pose.position.y
        current_state.pose.orientation.w = 1.0

        self.position_publisher.publish(current_state)

    def transform_to_global(self, position, transform):
        """Pozíció transzformálása a globális koordinátarendszerbe."""
        point = PoseStamped()
        point.header.frame_id = "map"
        point.pose.position.x = position.x
        point.pose.position.y = position.y
        point.pose.orientation.w = 1.0

        global_point = tf2_geometry_msgs.do_transform_pose(point, transform)
        return (global_point.pose.position.x, global_point.pose.position.y)

    def publish_occupancy_grid(self):
        global grid
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = Header()
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.header.frame_id = "map"

        # Map metaadatok beállítása
        occupancy_grid.info = MapMetaData()
        occupancy_grid.info.resolution = GRID_RESOLUTION
        occupancy_grid.info.width = GRID_SIZE
        occupancy_grid.info.height = GRID_SIZE
        occupancy_grid.info.origin.position.x = -GRID_SIZE // 2 * GRID_RESOLUTION
        occupancy_grid.info.origin.position.y = -GRID_SIZE // 2 * GRID_RESOLUTION
        occupancy_grid.info.origin.position.z = 0

        # Adatok beállítása (0-100 közötti értékek)
        occupancy_grid.data = (grid.flatten() * 100).astype(int).tolist()
        self.grid_publisher.publish(occupancy_grid)

if __name__ == "__main__":
    LidarGridMapper()
