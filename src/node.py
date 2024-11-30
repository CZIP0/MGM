#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from math import cos, sin
import message_filters
import tf2_ros
import tf2_geometry_msgs

# Grid map inicializálása
GRID_SIZE = 200  # Példa méret (szabadon változtatható)
GRID_RESOLUTION = 0.03
grid = np.zeros((GRID_SIZE, GRID_SIZE))  # Kezdeti log odds értékek (0), ami 50%-os esélyt jelent

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
        # self.tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(0.2))  # 200 ms cache idő

        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Feliratkozás az AMCL pozícióra és LiDAR topikokra a message_filters segítségével
        lidar_sub = message_filters.Subscriber("/scan", LaserScan)
        amcl_sub = message_filters.Subscriber("/amcl_pose", PoseWithCovarianceStamped)
        odom_sub = message_filters.Subscriber("/odom", Odometry)
        # ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, odom_sub], queue_size=100, slop=0.1)
        ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, odom_sub], queue_size=100, slop=2)

        ts.registerCallback(self.callback)

        # Grid map publikáló RVIZ számára
        self.grid_publisher = rospy.Publisher("/grid_map", OccupancyGrid, queue_size=1)
        self.position_publisher = rospy.Publisher("/robot_position", PoseStamped, queue_size=1)

        rospy.spin()

    def callback(self, lidar_data, odom_data):
        global grid

        # Transformáció lekérése
        try:
            transform_lidar = self.tfBuffer.lookup_transform("map", lidar_data.header.frame_id, lidar_data.header.stamp, rospy.Duration(0.02))
            # if transform_lidar is None:
            #     transform_lidar = prev_transform_lidar
            # else:
            #     prev_transform_lidar = transform_lidar
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform error: {e}")
            return
        
        try:
            transform_pos = self.tfBuffer.lookup_transform("map", odom_data.header.frame_id, odom_data.header.stamp, rospy.Duration(0.02))
            # if transform_pos is None:
            #     transform_pos = prev_transform_pos
            # else:
            #     prev_transform_pos = transform_pos
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform error: {e}")
            return

        # A robot pozíciójának transzformációja a globális koordinátarendszerbe
        odom_pose = PoseStamped()
        odom_pose.header = odom_data.header
        odom_pose.pose = odom_data.pose.pose
        robot_position = tf2_geometry_msgs.do_transform_pose(odom_pose, transform_pos)

        robot_x = int(robot_position.pose.position.x / GRID_RESOLUTION) + GRID_SIZE // 2
        robot_y = int(robot_position.pose.position.y / GRID_RESOLUTION) + GRID_SIZE // 2

        # LiDAR adatainak feldolgozása
        angle_min = lidar_data.angle_min
        angle_increment = lidar_data.angle_increment

        # Inverse Sensor Model paraméterei - Új részek kezdete
        log_odds_hit = 0.85  # Akadály frissítés (log odds)
        log_odds_miss = -0.7  # Szabad terület frissítés (log odds)
        max_range = lidar_data.range_max
        min_range = lidar_data.range_min
        # Inverse Sensor Model paraméterei - Új részek vége

        for i, distance in enumerate(lidar_data.ranges[::2]):
            if distance < min_range or distance > max_range:
                continue  # Skip invalid measurements

            angle = angle_min + i * angle_increment * 2
            x = distance * cos(angle)
            y = distance * sin(angle)

            # Lokális pont transzformálása globális koordinátarendszerbe
            local_point = PoseStamped()
            local_point.header = lidar_data.header
            local_point.pose.position.x = x
            local_point.pose.position.y = y
            local_point.pose.orientation.w = 1.0

            transformed_point = tf2_geometry_msgs.do_transform_pose(local_point, transform_lidar)
            x_end = int(transformed_point.pose.position.x / GRID_RESOLUTION) + GRID_SIZE // 2
            y_end = int(transformed_point.pose.position.y / GRID_RESOLUTION) + GRID_SIZE // 2

            # Bresenham algoritmus használata az akadályig vezető vonal megrajzolásához
            points = bresenham(robot_x, robot_y, x_end, y_end)

            # Inverse Sensor Model alkalmazása - Új részek kezdete
            for j, (y, x) in enumerate(points):  
                if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                    if j < len(points) - 1:  # Szabad terület
                        if grid[x][y] > 0:
                            # grid[x][y] = 10
                            continue
                        else:
                            grid[x][y] += log_odds_miss
                    else:  # Akadály
                        grid[x][y] += log_odds_hit
            # Inverse Sensor Model alkalmazása - Új részek vége

        # Publikálás és pozíciók küldése
        self.publish_occupancy_grid()
        self.publish_robot_position(robot_position)

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

        # Adatok beállítása (0-100 közötti értékek log odds-ból konvertálva)
        occupancy_grid.data = [int(self.log_odds_to_probability(grid[i, j]) * 100) for i in range(GRID_SIZE) for j in range(GRID_SIZE)]
        self.grid_publisher.publish(occupancy_grid)

    def log_odds_to_probability(self, log_odds):
        """Log odds átváltása valószínűségre."""
        return 1.0 / (1.0 + np.exp(-log_odds))

    def publish_robot_position(self, robot_position):
        current_state = PoseStamped()
        current_state.header.frame_id = "map"
        current_state.header.stamp = rospy.Time.now()
        current_state.pose.position.x = robot_position.pose.position.x
        current_state.pose.position.y = robot_position.pose.position.y
        current_state.pose.orientation = robot_position.pose.orientation

        self.position_publisher.publish(current_state)

if __name__ == "__main__":
    LidarGridMapper()
