#!/usr/bin/env python3

import rospy
import rosbag
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from math import cos, sin, sqrt

class TurtleBot3DataProcessor:
    def __init__(self, input_bag_file, output_bag_file):
        rospy.init_node('turtlebot3_data_processor')

        # Rosbag fájlok megnyitása
        self.input_bag = rosbag.Bag(input_bag_file, 'r')
        self.output_bag = rosbag.Bag(output_bag_file, 'w')

        # TF buffer és listener inicializálása
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def process_data(self):
        for topic, msg_odom, msg in self.input_bag.read_messages(topics=['/odom', '/scan']):
            if topic == '/odom':
                self.process_odom(msg_odom)
            elif topic == '/scan':
                self.process_scan(msg)

        # Rosbag fájlok bezárása
        self.input_bag.close()
        self.output_bag.close()

    def process_odom(self, msg):
        # Odometry adat mentése az új rosbag fájlba
        self.output_bag.write('robot_odom', msg_odom)

    def process_scan(self, msg):
        # LaserScan adat transzformálása globális koordináta rendszerbe
        try:
            # Transformáció lekérése a "base_link" és a "map" keretek között
            transform = self.tfBuffer.lookup_transform('map', msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            # Új LaserScan üzenet létrehozása a transzformált adatokkal
            transformed_scan = LaserScan()
            transformed_scan.header = Header(stamp=msg.header.stamp, frame_id='map')
            transformed_scan.angle_min = msg.angle_min
            transformed_scan.angle_max = msg.angle_max
            transformed_scan.angle_increment = msg.angle_increment
            transformed_scan.time_increment = msg.time_increment
            transformed_scan.scan_time = msg.scan_time
            transformed_scan.range_min = msg.range_min
            transformed_scan.range_max = msg.range_max
            transformed_scan.ranges = []
            transformed_scan.intensities = msg.intensities

            # Minden egyes pont transzformálása
            for i, range in enumerate(msg.ranges):
                if range < msg.range_min or range > msg.range_max:
                    transformed_scan.ranges.append(float('inf'))
                    continue

                angle = msg.angle_min + i * msg.angle_increment
                x = range * cos(angle)
                y = range * sin(angle)

                point = PoseStamped()
                point.header = msg.header
                point.pose.position.x = x
                point.pose.position.y = y
                point.pose.position.z = 0.0
                point.pose.orientation.w = 1.0

                transformed_point = tf2_geometry_msgs.do_transform_pose(point, transform)
                transformed_range = sqrt(transformed_point.pose.position.x**2 + transformed_point.pose.position.y**2)
                transformed_scan.ranges.append(transformed_range)

            # Transzformált LaserScan adat mentése az új rosbag fájlba
            self.output_bag.write('lidar_data', transformed_scan)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform error: {e}")

if __name__ == '__main__':
    processor = TurtleBot3DataProcessor('turtlebot3_data.bag', 'transformed_data.bag')
    processor.process_data()
