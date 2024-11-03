#!/usr/bin/env python3

import rospy
import rosbag
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class TurtleBot3DataProcessor:
    def __init__(self, input_bag_file, output_bag_file):
        rospy.init_node('turtlebot3_data_processor')

        # Rosbag fájlok megnyitása
        self.input_bag = rosbag.Bag(input_bag_file, 'r')
        self.output_bag = rosbag.Bag(output_bag_file, 'w')

    def process_data(self):
        for topic, msg in self.input_bag.read_messages(topics=['/odom', '/scan']):
            if topic == '/odom':
                self.process_odom(msg)
            elif topic == '/scan':
                self.process_scan(msg)

        # Rosbag fájlok bezárása
        self.input_bag.close()
        self.output_bag.close()

    def process_odom(self, odom_msg):
        # Odometry adat mentése az új rosbag fájlba
        self.output_bag.write('robot_odom', odom_msg)

    def process_scan(self, scan_msg):
        # LaserScan adat mentése az új rosbag fájlba
        self.output_bag.write('lidar_data', scan_msg)

if __name__ == '__main__':
    processor = TurtleBot3DataProcessor('turtlebot3_data.bag', 'transformed_data.bag')
    processor.process_data()
