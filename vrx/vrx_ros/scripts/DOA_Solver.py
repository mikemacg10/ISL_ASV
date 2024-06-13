#!/usr/bin/env python3

# Code by Michael MacGillivray.
# Future Work to implment a DOA solver for multiple BF DoA measurments. Implment so it can be sent to PF. 


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import random
import tf2_ros
from my_robot_interfaces.msg import RangeBearing

class DOA_intercection_solver(Node):
    def __init__(self):
        super().__init__('sovler_DOA')

        # Create subscribers for all boats
        self.target_positon_ground_truth_subcriber = self.create_subscription(
            Odometry,
            '/wamv/target/estimated_location_BF_DS_MVDR',
            self.target_pose_callback_1,
            1
        )

        # Create subscribers for all boats
        self.target_positon_ground_truth_subcriber = self.create_subscription(
            Odometry,
            '/wamv/target/estimated_location_BF_DS_MVDR',
            self.target_pose_callback_2,
            1
        )

    def target_pose_callback_2(self, msg):
        angle = msg.bearing 
        tracker_heading = msg.heading
        print('Tracker Heading:', tracker_heading)

        self.tracker_positions = np.array([msg.x, msg.y, msg.z])
        print('Tracker Positions:', self.tracker_positions)

        

        t_new_msg = self.get_clock().now().to_msg()

        # Calculate the total time in seconds
        t_new = t_new_msg.sec + t_new_msg.nanosec * 1e-9
