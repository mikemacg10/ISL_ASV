#!/usr/bin/env python3

#Author: Michael MacGillivray
#This code will use multilateration to solve to the pose of a target ASV. 
#ROS2 Humble and Gazebo Garden are used. 
#The original Ros Packaget is the VRX_ws from git. 


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Quaternion, Point
from tf2_ros import TransformListener
import tf2_py
import matplotlib.pyplot as plt

import numpy as np
import scipy.optimize as opt
from scipy.optimize import least_squares
import csv

class TrilaterationSolver(Node):
    def __init__(self):
        super().__init__('trilateration_solver')

        # Initialize variables to store the latest positions for boats and the target
        self.boat_1_positions = 0  # List to store x, y, z values for each boat
        self.boat_2_positions = 0  # List to store x, y, z values for each boat
        self.boat_3_positions = 0  # List to store x, y, z values for each boat
        self.boat_4_positions = 0  # List to store x, y, z values for each boat
        self.target_PF_position = None


        self.target_position = 0  
        self.actual_x = []
        self.actual_y = []
        self.estimated_x = []
        self.estimated_y = []

        self.x_last = 0
        self.y_last = 0
        self.z_last = -0.10

        # Create subscribers for all boats
        self.boat_1_subscriber = self.create_subscription(
            Odometry,
            '/wamv/sensors/position/ground_truth_odometry',
            self.pose_boat_1_callback,
            1
        )
        
        # Create subscribers for all boats
        self.boat_2_subscriber = self.create_subscription(
            Odometry,
            '/wamv_m_2/sensors/position/ground_truth_odometry',
            self.pose_boat_2_callback,
            1
        )
        
        # Create subscribers for all boats
        self.boat_3_subscriber = self.create_subscription(
            Odometry,
            '/wamv_m_3/sensors/position/ground_truth_odometry',
            self.pose_boat_3_callback,
            1
        )
        
        # Create subscribers for all boats
        self.boat_4_subscriber = self.create_subscription(
            Odometry,
            '/wamv_m_4/sensors/position/ground_truth_odometry',
            self.pose_boat_4_callback,
            1
        )


        # Subscribe to target position
        self.pose_target_subscriber = self.create_subscription(
            Odometry,
            '/wamv_m_5/sensors/position/ground_truth_odometry',
            self.pose_target_callback,
            1
        )

        # Subscribe to target position
        self.PF_estimate = self.create_subscription(
            PoseStamped,
            '/target/particle_filter_estimated_location',
            self.PF_target_callback,
            1
        )

        # Create a publisher for estimated location
        self.estimated_location_publisher = self.create_publisher(
            PoseStamped,
            '/target/estimated_location_tdoa',
            1
        )


        self.signal_send = self.create_timer(10, self.solve)

    def pose_boat_1_callback(self, msg):        
        # Process the received message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z

        self.boat_1_positions = np.array([current_x, current_y, current_z])
    
    def pose_boat_2_callback(self, msg):        
        # Process the received message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z

        self.boat_2_positions = np.array([current_x, current_y, current_z])
    
    def pose_boat_3_callback(self, msg):        
        # Process the received message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z

        self.boat_3_positions = np.array([current_x, current_y, current_z])
    
    def pose_boat_4_callback(self, msg):        
        # Process the received message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z

        self.boat_4_positions = np.array([current_x, current_y, current_z])

    def pose_target_callback(self, msg):
        # Process the received message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = 0
        # Update the latest x, y, z values for the target
        self.target_position = np.array([current_x, current_y, current_z])

    def PF_target_callback(self, msg):
        # Process the received message
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        current_z = -0.01
        # Update the latest x, y, z values for the target
        self.target_PF_position = np.array([current_x, current_y, current_z])

    def publish_estimated_location(self, x, y, z):
        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Publish the estimated location
        self.estimated_location_publisher.publish(pose_msg)
       
    def functions(self, x0, y0, z0, x1, y1, z1, d01, STATE, x2=0, y2=0, z2=0, d02=0, d12=0, x3=0, y3=0, z3=0,  d03=0,  d13=0, d23=0):
        if STATE == 4:
            def fn(args):
                x, y, z = args
                a = np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2) - np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2) - d01
                b = np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2) - d02
                c = np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2) - d03
                d = np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2) - d12
                e = np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2) - d13
                f = np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - d23
                return [a, b, c, d, e, f]
        

        if STATE == 3:
            def fn(args):
                x, y, z = args
                a = np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2) - np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2) - d01
                b = np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2) - d02
                c = np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2) - d12
                return [a, b, c]
            
            
        if STATE == 2:
            def fn(args):
                x, y, z = args
                a = np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2) - np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2) - d01
                return [a]
        return fn

    def jacobian(self, x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3):
        def fn(args):
            x, y, z = args
            adx = (x - x1) / np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2) - (x - x0) / np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)
            bdx = (x - x2) / np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - (x - x0) / np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)
            cdx = (x - x3) / np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - (x - x0) / np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2)
            ady = (y - y1) / np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2) - (y - y0) / np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)
            bdy = (y - y2) / np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - (y - y0) / np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)
            cdy = (y - y3) / np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - (y - y0) / np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2)
            adz = (z - z1) / np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2) - (z - z0) / np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)
            bdz = (z - z2) / np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - (z - z0) / np.sqrt((x - x1) ** 2 + (y - y1) ** 2 + (z - z1) ** 2)
            cdz = (z - z3) / np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - (z - z0) / np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2)

            ddx = (x - x2) / np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - (x - x0) / np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
            edx = (x - x3) / np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - (x - x0) / np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
            fdx = (x - x3) / np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - (x - x0) / np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
            ddy = (y - y2) / np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - (y - y0) / np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
            edy = (y - y3) / np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - (y - y0) / np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
            fdy = (y - y3) / np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - (y - y0) / np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
            ddz = (z - z2) / np.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) - (z - z0) / np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
            edz = (z - z3) / np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - (z - z0) / np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)
            fdz = (z - z3) / np.sqrt((x - x3) ** 2 + (y - y3) ** 2 + (z - z3) ** 2) - (z - z0) / np.sqrt((x - x0) ** 2 + (y - y0) ** 2 + (z - z0) ** 2)

            return np.array([
                [adx, ady, adz],
                [bdx, bdy, bdz],
                [cdx, cdy, cdz],
                [ddx, ddy, ddz],
                [edx, edy, edz],
                [fdx, fdy, fdz]])
        return fn


    def solve(self):
        # Extract x and y coordinates for each boat

        ERROR = 150 # Error in meters (equivlent to 0.1s std error for hydrophone)

        positions = {
            'boat_0': {
            'x': self.boat_1_positions[0],
            'y': self.boat_1_positions[1],
            'z': self.boat_1_positions[2]
            },
            'boat_1': {
            'x': self.boat_2_positions[0],
            'y': self.boat_2_positions[1],
            'z': self.boat_2_positions[2]
            },
            'boat_2': {
            'x': self.boat_3_positions[0],
            'y': self.boat_3_positions[1],
            'z': self.boat_3_positions[2]
            },
            'boat_3': {
            'x': self.boat_4_positions[0],
            'y': self.boat_4_positions[1],
            'z': self.boat_4_positions[2]
            }
        }

        x0, y0, z0 = positions['boat_0']['x'], positions['boat_0']['y'], positions['boat_0']['z']
        x1, y1, z1 = positions['boat_1']['x'], positions['boat_1']['y'], positions['boat_1']['z']
        x2, y2, z2 = positions['boat_2']['x'], positions['boat_2']['y'], positions['boat_2']['z']
        x3, y3, z3 = positions['boat_3']['x'], positions['boat_3']['y'], positions['boat_3']['z']

        x_target, y_target, z_target = self.target_position[0], self.target_position[1], self.target_position[2]
  
        ranges = {
            'range_0': np.sqrt((x0 - x_target)**2 + (y0 - y_target)**2 + (z0 - z_target)**2) + np.random.normal(0, ERROR),
            'range_1': np.sqrt((x1 - x_target)**2 + (y1 - y_target)**2 + (z1 - z_target)**2) + np.random.normal(0, ERROR),
            'range_2': np.sqrt((x2 - x_target)**2 + (y2 - y_target)**2 + (z2 - z_target)**2) + np.random.normal(0, ERROR),
            'range_3': np.sqrt((x3 - x_target)**2 + (y3 - y_target)**2 + (z3 - z_target)**2) + np.random.normal(0, ERROR)
        }

        print(ranges)

        if sum([ranges['range_0'] < 10000, ranges['range_1'] < 10000, ranges['range_2'] < 10000, ranges['range_3'] < 10000]) >= 4:
            d01 = ranges['range_1'] - ranges['range_0'] 
            d02 = ranges['range_2'] - ranges['range_0'] 
            d03 = ranges['range_3'] - ranges['range_0'] 
            d12 = ranges['range_2'] - ranges['range_1'] 
            d13 = ranges['range_3'] - ranges['range_1'] 
            d23 = ranges['range_3'] - ranges['range_2'] 
            
            STATE = 4

            print("four detections")
            
            F = self.functions(x0, y0, z0, x1, y1, z1, d01, STATE, x2, y2, z2, d02, d12, x3, y3, z3,  d03,  d13, d23)
            #J = self.jacobian(x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3)
        elif sum([ranges['range_0'] < 10000, ranges['range_1'] < 10000, ranges['range_2'] < 10000, ranges['range_3'] < 10000]) >= 3:
            # Save the three boats that are less than 10000
            boats_range_greater_than_10000 = [i for i, r in enumerate([ranges['range_0'], ranges['range_1'], ranges['range_2'], ranges['range_3']]) if r < 10000]
            
            print(boats_range_greater_than_10000)

            range_0 = ranges["range_" + str(boats_range_greater_than_10000[0])]
            range_1 = ranges["range_" + str(boats_range_greater_than_10000[1])]
            range_2 = ranges["range_" + str(boats_range_greater_than_10000[2])]

            print("ranges", range_0, range_1, range_2)

            d01 = range_1 - range_0 
            d02 = range_2 - range_0 
            d12 = range_2 - range_1 

            print(d01, d02, d12)

            # What boats recieved signals
            boat_0 = 'boat_' + str(boats_range_greater_than_10000[0])
            boat_1 = 'boat_' + str(boats_range_greater_than_10000[1])
            boat_2 = 'boat_' + str(boats_range_greater_than_10000[2])

            print(boat_0, boat_1, boat_2)

            x0, y0, z0 = positions[boat_0]['x'], positions[boat_0]['y'], positions[boat_0]['z']
            x1, y1, z1 = positions[boat_1]['x'], positions[boat_1]['y'], positions[boat_1]['z']
            x2, y2, z2 = positions[boat_2]['x'], positions[boat_2]['y'], positions[boat_2]['z']

            STATE = 3
            print("three detections")

            
            F = self.functions(x0, y0, z0, x1, y1, z1, d01, STATE, x2, y2, z2, d02, d12)
            #J = self.jacobian(x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3)
        elif sum([ranges['range_0'] < 10000, ranges['range_1'] < 10000, ranges['range_2'] < 10000, ranges['range_3'] < 10000]) >= 2:
            # Save the three boats that are less than 10000
            boats_range_greater_than_10000 = [i for i, r in enumerate([ranges['range_0'], ranges['range_1'], ranges['range_2'], ranges['range_3']]) if r > 10000]
            range_0 = ranges["range_" + str(boats_range_greater_than_10000[0])]
            range_1 = ranges["range_" + str(boats_range_greater_than_10000[1])]

            d01 = range_1 - range_0

            # What boats recieved signals
            boat_0 = 'boat_' + str(boats_range_greater_than_10000[0])
            boat_1 = 'boat_' + str(boats_range_greater_than_10000[1])

            x0, y0, z0 = positions[boat_0]['x'], positions[boat_0]['y'], positions[boat_0]['z']
            x1, y1, z1 = positions[boat_1]['x'], positions[boat_1]['y'], positions[boat_1]['z']           
            
            STATE = 2
            print("two detections")

            
            F = self.functions(x0, y0, z0, x1, y1, z1, d01, STATE)
            #J = self.jacobian(x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3)
        else:
            print("Not enough detections")
            return None
        
        if self.target_PF_position is not None:
            xp, yp, zp = self.target_PF_position[0], self.target_PF_position[1], self.target_PF_position[2]
        else:
            xp = self.x_last
            yp = self.y_last
            zp = self.z_last
        

        #Soultion bounds for solver to adhere by (x_min, y_min, z_min). 
        bound = ([-np.inf, -np.inf, -1],[np.inf, np.inf, 0.01]) #([-1e5, -1e5, -1e5],[1e5, 1e5, 0.10])

        estimated_values = least_squares(F, x0=[xp, yp, zp], bounds=bound).x

        if estimated_values is not None:
            x, y, z = estimated_values
            print(f"x error: {x - x_target}")
            print(f"y error: {y - y_target}")
            print(f"z error: {z - z_target} \n")
            self.publish_estimated_location(x, y, z)
        else:
            return None
        
        if abs(x) > 10000 or abs(y) > 10000 or abs(z0) > 10000:
            self.x_last = 0
            self.y_last = 0
            self.z_last = 0.01
        else:       
            self.x_last = x
            self.y_last = y
            self.z_last = z


        # # Update the lists with the current x and y coordinates
        # self.actual_x.append(x_target)
        # self.actual_y.append(y_target)
        # self.estimated_x.append(x)
        # self.estimated_y.append(y)


def main(args=None):
    rclpy.init(args=args)
    trilateration_solver = TrilaterationSolver()
    try:
        rclpy.spin(trilateration_solver)
    except KeyboardInterrupt:
        pass
    trilateration_solver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
