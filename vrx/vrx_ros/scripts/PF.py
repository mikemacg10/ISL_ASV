#!/usr/bin/env python

#PF Main Node. 
# code refactored from PF: Hands on Tutorial. 
# This node will subscribe to the target's estimated location and the target's ground truth location.
# The node will then use the particle filter to estimate the target's location.
# The node will then publish the estimated location.

#Michael MacGillivray

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from vrx_ros.core.particle_filters import ParticleFilterRangeOnly
from vrx_ros.core.resampling import ResamplingAlgorithms
from my_robot_interfaces.msg import RangeBearing
import csv
import numpy as np
import matplotlib.pyplot as plt

class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter_node')

        self.initialized = False
        self.target_positions = []
        self.tracker_positions = []
        self.time_prev = 0.0
        self.first_run = True
        self.count = 0
        
        # # Set up robot and particle filter
        # self.particle_filter = self.setup_particle_filter()

        # Subscribe to pose estimate topic
        self.subscription_1 = self.create_subscription(
            PoseStamped,
            '/target/estimated_location_tdoa',
            self.pose_estimate_callback,
            1)
                
        # Create subscribers for Target
        self.target_subscriber = self.create_subscription(
            Odometry,
            '/wamv_m_5/sensors/position/ground_truth_odometry',
            self.target_callback_ground_truth,
            1
        )

        # Create subscribers for Target
        self.taracker_subscriber = self.create_subscription(
            Odometry,
            '/wamv/sensors/position/ground_truth_odometry',
            self.tracker_callback_ground_truth,
            1
        )

        # Create subscribers for Target
        self.target_subscriber = self.create_subscription(
            RangeBearing,
            '/wamv/target/estimated_location_BF_DS_MVDR',
            self.pose_estimate_callback_beamformer,
            1
        )

        # Create a publisher for PF estimated location
        self.particle_filter_estimated_location_publisher = self.create_publisher(
            PoseStamped,
            '/target/particle_filter_estimated_location',
            1
        )

    def tracker_callback_ground_truth(self, msg):        
            # Tracker for BF trials
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            current_z = msg.pose.pose.position.z

            self.tracker_positions = np.array([current_x, current_y, current_z])

    def target_callback_ground_truth(self, msg):        
            # Process the received message
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            current_z = msg.pose.pose.position.z

            self.target_positions = np.array([current_x, current_y, current_z])          
    
    def publish_estimated_location(self, x, y, yaw):
        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0

        # Publish the estimated location
        self.particle_filter_estimated_location_publisher.publish(pose_msg)

    def setup_particle_filter(self, measurements, STATE):
        # Particle filter settings
        number_of_particles = 300
        pf_state_limits = [-1000, 1000, -1000, 1000]
        motion_model_forward_std = 15
        motion_model_turn_std = 3.14/4
        process_noise = [motion_model_forward_std, motion_model_turn_std]
        meas_model_distance_std = 75
        measurement_noise = meas_model_distance_std
        algorithm = ResamplingAlgorithms.SYSTEMATIC
        measurements_PF = measurements
        state = STATE

        # Initialize particle filter
        particle_filter = ParticleFilterRangeOnly(
            number_of_particles=number_of_particles,
            limits=pf_state_limits,
            process_noise=process_noise,
            measurement_noise=measurement_noise,
            measurement=measurements_PF,
            state = state,
            resampling_threshold = number_of_particles/10,
            resampling_algorithm=algorithm)
        particle_filter.initialize_particles_uniform()

        return particle_filter
    
    def pose_estimate_callback(self, msg):
        measurements = np.array([[msg.pose.position.x], [msg.pose.position.y]])
        t_new = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        STATE = 1

        if self.initialized == False:
            print('Initializing Particle Filter')
            self.particle_filter = self.setup_particle_filter(measurements, STATE)
            self.time_prev = t_new
            self.initialized = True
        
        self.ParticleFilter(measurements, t_new, STATE)
    
    def pose_estimate_callback_beamformer(self, msg):
        angle = msg.bearing 
        tracker_heading = msg.heading
        print('Tracker Heading:', tracker_heading)

        self.tracker_positions = np.array([msg.x, msg.y, msg.z])
        print('Tracker Positions:', self.tracker_positions)

        x = msg.range * np.cos(angle)
        y_1 = msg.range * np.sin(angle)
        y_2 = -y_1

        measurements = np.array([self.transformation_matrix(self.tracker_positions[0],
                                                             self.tracker_positions[1],
                                                               tracker_heading, x, y_1), 
                                self.transformation_matrix(self.tracker_positions[0],
                                                            self.tracker_positions[1],
                                                              tracker_heading, x, y_2)])
                
        STATE = 2

        t_new_msg = self.get_clock().now().to_msg()

        # Calculate the total time in seconds
        t_new = t_new_msg.sec + t_new_msg.nanosec * 1e-9

        if self.initialized == False:
            self.particle_filter = self.setup_particle_filter(measurements, STATE)
            self.time_prev = t_new
            self.initialized = True
            return
        
        self.ParticleFilter(measurements, t_new, STATE)

    def transformation_matrix(self, x, y, theta_rad, x_prime, y_prime):
         # Create the homogeneous transformation matrix
        homogenous_matrix = np.array([[np.cos(theta_rad), -np.sin(theta_rad), x],
                                    [np.sin(theta_rad), np.cos(theta_rad), y],
                                    [0, 0, 1]])

        # Create the vector of coordinates
        vector = np.array([[x_prime], [y_prime], [1]])

        # Multiply the homogeneous transformation matrix by the vector of coordinates
        transformed_vector = homogenous_matrix @ vector

        return transformed_vector[:2]

    def ParticleFilter(self, measurements, t_new, STATE):
        #Motion Model Inputs. 
        velocity = 3
        angular_velo = 0

        self.count += 1   

        #Obtain time step informaiton. 
        d_t = t_new - self.time_prev
        print('Time Step:', d_t)

        self.time_prev = t_new

        # Extract pose information from message
        estimated_distance_travelled = velocity * d_t
        estimated_rotation = angular_velo * d_t

        # Simulate measurement
        # measurements = np.array([[msg.pose.position.x], [msg.pose.position.y]])

        # Update particle filter
        self.particle_filter.update(
        robot_forward_motion=estimated_distance_travelled,
        robot_angular_motion=estimated_rotation,
        measurements=measurements,
        STATE=STATE)        

        # Get current robot state
        current = [self.target_positions[0], self.target_positions[1]]

        print('Current Position', current)
        print('Measurements', measurements)

        particles = self.particle_filter.get_particles()

        print('Particles:', np.shape(particles))
        # Get particle filter's guess of the robot state
        if STATE == 1:
            guess_1 = self.particle_filter.get_average_state_TDOA()
            with open('/home/michael-asv/vrx_ws/src/vrx/vrx_ros/scripts/poses.csv', 'a') as file:
                writer = csv.writer(file)
                if file.tell() == 0:
                    writer.writerow(['Guess X', 'Guess Y', 'Current X', 'Current Y', 'Tracker X', 'Tracker Y', 'Measured X', 'Measured Y'])
                    writer.writerow([float(guess_1[0]), float(guess_1[1]), float(current[0]), float(current[1]), float(self.tracker_positions[0]), float(self.tracker_positions[1]), float(measurements[0][0]), float(measurements[1][0])])
                writer.writerow([float(guess_1[0]), float(guess_1[1]), float(current[0]), float(current[1]), float(self.tracker_positions[0]), float(self.tracker_positions[1]), float(measurements[0][0]), float(measurements[1][0])])
        else:
            guess_1, guess_2 = self.particle_filter.get_average_state_BF()
            with open('/home/michael-asv/vrx_ws/src/vrx/vrx_ros/scripts/poses.csv', 'a') as file:
                writer = csv.writer(file)
                if file.tell() == 0:
                    writer.writerow(['Guess X', 'Guess Y', 'Guess X', 'Guess Y', 'Current X', 'Current Y', 'Tracker X', 'Tracker Y', 'Measured X', 'Measured Y', 'Measured X', 'Measured Y'])
                    writer.writerow([float(guess_1[0]), float(guess_1[1]), float(guess_2[0]), float(guess_2[1]), float(current[0]), float(current[1]), float(self.tracker_positions[0]), float(self.tracker_positions[1]), float(measurements[0][0]), float(measurements[0][1]), float(measurements[1][0]), float(measurements[1][1])])
                writer.writerow([float(guess_1[0]), float(guess_1[1]), float(guess_2[0]), float(guess_2[1]), float(current[0]), float(current[1]), float(self.tracker_positions[0]), float(self.tracker_positions[1]), float(measurements[0, 0]), float(measurements[0, 1]), float(measurements[1, 0]), float(measurements[1, 1])])

        self.publish_estimated_location(guess_1[0], guess_1[1], guess_1[2])

        # # plot current guess, current position, and the particles as a heatmap
        # plt.scatter([guess_1[0]], [guess_1[1]], color='red', label='Guess')
        # plt.scatter([guess_2[0]], [guess_2[1]], color='red', label='Guess')
        # plt.scatter([current[0]], [current[1]], color='blue', label='Current')
        # plt.scatter(particles[:, 0], particles[:, 1], color='green', alpha=0.1, label='Particles')
        # plt.legend()
        # plt.show()

        #close plot
        # plt.close()

        # if self.count % 10 == 0:
        #     self.particle_filter.save_particles()
        # error = ((guess[0] - current[0]) ** 2 + (guess[1] - current[1]) ** 2) ** 0.5
        # print(f'Error: {error}')
    
    
def main(args=None):    
    rclpy.init(args=args)
    particle_filter = ParticleFilterNode()
    rclpy.spin(particle_filter)
    particle_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
