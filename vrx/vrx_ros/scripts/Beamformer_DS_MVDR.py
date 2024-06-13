#!/usr/bin/env python3

# Code by Michael MacGillivray.
# Major citation to PySDR: A Guide to SDR and DSP using Python
# Direction of Arrival (DOA) Estimation using Beamforming Techniques

# This code defines a class DOA_Estimation for estimating the direction of arrival of a signal using two beamforming techniques:
# 1. Minimum Variance Distortionless Response (MVDR) beamforming.
# 2. Delay-and-Sum beamforming.

# The DOA estimation is performed based on the received signal at an array of sensors.

# The code also includes a Plotter class for visualizing the results of the DOA estimation.


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import random
import tf2_ros
from my_robot_interfaces.msg import RangeBearing


class DOA_Estimation(Node):
    def __init__(self, boat_name):

        super().__init__('beamformer_ds_mvdr' + boat_name)

        self.boat_name = boat_name

        self.sample_rate = 16e3
        self.N = 4000
        self.d = 0.5
        self.Nr = 10
        self.frequency = 50
        self.target_positon_ground_truth = np.array([0, 0, 0])
        self.self_pose = np.array([0, 0, 0])
        self.theta_scan = np.linspace(-1 * np.pi, np.pi, 180)
        self.results = None
        self.range = 0
        self.theta = 0
        self.yaw = 0

        # Create subscribers for all boats
        self.target_positon_ground_truth_subcriber = self.create_subscription(
            Odometry,
            '/wamv_m_5/sensors/position/ground_truth_odometry',
            self.target_pose_callback,
            1
        )

        current_position_topic = '/{}/sensors/position/ground_truth_odometry'.format(boat_name)

        self.current_positon_ground_truth_subcriber = self.create_subscription(
            Odometry,
            current_position_topic,
            self.self_pose_callback,
            1
        )

        current_position_topic = '/{}/target/estimated_location_BF_DS_MVDR'.format(boat_name)

        # Create a publisher for estimated location
        self.estimated_location_publisher = self.create_publisher(
            RangeBearing,
            current_position_topic,
            1
        )


        self.signal_send = self.create_timer(10.0, self.simulate_received_signal)

    def target_pose_callback(self, msg):        
        # Process the received message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z

        self.target_positon_ground_truth = np.array([current_x, current_y, current_z])

    def self_pose_callback(self, msg):        
        # Process the received message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z

        quaternion = msg.pose.pose.orientation

         # Convert quaternion to euler angles (yaw, pitch, roll)
        self.yaw = self.euler_from_quaternion(quaternion)    

        self.self_pose = np.array([current_x, current_y, current_z])

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # sinr_cosp = 2 * (w * x + y * z)
        # cosr_cosp = 1 - 2 * (x * x + y * y)
        # roll = np.arctan2(sinr_cosp, cosr_cosp)

        # sinp = 2 * (w * y - z * x)
        # pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
 

        return yaw

    def publish_estimated_location(self):
        # Create a message
        pose_msg = RangeBearing()
        pose_msg.range = self.range
        pose_msg.bearing = self.theta
        pose_msg.heading = float(self.yaw)

        pose_msg.x = float(self.self_pose[0])
        pose_msg.y = float(self.self_pose[1])
        pose_msg.z = float(self.self_pose[2])

        # Publish the estimated location
        self.estimated_location_publisher.publish(pose_msg)      

    def calc_angle(self):
        # Calculate the angle between the boat and the target
        angle = np.arctan2((self.target_positon_ground_truth[1] - self.self_pose[1]), (self.target_positon_ground_truth[0] - self.self_pose[0])) - self.yaw
        return angle
    
    def calc_distance(self):
        # Calculate the distance between the boat and the target
        distance = np.linalg.norm((self.target_positon_ground_truth - self.self_pose))
        distance += np.random.normal(0, distance/5)
        self.range = distance
    
    def generate_tone(self):
        """
        Generate a tone signal.

        Returns:
        - Tone signal.
        """
        t = np.arange(self.N) / self.sample_rate
        s = np.exp(2j * np.pi * self.frequency * t)
        #plot fft of tone signal with dB on y-axis
        # plt.figure(1)
        # plt.plot(np.abs(np.fft.fft(s)))
        # plt.yscale('log')
        # plt.show()
        
        #plot spectrogram of tone signal
        # plt.figure(2)
        # plt.specgram(s, NFFT=256, Fs=self.sample_rate, noverlap=128)
        # plt.show()
        return s
    
    def calculate_array_factor(self):
        """
        Calculate the array factor.

        Returns:
        - Array factor.
        """
        theta = self.calc_angle()
        delay_error = np.random.normal(loc=0, scale=0.051, size = self.Nr)


        a = np.exp(-2j * np.pi * self.d * np.arange(self.Nr) * np.cos(theta))
        n = 0 + 1j * np.random.uniform(-0.01, 0.01, self.Nr)
        return a * np.exp(-2j * np.pi * delay_error * np.arange(self.Nr))
    
    def simulate_received_signal(self):
        """
        Simulate the received signal at the array elements.

        Returns:
        - Received signal.
        """

        self.calc_distance()

        tx = self.generate_tone().reshape(-1, 1)
        a = self.calculate_array_factor().reshape(-1, 1)
        r = a @ tx.T
        # n = np.random.randn(self.Nr, self.N) + 1j * np.random.randn(self.Nr, self.N)
        # # add noise
        # r += 5*n

        # # make one of the signals just noise, indicating a missed signal
        # prob = np.random.rand()
        # random_numbers = random.sample(range(0, 9), 3)

        # if prob < 0.3:
        #     print("Missed signal")
        #     for i in range(len(random_numbers)):
        #         r[random_numbers[i]] = n[random_numbers[i]]

        self.delay_and_sum(r, self.theta_scan)
        self.publish_estimated_location()
    

    def mvdr_beamforming(self, r, theta_scan):
        """
        Perform MVDR beamforming to estimate the DOA.

        Args:
        - r: Received signal.
        - theta_scan: Array of angles to scan.

        Returns:
        - Results of MVDR beamforming.
        """
        results = []
        for theta_i in theta_scan:
            w = self._w_mvdr(theta_i, r)
            r_weighted = w.conj().T @ r
            power_dB = 10 * np.log10(np.var(r_weighted))
            results.append(power_dB)
        results -= np.max(results)
        return results
    
    def _w_mvdr(self, theta, r):
        """
        Calculate the MVDR weight vector.

        Args:
        - theta: Angle.
        - r: Received signal.

        Returns:
        - MVDR weight vector.
        """
        a = np.exp(-2j * np.pi * self.d * np.arange(self.Nr) * np.cos(theta))
        a = a.reshape(-1, 1)
        R = r @ r.conj().T
        Rinv = np.linalg.pinv(R)
        w = (Rinv @ a) / (a.conj().T @ Rinv @ a)
        return w
    
    def delay_and_sum(self, r, theta_scan):
        """
        Perform Delay-and-Sum beamforming to estimate the DOA.

        Args:
        - r: Received signal.
        - theta_scan: Array of angles to scan.

        Returns:
        - Results of Delay-and-Sum beamforming.
        """
        results = []
        max_index = 0
        max_value = 0
        for i, theta_i in enumerate(theta_scan):
            w = np.exp(-2j * np.pi * self.d * np.arange(self.Nr) * np.cos(theta_i))
            r_weighted = w.conj().T @ r
            power_dB = 10 * np.log10(np.var(r_weighted))
            results.append(power_dB)
            if power_dB > max_value:
                max_value = power_dB
                max_index = i
        results -= np.max(results)
        self.theta = np.abs(theta_scan[max_index])
