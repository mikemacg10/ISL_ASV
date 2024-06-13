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
from Beamformer_DS_MVDR import DOA_Estimation


def main(args=None):
    rclpy.init(args=args)

    # Create instances of DOA_Estimation for each boat
    boat1_beamformer = DOA_Estimation('wamv')
    
    try:
        # Run the ROS2 event loop for each instance
        rclpy.spin(boat1_beamformer)
    except KeyboardInterrupt:
        pass

    # Clean up resources
    boat1_beamformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()