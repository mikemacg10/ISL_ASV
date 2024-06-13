#!/usr/bin/env python

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from matplotlib.animation import FuncAnimation

import matplotlib.pyplot as plt

# Lists for 2D position data
real_positions = []
estimated_positions = []
particle_filter_positions = []

# Lists for error analysis
real_positions_error = []
estimated_positions_error = []
particle_filter_positions_error = []

# Initialize the figure for 2D position plots
fig_2d = plt.figure(figsize=(10, 5))
ax_2d = fig_2d.add_subplot(111)
fig_2d.suptitle('Real vs Estimated vs Particle Filter Position')

# Initialize the figure for error plots (now 4 subplots)
fig_error, axs = plt.subplots(4, 1, figsize=(10, 10))  # Adjusted for additional subplots
fig_error.suptitle('Localization Error in X, Y (%)')

def real_position_callback(msg):
    real_positions.append((msg.point.x, msg.point.y))
    print(real_positions)
    real_positions_error.append((msg.header.stamp.sec, msg.point.x, msg.point.y))

def estimated_position_callback(msg):
    estimated_positions.append((msg.point.x, msg.point.y))
    print(estimated_positions)
    estimated_positions_error.append((msg.header.stamp.sec, msg.point.x, msg.point.y))

def particle_filter_position_callback(msg):
    particle_filter_positions.append((msg.point.x, msg.point.y))
    print(particle_filter_positions)
    particle_filter_positions_error.append((msg.header.stamp.sec, msg.point.x, msg.point.y))

def animate_2d(i):
    ax_2d.clear()
    ax_2d.set_xlabel('X')
    ax_2d.set_ylabel('Y')
    ax_2d.set_title('2D Positions')

    if real_positions:
        real_x, real_y = zip(*real_positions)
        ax_2d.scatter(real_x, real_y, color='r', marker='o', label='Real Position')

    if estimated_positions:
        est_x, est_y = zip(*estimated_positions)
        ax_2d.scatter(est_x, est_y, color='g', marker='^', label='Estimated Position')

    if particle_filter_positions:
        kf_x, kf_y = zip(*particle_filter_positions)
        ax_2d.scatter(kf_x, kf_y, color='b', marker='s', label='Particle Filter Estimation')

    ax_2d.legend()

def calculate_error(real, other):
    errors = []
    min_len = min(len(real), len(other))
    for i in range(min_len):
        real_pos = real[i]
        other_pos = other[i]
        error_x = ((other_pos[1] - real_pos[1]) / real_pos[1]) * 100 if real_pos[1] else 0
        error_y = ((other_pos[2] - real_pos[2]) / real_pos[2]) * 100 if real_pos[2] else 0
        errors.append((real_pos[0], error_x, error_y))
    return zip(*errors)

def animate_error(i):
    if real_positions_error and estimated_positions_error:
        times, error_xs, error_ys = calculate_error(real_positions_error, estimated_positions_error)

        axs[0].clear()
        axs[0].plot(times, error_xs, label='Error X (Est)', color='r')
        axs[0].set_ylabel('% Error in X')
        axs[0].legend(loc='upper left')

        axs[1].clear()
        axs[1].plot(times, error_ys, label='Error Y (Est)', color='g')
        axs[1].set_ylabel('% Error in Y')
        axs[1].legend(loc='upper left')

        axs[2].clear()
        axs[2].set_visible(False)

        axs[3].clear()
        axs[3].set_visible(False)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('position_plotter')

    node.create_subscription(PoseStamped,
                             '/target/estimated_location', 
                             estimated_position_callback, 1)
    
    node.create_subscription(PoseStamped, 
                             '/target/particle_filter_estimated_location', 
                             particle_filter_position_callback, 1)
    
    node.create_subscription(Odometry, 
                             '/wamv/sensors/position/ground_truth_odometry', 
                             real_position_callback, 1)

    ani_2d = FuncAnimation(fig_2d, animate_2d, interval=1000)
    ani_error = FuncAnimation(fig_error, animate_error, interval=1000)

    plt.show()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
