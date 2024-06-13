from abc import abstractmethod
import copy
import numpy as np
import csv


class ParticleFilter:
    """
    Notes:
        * State is (x, y, heading), where x and y are in meters and heading in radians
        * State space assumed limited size in each dimension, world is cyclic (hence leaving at x_max means entering at
        x_min)
        * Abstract class
    """

    def __init__(self, number_of_particles, limits, process_noise, measurement_noise, measurements=None, state=None):
        """
        Initialize the abstract particle filter.

        :param number_of_particles: Number of particles
        :param limits: List with maximum and minimum values for x and y dimension: [xmin, xmax, ymin, ymax]
        :param process_noise: Process noise parameters (standard deviations): [std_forward, std_angular]
        :param measurement_noise: Measurement noise parameters (standard deviations): [std_range, std_angle]
        """

        if number_of_particles < 1:
            print("Warning: initializing particle filter with number of particles < 1: {}".format(number_of_particles))

        # Initialize filter settings
        self.n_particles = number_of_particles
        self.particles = []
        self.particles_2 = []
        self.inital_measurements = measurements
        self.state = state
        self.count_for_reinitialization = 0

        # State related settings
        self.state_dimension = 3  # x, y, theta

        # Set noise
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def initialize_particles_uniform_measured(self):
        """
        Initialize the particles uniformly over the world assuming a 3D state (x, y, heading).
        No arguments are required and function always succeeds hence no return value.
        """
        # Initialize particles with uniform weight distribution
        self.particles = []
        weight = 1.0 / self.n_particles
        
        # Create arrays of uniformly distributed random numbers for each dimension
        x_values = np.random.uniform(self.x_min, self.x_max, self.n_particles)
        y_values = np.random.uniform(self.y_min, self.y_max, self.n_particles)
        heading_values = np.random.uniform(0, 2 * np.pi, self.n_particles)
        
        # Combine the random values for each particle and append to particles list
        for i in range(self.n_particles):
            self.particles.append([weight, [x_values[i], y_values[i], heading_values[i]]])

    def initialize_particles_uniform(self):
        """
        Initialize the particles uniformly over the world assuming a 3D state (x, y, heading).
        No arguments are required and function always succeeds hence no return value.
        """
        # Initialize particles with uniform weight distribution
        self.particles = []
        weight = 1.0 / self.n_particles
        # Create arrays of uniformly distributed random numbers for each dimension
        heading_values = np.random.uniform(0, 2 * np.pi, self.n_particles)
        spread_value = 250

        if self.state == 1:
            # Intiliaze particles with for TDOA Measurement
            x_values = np.random.uniform(self.inital_measurements[0, 0]- spread_value, self.inital_measurements[0, 0]+spread_value, self.n_particles)
            y_values = np.random.uniform(self.inital_measurements[1, 0]-spread_value, self.inital_measurements[1, 0]+spread_value, self.n_particles)
        elif self.state == 2:
            print('Initializing Particles for Beamformer')
            # Intiliaze particles with for BF Measurement
            half_particles = self.n_particles // 2
            y_values = np.zeros(self.n_particles)
            y_values[:half_particles] += np.random.uniform(self.inital_measurements[1, 1]- spread_value, self.inital_measurements[1, 1]+spread_value, size=half_particles)
            y_values[:half_particles] += np.random.uniform(self.inital_measurements[0, 1]- spread_value, self.inital_measurements[0, 1]+spread_value, size=half_particles)
        
            # y_values[:half_particles] += np.random.uniform(self.inital_measurements[1, 1], 500, size=half_particles)
            # y_values[half_particles:] += np.random.normal(self.inital_measurements[0, 1], 500, size=self.n_particles - half_particles)

            # x_values = np.random.normal(self.inital_measurements[0, 0], 500, size=self.n_particles)
            x_values = np.random.uniform(self.inital_measurements[0, 0]- spread_value, self.inital_measurements[0, 0]+spread_value, self.n_particles)
        elif self.state == 3:
            y_values_1 = np.random.uniform(self.inital_measurements[1, 1]- spread_value, self.inital_measurements[1, 1]+spread_value, size=half_particles)
            y_values_2 = np.random.uniform(self.inital_measurements[0, 1]- spread_value, self.inital_measurements[0, 1]+spread_value, size=half_particles)

            x_values_1 = np.random.uniform(self.inital_measurements[0, 0]- spread_value, self.inital_measurements[0, 0]+spread_value, size=half_particles)
            x_values_2 = np.random.uniform(self.inital_measurements[0, 0]- spread_value, self.inital_measurements[0, 0]+spread_value, size=half_particles)

            heading_values_1 = np.random.uniform(-np.pi, np.pi, half_particles)
            heading_values_2 = np.random.uniform(-np.pi, np.pi, half_particles)



        if self.state == 3:
            for i in range(half_particles):
                self.particles.append([weight, [x_values_1[i], y_values_1[i], heading_values_1[i]]])
                self.particles_2.append([weight, [x_values_2[i], y_values_2[i], heading_values_2[i]]])
        else:     
            # Combine the random values for each particle and append to particles list
            for i in range(self.n_particles):
                self.particles.append([weight, [x_values[i], y_values[i], heading_values[i]]])
            self.save_particles()

    def get_average_state_BF(self):        
        # Initialize variables to store data for both groups
        sum_weights_greater_500 = 0.0
        sum_weights_less_equal_500 = 0.0
        avg_x_greater_500 = 0.0
        avg_y_greater_500 = 0.0
        avg_theta_greater_500 = 0.0
        avg_x_less_equal_500 = 0.0
        avg_y_less_equal_500 = 0.0
        avg_theta_less_equal_500 = 0.0
        
        # Counters for each group
        count_greater_500 = 0
        count_less_equal_500 = 0

        # Iterate over particles to split and compute sums
        for weighted_sample in self.particles:
            weight = weighted_sample[0]
            position = weighted_sample[1]
            if position[1] > 500:
                sum_weights_greater_500 += weight
                avg_x_greater_500 += weight * position[0]
                avg_y_greater_500 += weight * position[1]
                avg_theta_greater_500 += weight * position[2]
                count_greater_500 += 1
            else:
                sum_weights_less_equal_500 += weight
                avg_x_less_equal_500 += weight * position[0]
                avg_y_less_equal_500 += weight * position[1]
                avg_theta_less_equal_500 += weight * position[2]
                count_less_equal_500 += 1

        # Compute mean values for both groups
        mean_greater_500 = [avg_x_greater_500 / sum_weights_greater_500,
                            avg_y_greater_500 / sum_weights_greater_500,
                            avg_theta_greater_500 / sum_weights_greater_500] if count_greater_500 > 0 else [0, 0, 0]
        mean_less_equal_500 = [avg_x_less_equal_500 / sum_weights_less_equal_500,
                            avg_y_less_equal_500 / sum_weights_less_equal_500,
                            avg_theta_less_equal_500 / sum_weights_less_equal_500] if count_less_equal_500 > 0 else [0, 0, 0]

        return mean_greater_500, mean_less_equal_500

    def get_average_state_TDOA(self):
        """
        Compute average state according to all weighted particles

        :return: Average x-position, y-position and orientation
        """

        # Compute sum of all weights
        sum_weights = 0.0
        for weighted_sample in self.particles:
            sum_weights += weighted_sample[0]

        # Compute weighted average
        avg_x = 0.0
        avg_y = 0.0
        avg_theta = 0.0
        for weighted_sample in self.particles:
            avg_x += weighted_sample[0] / sum_weights * weighted_sample[1][0]
            avg_y += weighted_sample[0] / sum_weights * weighted_sample[1][1]
            avg_theta += weighted_sample[0] / sum_weights * weighted_sample[1][2]

        return [avg_x, avg_y, avg_theta]

    def get_max_weight(self):
        """
        Find maximum weight in particle filter.

        :return: Maximum particle weight
        """
        return max([weighted_sample[0] for weighted_sample in self.particles])
    
    def get_particles(self):
        """
        Return x and y coordinates of all particles.

        :return: All particles
        """
        #convert particles to numpy array
        particles = np.array([particle[1] for particle in self.particles])
        return particles

    def save_particles(self):
        """
        Save all particle states to a CSV file.

        :param filename: Name of the CSV file.
        """

        with open('/home/michael-asv/vrx_ws/src/vrx/vrx_ros/scripts/particles_10.csv', 'a') as file:
            writer = csv.writer(file)
            writer.writerow(['Index', 'X', 'Y', 'Heading', 'Weight'])
            for i, particle in enumerate(self.particles):
                writer.writerow([i+1, particle[1][0], particle[1][1], particle[1][2], particle[0]])

    def normalize_weights(self, weighted_samples, measurement):
        """
        Normalize all particle weights.
        """

        # Compute sum weighted samples
        sum_weights = 0.0
        for weighted_sample in weighted_samples:
            sum_weights += weighted_sample[0]

        # Check if weights are non-zero
        if sum_weights < 1e-15:
            print("Weight normalization failed: sum of all weights is {} (weights will be reinitialized)".format(sum_weights))

            # Set uniform weights
            self.count_for_reinitialization += 1
            if self.count_for_reinitialization > 5:
                print("Reinitializing particles, lost track of robot!")
                self.count_for_reinitialization = 0
                self.inital_measurements = measurement
                self.initialize_particles_uniform()
                return self.particles
            else:
                return [[1.0 / len(weighted_samples), weighted_sample[1]] for weighted_sample in weighted_samples]

        # Return normalized weights
        self.count_for_reinitialization = 0
        return [[weighted_sample[0] / sum_weights, weighted_sample[1]] for weighted_sample in weighted_samples]

    def propagate_sample(self, sample, forward_motion, angular_motion):
        """
        Propagate an individual sample with a simple motion model that assumes the robot rotates angular_motion rad and
        then moves forward_motion meters in the direction of its heading. Return the propagated sample (leave input
        unchanged).

        :param sample: Sample (unweighted particle) that must be propagated
        :param forward_motion: Forward motion in meters
        :param angular_motion: Angular motion in radians
        :return: propagated sample
        """
        # 1. rotate by given amount plus additive noise sample (index 1 is angular noise standard deviation)
        propagated_sample = copy.deepcopy(sample)
        propagated_sample[2] += np.random.normal(angular_motion, self.process_noise[1], 1)[0]
        
        #Wrap to pi
        propagated_sample[2] = (propagated_sample[2] + np.pi) % (2 * np.pi) - np.pi

        # Compute forward motion by combining deterministic forward motion with additive zero mean Gaussian noise
        forward_displacement = np.random.normal(forward_motion, self.process_noise[0], 1)[0]

        # 2. move forward
        propagated_sample[0] += forward_displacement * np.cos(propagated_sample[2])
        
        #added small jitter in propagation model
        propagated_sample[1] += forward_displacement * np.sin(propagated_sample[2]) + np.random.normal(0, 4)

        # Make sure we stay within cyclic world
        return propagated_sample