from .particle_filter_base import ParticleFilter
from vrx_ros.core.resampling.resampler import Resampler

import numpy as np
import time


class ParticleFilterRangeOnly(ParticleFilter):
    """
    Notes:
        * State is (x, y, heading), where x and y are in meters and heading in radians
        * State space assumed limited size in each dimension, world is cyclic (hence leaving at x_max means entering at
        x_min)
        * propagation and measurement models are largely hardcoded (except for standard deviations).
    """

    def __init__(self,
                 number_of_particles,
                 limits,
                 process_noise,
                 measurement_noise,
                 measurement,
                 state,
                 resampling_threshold,
                 resampling_algorithm):
        """
        Initialize the SIR range measurement only particle filter. Largely copied from the SIR particle filter.

        :param number_of_particles: Number of particles.
        :param limits: List with maximum and minimum values for x and y dimension: [xmin, xmax, ymin, ymax].
        :param process_noise: Process noise parameters (standard deviations): [std_forward, std_angular].
        :param measurement_noise: Measurement noise parameter range (standard deviation): std_range.
        :param resampling_algorithm: Algorithm that must be used for core.
        """
        # Initialize particle filter base class
        ParticleFilter.__init__(self, number_of_particles, limits, process_noise, measurement_noise, measurement, state)

        # Set SIR specific properties
        self.resampling_threshold = resampling_threshold
        self.resampling_algorithm = resampling_algorithm
        self.resampler = Resampler()
        print("ParticleFilter initialized")

    def needs_resampling(self):
        """
        Method that determines whether not a core step is needed for the current particle filter state estimate.
        The sampling importance core (SIR) scheme resamples every time step hence always return true.

        :return: Boolean indicating whether or not core is needed.
        """
        sum_weights_squared = 0
        for par in self.particles:
            sum_weights_squared += par[0] * par[0]

        print("sum_weights_squared: ", sum_weights_squared)

        return 1.0 / sum_weights_squared < self.resampling_threshold
        # return True

    @staticmethod
    def compute_likelihood(sample, measurement, measurement_noise, STATE=1, manuver_occured=True):
        """
        Compute the importance weight p(z|sample) for a specific measurement given sample state and landmarks.

        :param sample: Sample (unweighted particle) that must be propagated
        :param measurement: List with measurements, for each landmark distance_to_landmark, unit is meters
        :param measurement_noise: Measurement noise parameter range (standard deviation): std_range.
        :return Importance weight
        """

        if STATE == 1:
            # Compute the difference between the true and expected distance measurements
            # Compute the difference between the true and expected distance measurements
            x_diff = sample[0] - measurement[0, 0]
            y_diff = sample[1] - measurement[1, 0]

            # Compute the probability using the difference and measurement noise
            p_z_given_x_distance = np.exp(-(x_diff * x_diff) / (2.0 * measurement_noise * measurement_noise))
            p_z_given_y_distance = np.exp(-(y_diff * y_diff) / (2.0 * measurement_noise * measurement_noise))

            # Compute the likelihood for the current landmark
            measurement_likelihood_sample = p_z_given_x_distance * p_z_given_y_distance
        elif STATE == 2:
            if manuver_occured == True:
                    # Compute the difference between the true and expected distance measurements
                #print(measurement[0, 1])

                # x_diff = sample[0] - measurement[0, 0]
                # y_diff_1 = sample[1] - measurement[1, 1]
                # # Compute the probability using the difference and measurement noise
                # p_z_given_x_distance = np.exp(-0.5 * np.square(x_diff) / np.square(measurement_noise))
                # p_z_given_y_distance_1 = np.exp(-0.5 * np.square(y_diff_1) / np.square(measurement_noise))

                # # Compute the likelihood for the current landmark
                # measurement_likelihood_sample = p_z_given_y_distance_1 * p_z_given_x_distance

                x_diff_1 = sample[0] - measurement[0, 0]
                x_diff_2 = sample[0] - measurement[1, 0]
                y_diff_1 = sample[1] - measurement[0, 1]
                y_diff_2 = sample[1] - measurement[1, 1]

                # Compute the probability using the difference and measurement noise
                p_z_given_x_distance_1 = np.exp(-0.5 * np.square(x_diff_1) / np.square(measurement_noise))
                p_z_given_x_distance_2 = np.exp(-0.5 * np.square(x_diff_2) / np.square(measurement_noise))
                p_z_given_y_distance_1 = np.exp(-0.5 * np.square(y_diff_1) / np.square(measurement_noise))
                p_z_given_y_distance_2 = np.exp(-0.5 * np.square(y_diff_2) / np.square(measurement_noise))

                # Compute the likelihood for the current landmark
                measurement_likelihood_sample = ((p_z_given_y_distance_1 * p_z_given_x_distance_1)) + ((p_z_given_y_distance_2*p_z_given_x_distance_2))
            else:
                pass

        
                
            


        # Return the importance weight
        return measurement_likelihood_sample

    def update(self, robot_forward_motion, robot_angular_motion, measurements, STATE):
        """
        Process a measurement given the measured robot displacement and resample if needed.

        :param robot_forward_motion: Measured forward robot motion in meters.
        :param robot_angular_motion: Measured angular robot motion in radians.
        :param measurements: Measurements.
        """

        # Loop over all particles
        new_particles = []

        # Propagate and compute weights for all particles in parallel
        propagated_states = [self.propagate_sample(par[1], robot_forward_motion, robot_angular_motion) for par in self.particles]
        weights = [self.compute_likelihood(state, measurements, self.measurement_noise, STATE) for state in propagated_states]
        
        new_particles = [[par[0] * weight, state] for par, weight, state in zip(self.particles, weights, propagated_states)]
        print("particles updating")

        # Update particles
        self.particles = self.normalize_weights(new_particles, measurements)

        # Resample if needed
        if self.needs_resampling():
            print("Resampling")
            self.particles = self.resampler.resample(self.particles, self.n_particles, self.resampling_algorithm)
