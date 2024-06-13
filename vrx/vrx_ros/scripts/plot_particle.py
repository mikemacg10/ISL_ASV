import pandas as pd

import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file
#data = pd.read_csv('/home/michael-asv/vrx_ws/src/vrx/vrx_ros/scripts/BF DATA/30 Second Calls/')
data = pd.read_csv('/home/michael-asv/vrx_ws/src/vrx/vrx_ros/scripts/TDOA DATA/10 Second Calls/4ASV_TDOA_NOISE_50_PF_MN_25.csv')
data = pd.read_csv('/home/michael-asv/vrx_ws/src/vrx/vrx_ros/scripts/poses.csv')
#data = pd.read_csv('/home/michael-asv/vrx_ws/src/vrx/vrx_ros/scripts/TDOA DATA/4ASV_TDOA_NOISE_150_PF_MN_75_30seconds.csv')
np_array = data.values

#error in measurement
error_euc = ((np_array[:,6] - np_array[:,2])**2 + (np_array[:,7] - np_array[:,3])**2)**0.5
error_x = (np_array[:,6] - np_array[:,2])
error_y = (np_array[:,7] - np_array[:,3])

# PF error
error_x_PF_euc = ((np_array[:,0] - np_array[:,2])**2 + (np_array[:,1] - np_array[:,3])**2)**0.5
error_x_PF = (np_array[:,0] - np_array[:,2])
error_y_PF = (np_array[:,1] - np_array[:,3])


# Plot the data
plt.figure(1)
plt.plot(np_array[:,0], np_array[:,1], 'bx', markersize=2, label='Estimated')
plt.plot(np_array[:,2], np_array[:,3], 'r', markersize=1, label='Actual')
plt.plot(np_array[:,6], np_array[:,7], 'g*', markersize=1, label='measured')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('100 particles - sample every 10 seconds\n Sigma = 0.1s')
plt.legend()
# plt.axis('equal')  # Set equal axis scaling

# Plot the error in measurement
plt.figure(2)
plt.plot(error_x, 'bx-', markersize=1, label='Measurement error')
plt.plot(error_x_PF, 'rx-', markersize=1, label='PF error')
plt.plot([0, len(error_y)], [0, 0], 'k--')  # Add dashed line at y=0
plt.plot([0, len(error_y)], [100, 100], 'k--')  # Add dashed line at y=100
plt.plot([0, len(error_y)], [-100, -100], 'k--')  # Add dashed line at y=100
plt.xlabel('Measurment number')
plt.ylabel('Error in measurement [m]')
plt.title('Error in x measurement')
plt.legend()
plt.show()

# Plot the error in measurement
plt.figure(3)
plt.plot(error_y, 'bx-', markersize=1, label='Measurement error')
plt.plot(error_y_PF, 'rx-', markersize=1, label='PF error')
plt.plot([0, len(error_y)], [0, 0], 'k--')  # Add dashed line at y=0
plt.plot([0, len(error_y)], [100, 100], 'k--')  # Add dashed line at y=100
plt.plot([0, len(error_y)], [-100, -100], 'k--')  # Add dashed line at y=100
plt.xlabel('Measurement number')
plt.ylabel('Error in measurement')
plt.title('Error in y measurement')
plt.legend()
plt.show()

# Plot the error in measurement
plt.figure(3)
plt.plot(error_euc, 'bx-', markersize=1, label='Measurement error')
plt.plot(error_x_PF_euc, 'rx-', markersize=1, label='PF error')
plt.xlabel('Measurement number')
plt.ylabel('Error in measurement')
plt.title('Error in y measurement')
plt.legend()
plt.show()


import numpy as np

# Assuming np_array is your NumPy array containing the data

# Calculate squared errors in x-direction
squared_error_x_PF = (error_x_PF) ** 2
squared_error_x = (error_x) ** 2


# Calculate squared errors in y-direction
squared_error_y_PF = (error_y_PF) ** 2
squared_error_y = (error_y) ** 2

# calculate squared errors in euclidean distance
squared_error_euc_PF = (error_x_PF_euc) ** 2
squared_error_euc = (error_euc) ** 2

# Calculate mean squared error in x-direction
mse_x = np.mean(squared_error_x)
mse_x_PF = np.mean(squared_error_x_PF)

# Calculate mean squared error in y-direction
mse_y = np.mean(squared_error_y)
mse_y_PF = np.mean(squared_error_y_PF)

# Calculate mean squared error in euclidean distance
mse_euc = np.mean(squared_error_euc)
mse_euc_PF = np.mean(squared_error_euc_PF)

# Calculate root mean squared error in x-direction
rmse_x = np.sqrt(mse_x)
rmse_x_PF = np.sqrt(mse_x_PF)

# Calculate root mean squared error in y-direction
rmse_y = np.sqrt(mse_y)
rmse_y_PF = np.sqrt(mse_y_PF)

# Calculate root mean squared error in euclidean distance
rmse_euc = np.sqrt(mse_euc)
rmse_euc_PF = np.sqrt(mse_euc_PF)


print("Root Mean Squared Error in x-direction:", rmse_x)
print("Root Mean Squared Error in y-direction:", rmse_y)
print("Root Mean Squared Error in x-direction PF:", rmse_x_PF)
print("Root Mean Squared Error in y-direction PF:", rmse_y_PF)
print("Root Mean Squared Error in euclidean distance:", rmse_euc)
print("Root Mean Squared Error in euclidean distance PF:", rmse_euc_PF)



