
# Prepare the system (ROS 2, Gazebo Garden)
Follow instructions from the provided link
https://github.com/osrf/vrx/wiki/preparing_system_tutorial

# Clone the ASV repository
```bash
mkdir workspace
cd workspace
git clone https://github.com/mikemacg10/ISL_ASV.git
cd ISL_ASV

# Build the ASV package
colcon build --merge install
```

# TO run the simulation:
```bash
source install/setup.bash
ros2 launch vrx_gz competition.launch.py simulation:=BF
ros2 launch vrx_ros BF.launch.py
```

The estimated positon from the PF will be saved in pose.csv in the vrx_ros folder. 



