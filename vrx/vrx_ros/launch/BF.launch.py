# Launch file for BF simulaiton. 
# This launch file will launch the Beamformer_DS_MVDR.py, ASV_control, PF.py, and controller_asv_1 node
# go to BF.py and PF. py to adjuct measurment noise and PF parameters.

#Michael MacGillivray

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vrx_ros',
            namespace='BF',
            executable='BF_1_main.py',
            name='BeanFormer'
        ),
        # Node(
        #     package='vrx_ros',
        #     namespace='BF',
        #     executable='BF_2_main.py',
        #     name='BeanFormer'
        # ),
        Node(
            package='vrx_ros',
            namespace='ASV_control',
            executable='ASV_control',
            name='sim'
        ),
        Node(
            package='vrx_ros',
            namespace='PF',
            executable='PF.py',
            name='sim'
        ),
        Node(
            package='vrx_ros',
            namespace='controller_asv_1',
            executable='controller_asv_1',
            name='sim'
        )

    ])