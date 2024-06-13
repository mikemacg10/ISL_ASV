from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.events import Shutdown

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vrx_ros',
            namespace='TDOA',
            executable='TDOA_3D.py',
            name='TDOA'
        ),
        Node(
            package='vrx_ros',
            namespace='PF',
            executable='PF.py',
            name='sim'
        ),
        Node(
            package='vrx_ros',
            namespace='ASV_control',
            executable='ASV_control',
            name='sim'
        ),
        Node(
            package='vrx_ros',
            namespace='controller_asv_1',
            executable='controller_asv_1',
            name='sim'
        ),
        Node(
            package='vrx_ros',
            namespace='controller_asv_2',
            executable='controller_asv_2',
            name='sim'
        ),
        Node(
            package='vrx_ros',
            namespace='controller_asv_3',
            executable='controller_asv_3',
            name='sim'
        ),
        Node(
            package='vrx_ros',
            namespace='controller_asv_4',
            executable='controller_asv_4',
            name='sim'
        )
    ])
