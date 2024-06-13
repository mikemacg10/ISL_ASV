# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    bridge_competition_topics = LaunchConfiguration(
        'bridge_competition_topics').perform(context).lower() == 'true'
    robot = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    robot_urdf = LaunchConfiguration('urdf').perform(context)
    gz_paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    competition_mode = LaunchConfiguration('competition_mode').perform(context).lower() == 'true'
    extra_gz_args = LaunchConfiguration('extra_gz_args').perform(context)
    simulaiton_mode = LaunchConfiguration('simulation').perform(context)

    launch_processes = []

    models = []
    if config_file and config_file != '':
        with open(config_file, 'r') as stream:
            models = Model.FromConfig(stream)
    else:
        if simulaiton_mode == 'TDOA':
            m_1 = Model('wamv', 'wam-v', [-5000, -5000, 0, 0, 0, 0])
            m_2 = Model('wamv_m_2', 'wam-v', [-5000, 5000, 0, 0, 0, 0])
            m_3 = Model('wamv_m_3', 'wam-v', [5000, -5000, 0, 0, 0, 0])
            m_4 = Model('wamv_m_4', 'wam-v', [5000, 5000, 0, 0, 0, 0])
            m_5 = Model('wamv_m_5', 'wam-v', [0, 0, 0, 0, 0, 0])
            if robot_urdf and robot_urdf != '':
                m_1.set_urdf(robot_urdf)
                m_2.set_urdf(robot_urdf)
                m_3.set_urdf(robot_urdf)
                m_4.set_urdf(robot_urdf)
                m_5.set_urdf(robot_urdf) 
            models.append(m_1)
            models.append(m_2)
            models.append(m_3)
            models.append(m_4)
            models.append(m_5)
        elif simulaiton_mode == 'BF':
            m_1 = Model('wamv', 'wam-v', [-500, 1000, 0, 0, 0, 0])
            m_5 = Model('wamv_m_5', 'wam-v', [-500, 0, 0, 0, 0, 0])
            if robot_urdf and robot_urdf != '':
                m_1.set_urdf(robot_urdf)
                m_5.set_urdf(robot_urdf)
            models.append(m_1)
            models.append(m_5)          
      


    world_name, ext = os.path.splitext(world_name)
    launch_processes.extend(vrx_gz.launch.simulation(world_name, headless, 
                                                     gz_paused, extra_gz_args))
    world_name_base = os.path.basename(world_name)
    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_name_base, models, robot))

    if (sim_mode == 'bridge' or sim_mode == 'full') and bridge_competition_topics:
        launch_processes.extend(vrx_gz.launch.competition_bridges(world_name_base, competition_mode))

    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'world',
            default_value='sydney_regatta',
            description='Name of world'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='full',
            description='Simulation mode: "full", "sim", "bridge".'
                        'full: spawns robot and launch ros_gz bridges, '
                        'sim: spawns robot only, '
                        'bridge: launch ros_gz bridges only.'),
        DeclareLaunchArgument(
            'bridge_competition_topics',
            default_value='True',
            description='True to bridge competition topics, False to disable bridge.'),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='YAML configuration file to spawn'),
        DeclareLaunchArgument(
            'robot',
            default_value='',
            description='Name of robot to spawn if specified. '
                        'This must match one of the robots in the config_file'),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='True to run simulation headless (no GUI). '),
        DeclareLaunchArgument(
            'urdf',
            default_value='',
            description='URDF file of the wam-v model. '),
        DeclareLaunchArgument(
            'paused',
            default_value='False',
            description='True to start the simulation paused. '),
        DeclareLaunchArgument(
            'competition_mode',
            default_value='False',
            description='True to disable debug topics. '),
        DeclareLaunchArgument(
            'extra_gz_args',
            default_value='',
            description='Additional arguments to be passed to gz sim. '),
        DeclareLaunchArgument(
            'Simulation',
            default_value='TDOA',
            description='Choose between TDOA or BF Sim.'),
        OpaqueFunction(function=launch),
    ])
