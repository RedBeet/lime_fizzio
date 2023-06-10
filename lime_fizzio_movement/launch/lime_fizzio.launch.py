import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor = Node(
        package='lime_fizzio_movement',
        executable='lime_fizzio_motor',
        output='screen',
        parameters=[]
    )

    sensor = Node(
        package='lime_fizzio_movement',
        executable='lime_fizzio_sensor',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        motor,
        sensor,
    ])