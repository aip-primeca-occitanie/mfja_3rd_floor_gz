import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_path = get_package_share_directory('mfja_3rd_floor_bringup')
    base_launch = os.path.join(pkg_path, 'launch', 'mfja_3rdf_kuka.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch)),
    ])
