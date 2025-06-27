__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile


def generate_launch_description():
    namespaces = LaunchConfiguration('namespaces', default='drone1,drone2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespaces',
            default_value='drone1,drone2',
            description='Comma-separated list of namespaces (e.g., "drone1,drone2")'
        ),
        Node(
            package='px4_offboard',
            executable='visualizermultiagent',  # Make sure this runs PX4VisualizerArray
            name='visualizermultiagent',
            parameters=[{
                'namespaces': namespaces
            }]
        ),
        OpaqueFunction(function=launch_setup),
    ])


def patch_rviz_config(original_config_path, namespaces):
    """
    Patch the RViz config to insert multiple namespaces into relevant places.
    Assumes '__NS__' is used as a placeholder in the config and is repeated as needed.
    """
    with open(original_config_path, 'r') as f:
        content = f.read()

    # Replace __NS__ with actual namespace (if only one)
    # For multiple namespaces, this should ideally be templated per marker/topic
    if '__NS__' in content:
        # Use first namespace for fallback or example
        fallback_ns = namespaces[0] if namespaces else ''
        content = content.replace('__NS__', f'/{fallback_ns}')

    # Write patched content to temp file
    tmp_rviz_config = tempfile.NamedTemporaryFile(delete=False, suffix='.rviz')
    tmp_rviz_config.write(content.encode('utf-8'))
    tmp_rviz_config.close()

    return tmp_rviz_config.name


def launch_setup(context, *args, **kwargs):
    namespaces_str = LaunchConfiguration('namespaces').perform(context)
    namespaces = [ns.strip() for ns in namespaces_str.split(',') if ns.strip()]

    rviz_config_path = os.path.join(
        get_package_share_directory('px4_offboard'), 'visualize.rviz'
    )
    patched_config = patch_rviz_config(rviz_config_path, namespaces)

    return [
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', patched_config]
        )
    ]
