# Simple launch file for the gui and rviz2

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_publisher_gui',
            executable='tf_publisher_gui',
            name='tf_publisher_gui_1',
            parameters=[
            {'parent_frame': 'link_0'},
            {'child_frame': 'link_1'},
            {'x': 0.930},
            {'y': -1.1},
            {'z': 1.1},
            {'roll': 0.383972},
            {'pitch': -0.541052},
            {'yaw': 0.733038},
            {'hz': 10}
            ],
            output='screen'
        ),
        Node(
            package='tf_publisher_gui',
            executable='tf_publisher_gui',
            name='tf_publisher_gui_2',
            parameters=[
            {'parent_frame': 'link_1'},
            {'child_frame': 'link_2'},
            {'x': 0.170},
            {'y': 0.650},
            {'z': 0.460},
            {'roll': -0.401426},
            {'pitch': 0.698132},
            {'yaw': 0.383972},
            {'hz': 10}
            ],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', get_package_share_directory('tf_publisher_gui') + '/rviz/demo.rviz']
        )
    ])

# This launch file starts the GUI and RViz2. The GUI is started with the tf_publisher_gui executable and RViz2 with the rviz2 executable. The output='screen' argument is used to show the output of the nodes in the terminal. The launch file is saved in the launch directory of the package with the name demo.launch.py.
