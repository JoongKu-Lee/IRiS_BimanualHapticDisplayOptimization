from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():

    # robot_description = "/home/jiye/opti_ws/src/gen_humanik/model/robots/human_upper_body.urdf"
    # robot_description_semantic = "/home/jiye/opti_ws/config/human_upper_body.srdf"
    robot_description = ParameterValue(Command(['xacro ', '/home/jiye/opti_ws/src/gen_humanik/model/human_upper_body.urdf']), value_type=str)

    with open('/home/jiye/opti_ws/src/gen_humanik/config/human_upper_body.srdf', 'r') as file:
        sementaic_content = file.read()
    


    analyze_humanik_node = Node(
        package='gen_humanik',
        executable='humanik',
        name='humanik',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'robot_description_semantic': sementaic_content}
        ],
    )

    return LaunchDescription([analyze_humanik_node])