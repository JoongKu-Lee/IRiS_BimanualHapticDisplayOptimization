import os

def create_launchfile(robot_namespace):
    launch_filename = 'analyze_robotik' + robot_namespace + '.py'
    launch_path = os.getcwd()+"/src/master_optimization/launch/optimization/"
    # file = open(urdf_path+urdf_filename, 'w')

    urdf_path = os.getcwd()+"/src/master_optimization/model/robots/optimization/human_tworobot_"+robot_namespace+".urdf"
    srdf_path = os.getcwd()+"/src/master_optimization/config/human_tworobot_forrobotIK.srdf"

    # Create a URDF file for the robot
    with open(launch_path+launch_filename, 'w') as file:
        file.write('import launch\n')
        file.write('from launch import LaunchDescription\n')
        file.write('from launch_ros.actions import Node\n')
        file.write('from launch_ros.descriptions import ParameterValue\n')
        file.write('from launch.substitutions import Command\n')
        file.write('from launch.actions import RegisterEventHandler, EmitEvent\n')
        file.write('from launch.events import Shutdown\n')
        
        file.write('def generate_launch_description():\n')
        file.write('\n')
        file.write('    robot_namespace = \''+robot_namespace+'\'\n')
        file.write('    robot_description = ParameterValue(Command([\'xacro \', \''+urdf_path+'\']), value_type=str)\n')
        file.write('\n')
        file.write('    with open(\''+srdf_path+'\', \'r\') as file:\n')
        file.write('        semantic_content = file.read()\n')

        file.write('\n')


        file.write('\n')

        cal_node_name = 'calculate_score_node_'+robot_namespace
        state_pub_node_name = 'robot_state_publisher_node_'+robot_namespace
        remap_des = robot_namespace+"/robot_description"
        remap_des_sem = robot_namespace+"/robot_description_semantic"
        remap_joint_states = robot_namespace+"/joint_states"


        file.write('    robot_state_publisher = Node(\n')
        file.write('        package=\'robot_state_publisher\',\n')
        file.write('        executable=\'robot_state_publisher\',\n')
        file.write(f'        name=\'{state_pub_node_name}\',\n')
        file.write('        output=\'log\',\n')
        file.write('        parameters=[{\'robot_description\': robot_description}],\n')
        file.write('        remappings=[(\'robot_description\', \''+remap_des+'\'),\n')
        file.write('                    (\'joint_states\', \''+remap_joint_states+'\')],\n')
        file.write('\n')
        file.write('    )\n')


        file.write('    calculate_score_node = Node(\n')
        file.write('        package=\'master_optimization\',\n')
        file.write('        executable=\'calculate_score\',\n')
        file.write(f'        name=\'{cal_node_name}\',\n')
        file.write('        output=\'log\',\n')
        file.write('        parameters=[{\'robot_description\': robot_description, \'robot_description_semantic\': semantic_content, \'robot_namespace\': robot_namespace}],\n')
   
        file.write('\n')
        file.write('    )\n')
        file.write('\n')

        file.write('    shutdown_all_nodes_event_handler = RegisterEventHandler(\n')
        file.write('        event_handler = launch.event_handlers.OnProcessExit(\n')
        file.write('            target_action = calculate_score_node,\n')
        file.write('            on_exit = [EmitEvent(event=Shutdown())]\n')
        file.write('        )\n')
        file.write('    )\n')
        file.write('\n')

        # file.write('    return LaunchDescription([calculate_score_node, robot_state_publisher, shutdown_all_nodes_event_handler])\n')
        file.write('    return LaunchDescription([robot_state_publisher, calculate_score_node, shutdown_all_nodes_event_handler])\n')

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description, 'robot_description_semantic': sementic_content}],
    # )

    # shutdown_all_nodes_event_handler = RegisterEventHandler(
    #     event_handler = OnProcessExit(
    #         target_action = calculate_score_node,
    #         on_exit = [EmitEvent(event=Shutdown())]
    #     )
    # )



    # return LaunchDescription([calculate_score_node, robot_state_publisher, shutdown_all_nodes_event_handler])

        





    file.close()

# create_launchfile('trial11113')
    