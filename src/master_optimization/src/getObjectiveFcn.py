import os
import csv

# include create_launchfile function
from create_urdf import create_urdf
from create_launcefile import create_launchfile
import random

def getObjectivesFcn(val):

    try:

        x = val[0]
        y = val[1]
        z = val[2]
        roll = val[3]
        pitch = val[4]
        yaw = val[5]
        tcp_angle = val[6]
        print(x, y, z, roll, pitch, yaw, tcp_angle)

        num = random.randint(0, 1000000)
        # num=1

        robot_namespace = 'trial'+str(num)

        working_dir = os.getcwd()

        # Create a URDF file for the robot
        create_urdf(x, y, z, roll, pitch, yaw, tcp_angle, robot_namespace)

        # Create a launch file for the robot
        create_launchfile(robot_namespace)

        # Run the launch file
        os.system('colcon build --packages-select master_optimization')
        os.system('ros2 launch master_optimization analyze_robotik'+robot_namespace+'.py')

        # get current directory


        # calculate the score
        leftpanda_data = open(working_dir+'/install/master_optimization/share/master_optimization/data/leftpanda_data'+robot_namespace+'.csv', 'r')
        rightpanda_data = open(working_dir+'/install/master_optimization/share/master_optimization/data/rightpanda_data'+robot_namespace+'.csv', 'r')

        MAX_IK = 100
        MAX_FORCE = 100
        MAX_TORQUE = 10

        # left_IKWithoutCollision = [row[0] for row in csv.reader(leftpanda_data)][1:]
        # left_IKWithoutCollision = [float(i) for i in left_IKWithoutCollision]

        left_IKWithoutCollision = [row[1] for row in csv.reader(leftpanda_data)][1:]
        left_IKWithoutCollisionRedundancy = [min(float(i)/MAX_IK, 1) for i in left_IKWithoutCollision]
        left_IKWithoutCollision = [1 if float(i) > 0 else 0 for i in left_IKWithoutCollision]

        leftpanda_data.seek(0)
        left_RenderableForce_min = [row[2] for row in csv.reader(leftpanda_data)][1:]
        left_RenderableForce_min = [min(float(i)/MAX_FORCE, 1) for i in left_RenderableForce_min][1:]

        leftpanda_data.seek(0)
        left_RenderableTorque_min = [row[3] for row in csv.reader(leftpanda_data)][1:]
        left_RenderableTorque_min = [min(float(i)/MAX_TORQUE, 1) for i in left_RenderableTorque_min][1:]

        right_IKWithoutCollision = [row[1] for row in csv.reader(rightpanda_data)][1:]
        right_IKWithoutCollisionRedundancy = [min(float(i)/MAX_IK, 1) for i in right_IKWithoutCollision]
        right_IKWithoutCollision = [1 if float(i) > 0 else 0 for i in right_IKWithoutCollision]
        rightpanda_data.seek(0)
        right_RenderableForce_min = [row[2] for row in csv.reader(rightpanda_data)][1:]
        right_RenderableForce_min = [min(float(i)/MAX_FORCE, 1) for i in right_RenderableForce_min][1:]

        rightpanda_data.seek(0)
        right_RenderableTorque_min = [row[3] for row in csv.reader(rightpanda_data)][1:]
        right_RenderableTorque_min = [min(float(i)/MAX_TORQUE, 1) for i in right_RenderableTorque_min][1:]

        WorkSpace_Score = 0.5*(sum(left_IKWithoutCollision)+sum(left_IKWithoutCollisionRedundancy)) + 0.5*(sum(right_IKWithoutCollision)+sum(right_IKWithoutCollisionRedundancy))
        Haptics_Score = 0.5*(sum(left_RenderableForce_min)+sum(left_RenderableTorque_min)) + 0.5*(sum(right_RenderableForce_min)+sum(right_RenderableTorque_min))
        Score = -(WorkSpace_Score + Haptics_Score)


        leftpanda_data.close()
        rightpanda_data.close()

        # Delete the URDF and launch files
        # print("Deleting files")
        os.system('rm src/master_optimization/model/robots/optimization/human_tworobot_'+robot_namespace+'.urdf')
        os.system('rm src/master_optimization/launch/optimization/analyze_robotik'+robot_namespace+'.py')

        # open csv file and write x, y, z, roll, pitch, yaw, tcp_angle, WorkSpace_Score, Haptics_Score, Score
        with open(working_dir+'/optimization_data.csv', mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([x, y, z, roll, pitch, yaw, tcp_angle, WorkSpace_Score, Haptics_Score])

        print('Workspace_Score: ', WorkSpace_Score)
        print('Haptics_Score: ', Haptics_Score)
        print('Score: ', Score)



        return Score
    
    except Exception as e:
        print(e)
        print("##############################################")
        return 0

# getObjectivesFcn([-0.54223011,  0.30658228,  0.18673476, -0.83692792,  0.75907133,-0.76737354, -0.10024007])
# getObjectivesFcn([-0.205,  0.066,  0.262, -0.900,  0.177,-0.219, -0.569])