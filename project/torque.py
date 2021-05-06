import rospy
# import moveit_commander
# from moveit_commander import MoveGroupCommander
import numpy as np
from math import sqrt

from robot_api import Robot_Api

f = open('press_data.csv', 'w+')
f.write("x,y,z,j1,j2,j3,j4,j5,j6,e1,e2,e3,e4,e5,e6\n")


def cal_pose(x, y, dx, dy):
    rx = x + sqrt(0.5) * dx + sqrt(0.5) * dy
    ry = y + sqrt(0.5) * dx - sqrt(0.5) * dy
    return rx, ry


def write_file(x, y, z, e, joint_position):
    joint_position = np.array(joint_position)
    joint_position = joint_position - 360
    joint_position[joint_position < -180] = joint_position[joint_position < -180] + 360
    j = joint_position / 180 * np.pi
    f.write(f"{x},{y},{z},{j[0]},{j[1]},{j[2]},{j[3]},{j[4]},{j[5]},{e[0]},{e[1]},{e[2]},{e[3]},{e[4]},{e[5]}\n")


def test_arm(arm: Robot_Api):
    while True:
        a = input("control robot")
        x, y, z = arm.get_pose()
        efforts = arm.get_efforts()
        joint_position = arm.get_joint_positions()
        write_file(x, y, z, efforts, joint_position)
        print(f"x:{x} y:{y} z:{z}")
        delta = 0.0001
        if (a == 'w'):
            x, y = cal_pose(x, y, 0, -delta)
            arm.go_to_pose(x, y, z)
            arm.wait_for_action_end_or_abort()
        if (a == 'W'):
            x, y = cal_pose(x, y, 0, -10 * delta)
            arm.go_to_pose(x, y, z)
            arm.wait_for_action_end_or_abort()
        elif (a == 'a'):
            x, y = cal_pose(x, y, -delta, 0)
            arm.go_to_pose(x, y, z)
            arm.wait_for_action_end_or_abort()
        elif (a == 'A'):
            x, y = cal_pose(x, y, -delta * 10, 0)
            arm.go_to_pose(x, y, z)
            arm.wait_for_action_end_or_abort()
        elif (a == 's'):
            x, y = cal_pose(x, y, 0, delta)
            arm.go_to_pose(x, y, z)
            arm.wait_for_action_end_or_abort()
        elif (a == 'S'):
            x, y = cal_pose(x, y, 0, delta * 10)
            arm.go_to_pose(x, y, z)
            arm.wait_for_action_end_or_abort()
        elif (a == 'd'):
            x, y = cal_pose(x, y, delta, 0)
            arm.go_to_pose(x, y, z)
            arm.wait_for_action_end_or_abort()
        elif (a == 'D'):
            x, y = cal_pose(x, y, delta * 10, 0)
            arm.go_to_pose(x, y, z)
            arm.wait_for_action_end_or_abort()
        elif (a == 'q'):
            arm.go_to_pose(x, y, z + delta)
            arm.wait_for_action_end_or_abort()
        elif (a == 'Q'):
            arm.go_to_pose(x, y, z + delta * 10)
            arm.wait_for_action_end_or_abort()
        elif (a == 'e'):
            arm.go_to_pose(x, y, z - delta)
            arm.wait_for_action_end_or_abort()
        elif (a == 'E'):
            arm.go_to_pose(x, y, z - delta * 10)
            arm.wait_for_action_end_or_abort()
        elif (a == 'ok'):
            f.close()

            break


if __name__ == "__main__":
    arm = Robot_Api("my_gen3_lite")
    arm.arm_init()
    # arm.go_to_pose(0.373, -0.032, 0.045, theta_x=0, theta_y=180,
    #                theta_z=45, theta_change=True)
    # arm.wait_for_action_end_or_abort()
    test_arm(arm)

#
# group_name='arm'
# group=moveit_commander.MoveGroupCommander(group_name,ns='/my_gen3_lite',robot_description="/my_gen3_lite/robot_description")
# current_joint_values=group.get_current_joint_values()
#
# current_jacobian_matrix=numpy.zeros((6,6))
# current_jacobian_matrix=group.get_jacobian_matrix(current_joint_values)
# print(current_jacobian_matrix)
