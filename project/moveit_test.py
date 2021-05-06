import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
import numpy as np

group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name, ns='/my_gen3_lite',
                                            robot_description="/my_gen3_lite/robot_description")
current_joint_values = group.get_current_joint_values()
print(current_joint_values)

current_joint_values = [-0.051809668478619564, 0.3591648235685554, 2.814633170767457, -1.4581914843197303, -0.6271339566650918, -1.5182644090286477]


current_jacobian_matrix = group.get_jacobian_matrix(current_joint_values)
print(current_jacobian_matrix)
