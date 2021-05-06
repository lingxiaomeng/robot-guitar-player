import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
import numpy as np

group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name, ns='/my_gen3_lite',
                                            robot_description="/my_gen3_lite/robot_description")
# ("x,y,z,j1,j2,j3,j4,j5,j6,e1,e2,e3,e4,e5,e6\n")

data = np.loadtxt('press_data.csv', delimiter=',')
print(data.shape)
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]
j1 = data[:, 3]
j2 = data[:, 4]
j3 = data[:, 5]
j4 = data[:, 6]
j5 = data[:, 7]
j6 = data[:, 8]
e1 = data[:, 9]
e2 = data[:, 10]
e3 = data[:, 11]
e4 = data[:, 12]
e5 = data[:, 13]
e6 = data[:, 14]

f = open('force_data.csv', 'w+')

for i in range(len(z)):
    cz = z[i]
    cj = [j1[i], j2[i], j3[i], j4[i], j5[i], j6[i]]
    ce = [e1[i], e2[i], e3[i], e4[i], e5[i], e6[i]]
    current_jacobian_matrix = group.get_jacobian_matrix(cj)
    np_jacobian = np.array(current_jacobian_matrix)
    np_effort = np.array(ce)
    F = (np.linalg.inv(np_jacobian.transpose())).dot(np_effort.transpose())
    f.write('%f,%f,%f,%f,%f,%f\n' % (F[0], F[1], F[2], F[3], F[4], F[5]))

f.close()

# current_joint_values = group.get_current_joint_values()
# print(current_joint_values)

# current_joint_values = [-0.051809668478619564, 0.3591648235685554, 2.814633170767457, -1.4581914843197303, -0.6271339566650918, -1.5182644090286477]


# current_jacobian_matrix = group.get_jacobian_matrix(current_joint_values)
# print(current_jacobian_matrix)
