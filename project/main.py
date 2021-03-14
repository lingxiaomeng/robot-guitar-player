from __future__ import print_function

import time

from kortex_driver.msg import BaseCyclic_Feedback, ActionEvent

from robot_api import Robot_Api

import rospy


class Main:
    def __init__(self):
        self.left_arm = Robot_Api("my_left_arm")
        self.right_arm = Robot_Api("my_right_arm")
        self.right_x6 = 0.292414
        self.right_y6 = -0.137629
        self.right_z_free = 0.045

        self.left_x6 = 0.373
        self.left_y6 = -0.032
        self.left_z_free = 0.05

        self.string_heights = [0.01193, 0.01268, 0.01332, 0.01330, 0.01317, 0.01296]
        for i in range(len(self.string_heights)):
            self.string_heights[i] = self.string_heights[i] - self.string_heights[5] + 0.0439
        self.right_z = self.string_heights
        # print(self.right_z)
        # [0.0428, 0.0432, 0.044, 0.0443, 0.0443, 0.0439]

        self.left_grades = [0.03635, 0.03635 + 0.03431, 0.03635 + 0.03431 + 0.03239]
        self.left_strings = [0.028666, 0.028666, 0.014333, 0.014333, 0, 0]

        dis_string = 0.05292 / 5
        self.right_strings = [dis_string * 5, dis_string * 4, dis_string * 3, dis_string * 2, dis_string * 1, 0]

    def init_start(self):
        feedback = rospy.wait_for_message("/" + self.right_arm.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.right_x6 = feedback.base.commanded_tool_pose_x
        self.right_y6 = feedback.base.commanded_tool_pose_y
        self.right_z_free = 0.05

        feedback = rospy.wait_for_message("/" + self.left_arm.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.left_x6 = feedback.base.commanded_tool_pose_x
        self.left_y6 = feedback.base.commanded_tool_pose_y
        self.left_z_free = 0.035

    def finished(self):
        left_x, left_y, left_z = self.left_arm.get_pose()
        self.left_arm.go_to_pose(left_x, left_y, 0.04)

    def go_to_pose(self, left_x, left_y, left_z, right_x, right_y, right_z):
        left_len = len(left_x)
        right_len = len(right_x)
        left_idx = 0
        right_idx = 0
        left_state = 0
        right_state = 0
        while True:
            if left_state == 0:
                self.left_arm.go_to_pose(left_x[left_idx], left_y[left_idx], left_z[left_idx])
                left_state = 1
            if right_state == 0:
                self.right_arm.go_to_pose(right_x[right_idx], right_y[right_idx], right_z[right_idx])
                right_state = 1

            left_res, right_res = self.wait_for_action_end_or_abort()
            if left_res and left_state == 1:
                left_state = 0
                left_idx += 1
                if left_idx >= left_len:
                    left_state = 2
                    rospy.loginfo("LEFT ARM FINISHED")
            if right_res and right_state == 1:
                right_state = 0
                right_idx += 1
                if right_idx >= right_len:
                    right_state = 2
                    rospy.loginfo("RIGHT ARM FINISHED")

    def play(self, string, left_grade):
        right_dis_start = 0.003
        right_dis_end = 0.003

        right_sx = self.right_x6 - (self.right_strings[string - 1] - right_dis_start) / 1.414
        right_ex = self.right_x6 - (self.right_strings[string - 1] + right_dis_end) / 1.414

        right_sy = self.right_y6 + (self.right_strings[string - 1] - right_dis_start) / 1.414
        right_ey = self.right_y6 + (self.right_strings[string - 1] + right_dis_end) / 1.414

        if left_grade > 0:
            right_xs = [right_sx, right_sx]
            right_ys = [right_sy, right_sy]
            right_zs = [self.right_z_free, self.right_z[string - 1]]

            self.right_arm.go_to_pose(right_sx, right_sy, self.right_z_free)
            left_x, left_y, left_z = self.left_arm.get_pose()
            self.left_arm.go_to_pose(left_x, left_y, self.left_z_free)

            left_xs = [left_x]
            left_ys = [left_y]
            left_zs = [left_z]

            left_x = self.left_x6 + ((self.left_grades[left_grade - 1] - 0.005) / 1.4142) - (
                    self.left_strings[string - 1] / 1.4142)
            left_y = self.left_y6 + ((self.left_grades[left_grade - 1] - 0.005) / 1.4142) + (
                    self.left_strings[string - 1] / 1.4142)

            self.left_arm.go_to_pose(left_x, left_y, self.left_z_free)

            self.right_arm.go_to_pose(right_sx, right_sy, self.right_z[string - 1])
            self.left_arm.go_to_pose(left_x, left_y, 0.024)
            self.right_arm.go_to_pose(right_ex, right_ey, self.right_z[string - 1])
            self.right_arm.go_to_pose(right_ex, right_ey, self.right_z_free)
            # self.left_arm.go_to_pose(left_x, left_y, self.left_z_free)
        else:
            self.right_arm.go_to_pose(right_sx, right_sy, self.right_z_free)
            left_x, left_y, left_z = self.left_arm.get_pose()
            self.left_arm.go_to_pose(left_x, left_y, self.left_z_free)
            self.right_arm.go_to_pose(right_sx, right_sy, self.right_z[string - 1])
            self.right_arm.go_to_pose(right_ex, right_ey, self.right_z[string - 1])
            self.right_arm.go_to_pose(right_ex, right_ey, self.right_z_free)

    def test_right_arm(self):
        self.play(6, 1)
        for i in range(1, 7):
            self.play(i, 0)

    def play_star(self):
        self.play(string=5, left_grade=3)
        self.play(string=5, left_grade=3)
        self.play(string=3, left_grade=0)
        self.play(string=3, left_grade=0)
        self.play(string=3, left_grade=2)
        self.play(string=3, left_grade=2)
        self.play(string=3, left_grade=0)

        self.play(string=4, left_grade=3)
        self.play(string=4, left_grade=3)
        self.play(string=4, left_grade=2)
        self.play(string=4, left_grade=2)
        self.play(string=4, left_grade=0)
        self.play(string=4, left_grade=0)
        self.play(string=5, left_grade=3)

    def wait_for_action_end_or_abort(self):

        while not rospy.is_shutdown():
            if self.left_arm.last_action_notif_type == ActionEvent.ACTION_END and self.right_arm.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("ALL ACTION_END")
                return True, True
            elif self.left_arm.last_action_notif_type == ActionEvent.ACTION_END and not self.right_arm.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("LEFT ACTION_END")
                return True, False
            elif not self.left_arm.last_action_notif_type == ActionEvent.ACTION_END and self.right_arm.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("RIGHT ACTION_END")
                return False, True
            elif self.left_arm.last_action_notif_type == ActionEvent.ACTION_ABORT or self.right_arm.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.logerr("ACTION_ABORT")
                return False, False
            else:
                time.sleep(0.01)

    def main(self):
        # For testing purposes
        success = True
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass

        if success:
            # *******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.right_arm.example_clear_faults()
            success &= self.right_arm.example_subscribe_to_a_robot_notification()
            success &= self.right_arm.example_set_cartesian_reference_frame()

            success &= self.left_arm.example_clear_faults()
            success &= self.left_arm.example_subscribe_to_a_robot_notification()
            success &= self.left_arm.example_set_cartesian_reference_frame()
            self.right_arm.get_pose()
            self.left_arm.get_pose()
            self.right_arm.go_to_pose(x=0.292414, y=-0.137692, z=self.right_z_free, theta_x=180, theta_y=0,
                                      theta_z=135, theta_change=True)
            self.left_arm.go_to_pose(0.373, -0.032, 0.04, theta_x=0, theta_y=180,
                                     theta_z=45, theta_change=True)
            self.left_arm.wait_for_action_end_or_abort()
            self.right_arm.wait_for_action_end_or_abort()

            self.init_start()
            # self.test_right_arm()
            # self.play_star()
            # self.test_right_arm()
            # for i in range(6):

            # self.finished()

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    example = Main()
    example.main()
