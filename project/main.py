from __future__ import print_function

import time

from kortex_driver.msg import BaseCyclic_Feedback, ActionEvent

from robot_api import Robot_Api

import rospy
from state import *


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

        self.start_time = 0.0
        self.end_time = 0.0

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
        self.left_z_pressed = 0.022

    def init_start(self):
        feedback = rospy.wait_for_message("/" + self.right_arm.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.right_x6 = feedback.base.commanded_tool_pose_x
        self.right_y6 = feedback.base.commanded_tool_pose_y
        self.right_z_free = 0.052

        feedback = rospy.wait_for_message("/" + self.left_arm.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.left_x6 = feedback.base.commanded_tool_pose_x
        self.left_y6 = feedback.base.commanded_tool_pose_y
        self.left_z_free = 0.035
        self.left_z_pressed = 0.026

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
        while left_idx < left_len or right_idx < right_len:
            if left_state == 0 and left_idx < left_len:
                self.left_arm.go_to_pose(left_x[left_idx], left_y[left_idx], left_z[left_idx])
                left_state = 1
            if right_state == 0 and right_idx < right_len:
                self.right_arm.go_to_pose(right_x[right_idx], right_y[right_idx], right_z[right_idx])
                right_state = 1

            left_res, right_res = self.wait_for_action_end_or_abort()
            if left_res and left_state == 1:
                left_state = 0
                left_idx += 1
                rospy.loginfo("LEFT ACTION END")
                if left_idx >= left_len:
                    left_state = 2
                    rospy.loginfo("LEFT ARM FINISHED")
            if right_res and right_state == 1:
                right_state = 0
                right_idx += 1
                rospy.loginfo("RIGHT ACTION END")
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

            right_delta_z = 0.0000
            right_xs = [right_sx, right_sx]
            right_ys = [right_sy, right_sy]
            right_zs = [self.right_z_free, self.right_z[string - 1] - right_delta_z]

            # self.right_arm.go_to_pose(right_sx, right_sy, self.right_z_free)
            left_x, left_y, _ = self.left_arm.get_pose()
            # self.left_arm.go_to_pose(left_x, left_y, self.left_z_free)

            left_x1 = self.left_x6 + ((self.left_grades[left_grade - 1] - 0.005) / 1.4142) - (
                    self.left_strings[string - 1] / 1.4142)
            left_y1 = self.left_y6 + ((self.left_grades[left_grade - 1] - 0.005) / 1.4142) + (
                    self.left_strings[string - 1] / 1.4142)

            left_xs = [left_x, left_x1, left_x1]
            left_ys = [left_y, left_y1, left_y1]
            left_zs = [self.left_z_free, self.left_z_free, self.left_z_pressed]

            self.go_to_pose(left_xs, left_ys, left_zs, right_xs, right_ys, right_zs)

            right_xs = [right_ex, right_ex]
            right_ys = [right_ey, right_ey]
            right_zs = [self.right_z[string - 1] - right_delta_z, self.right_z_free]
            self.go_to_pose([], [], [], right_xs, right_ys, right_zs)
            delta_time = (time.time() - self.start_time)
            rospy.logwarn("TIME:%f" % delta_time)
            rospy.sleep(2.2 - delta_time)
            self.start_time = time.time()
        else:
            right_xs = [right_sx, right_sx, right_ex, right_ex]
            right_ys = [right_sy, right_sy, right_ey, right_ey]
            right_zs = [self.right_z_free, self.right_z[string - 1], self.right_z[string - 1], self.right_z_free]

            left_x, left_y, left_z = self.left_arm.get_pose()
            self.go_to_pose([left_x], [left_y], [self.left_z_free], right_xs, right_ys, right_zs)
            delta_time = (time.time() - self.start_time)
            rospy.logwarn("TIME:%f" % delta_time)
            rospy.sleep(2.2 - delta_time)
            self.start_time = time.time()

    def get_left_pose(self):
        pass

    def get_right_pose(self, pose):
        right_dis_start = 0.003

        cos135 = -1.414
        sin135 = 1.414

        string = int(round(pose))
        k = -1 if pose > string else 1
        x = self.right_x6 + (self.right_strings[string - 1] + k * right_dis_start) / cos135
        y = self.right_y6 + (self.right_strings[string - 1] + k * right_dis_start) / sin135
        return x, y

    def play_test(self):
        strings = [5, 5, 3, 3, 3, 3, 3, 3]
        grades = [3, 3, 0, 0, 2, 2, 0]
        assert len(strings) != len(strings)
        # state_left = 0
        # state_right = 0
        frame_left = 1
        frame_right = 1
        action_left = LEFT_UP
        action_right = 0
        left_res = True
        right_res = True
        pose_left = (0, 0)  # 0 string 1 grade
        pose_right = 6
        right_x, right_y = 0, 0
        left_x, left_y = 0, 0
        while True:
            if left_res:
                if action_left == LEFT_MOVE:
                    # if frame_left == frame_right:
                    pose_left = (strings[frame_left], grades[frame_left])
                    if pose_left[1] > 0:
                        self.left_arm.go_to_pose(0, 0, 0)  # todo
                        action_left = LEFT_PRESS

                if action_left == LEFT_PRESS:
                    if pose_left[1] == grades[frame_left + 1] and abs(strings[frame_left + 1] - pose_left[0]) < 1:
                        frame_left += 1
                    else:
                        self.left_arm.go_to_pose(0, 0, 0)
                        frame_left += 1
                        action_left = LEFT_UP

                if action_left == LEFT_UP:
                    if grades[frame_left] == 0:
                        frame_left += 1
                    else:
                        if frame_right == frame_left and action_right == RIGHT_UP:
                            self.left_arm.go_to_pose(0, 0, 0)  # TODO
                            action_left = LEFT_MOVE
                            pose_left = (0, 0)  # TODO

            if right_res:
                if action_right == RIGHT_MOVE:
                    right_string = strings[frame_left]
                    if pose_right >= right_string:
                        pose_right = right_string + 0.1
                        right_x, right_x = self.get_right_pose(pose_right)
                    else:
                        pose_right = right_string - 0.1
                        right_x, right_x = self.get_right_pose(pose_right)
                    self.right_arm.go_to_pose(right_x, right_y, self.right_z_free)
                    action_right = RIGHT_BEFORE_PLAY

                if action_right == RIGHT_BEFORE_PLAY:
                    self.right_arm.go_to_pose(right_x, right_y, self.right_z)
                    action_right = RIGHT_AFTER_PLAY

                if action_right == RIGHT_AFTER_PLAY:
                    if grades[frame_left] == 0 or (action_left == LEFT_UP and frame_left == frame_right):
                        right_string = strings[frame_left]
                        if pose_right >= right_string:
                            pose_right = right_string - 0.1
                            right_x, right_y = self.get_right_pose(pose_right)
                        else:
                            pose_right = right_string + 0.1
                            right_x, right_y = self.get_right_pose(pose_right)
                        self.right_arm.go_to_pose(right_x, right_y, self.right_z)
                        action_right = RIGHT_UP

                if action_right == RIGHT_UP:
                    self.right_arm.go_to_pose(right_x, right_y, self.right_z_free)
                    action_right = RIGHT_MOVE

            left_res, right_res = self.wait_for_action_end_or_abort()

    def test_right_arm(self):
        self.play(6, 1)
        for i in range(1, 7):
            self.play(i, 3)

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
            # print("left:%s, right:%s" % (
            #     str(self.left_arm.last_action_notif_type), str(self.right_arm.last_action_notif_type)))
            if self.left_arm.last_action_notif_type == ActionEvent.ACTION_END and self.right_arm.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("ALL ACTION_END")
                return True, True
            elif self.left_arm.last_action_notif_type == ActionEvent.ACTION_END and self.right_arm.last_action_notif_type != ActionEvent.ACTION_END:
                # rospy.loginfo("LEFT ACTION_END")
                return True, False
            elif self.left_arm.last_action_notif_type != ActionEvent.ACTION_END and self.right_arm.last_action_notif_type == ActionEvent.ACTION_END:
                # rospy.loginfo("RIGHT ACTION_END")
                return False, True
            elif self.left_arm.last_action_notif_type == ActionEvent.ACTION_ABORT or self.right_arm.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.logerr("ACTION_ABORT")
                return False, False
            else:
                time.sleep(0.001)

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
            self.left_arm.go_to_pose(0.373, -0.032, 0.045, theta_x=0, theta_y=180,
                                     theta_z=45, theta_change=True)
            self.left_arm.wait_for_action_end_or_abort()
            self.right_arm.wait_for_action_end_or_abort()

            self.init_start()

            # self.play(string=5, left_grade=1)
            # self.play_star()
            # self.test_right_arm()
            self.play_star()
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
