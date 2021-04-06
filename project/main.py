from __future__ import print_function

import math
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

    def get_left_pose(self, pose):
        string = pose[0]
        left_grade = pose[1]
        x = self.left_x6 + ((self.left_grades[left_grade - 1] - 0.005) / 1.4142) - (
                self.left_strings[string - 1] / 1.4142)
        y = self.left_y6 + ((self.left_grades[left_grade - 1] - 0.005) / 1.4142) + (
                self.left_strings[string - 1] / 1.4142)
        return x, y

    def get_right_pose(self, pose):
        right_dis_start = 0.003

        cos135 = -1.414
        sin135 = 1.414

        string = int(round(pose))
        k = -1 if pose > string else 1
        x = self.right_x6 + (self.right_strings[string - 1] + k * right_dis_start) / cos135
        y = self.right_y6 + (self.right_strings[string - 1] + k * right_dis_start) / sin135
        z = self.right_z[string - 1]
        return x, y, z

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
        right_x, right_y, right_z_down = 0, 0, 0
        left_x, left_y = 0, 0
        while True:
            if left_res:
                print(f"LEFT Action: {action_left}")
                if action_left == LEFT_MOVE:
                    pose_left = (strings[frame_left], grades[frame_left])
                    if pose_left[1] > 0:
                        left_x, left_y = self.get_left_pose(pose_left)
                        self.left_arm.go_to_pose(left_x, left_y, self.left_z_free)  # todo
                        action_left = LEFT_PRESS
                    else:
                        frame_left += 1

                if action_left == LEFT_PRESS:
                    self.left_arm.go_to_pose(left_x, left_y, self.left_z_pressed)
                    action_left = LEFT_UP

                if action_left == LEFT_UP:
                    if pose_left[1] == grades[frame_left + 1] and math.ceil(strings[frame_left + 1] / 2) == math.ceil(
                            pose_left[0] / 2):
                        frame_left += 1
                    elif frame_left == frame_right and action_right == RIGHT_UP:
                        self.left_arm.go_to_pose(left_x, left_y, self.left_z_free)
                        frame_left += 1
                        action_left = LEFT_MOVE

            if right_res:
                print(f"Right Action: {action_right}")
                if action_right == RIGHT_MOVE:
                    right_string = strings[frame_left]
                    if pose_right >= right_string:
                        pose_right = right_string + 0.1
                        right_x, right_x, right_z_down = self.get_right_pose(pose_right)
                    else:
                        pose_right = right_string - 0.1
                        right_x, right_x, right_z_down = self.get_right_pose(pose_right)
                    self.right_arm.go_to_pose(right_x, right_y, self.right_z_free)
                    action_right = RIGHT_BEFORE_PLAY

                if action_right == RIGHT_BEFORE_PLAY:
                    self.right_arm.go_to_pose(right_x, right_y, right_z_down)
                    action_right = RIGHT_AFTER_PLAY

                if action_right == RIGHT_AFTER_PLAY:
                    if grades[frame_left] == 0 or (action_left == LEFT_UP and frame_left == frame_right):
                        right_string = strings[frame_left]
                        if pose_right >= right_string:
                            pose_right = right_string - 0.1
                            right_x, right_y, _ = self.get_right_pose(pose_right)
                        else:
                            pose_right = right_string + 0.1
                            right_x, right_y, _ = self.get_right_pose(pose_right)
                        self.right_arm.go_to_pose(right_x, right_y, right_z_down)
                        action_right = RIGHT_UP

                if action_right == RIGHT_UP:
                    self.right_arm.go_to_pose(right_x, right_y, self.right_z_free)
                    action_right = RIGHT_MOVE

            left_res, right_res = self.wait_for_action_end_or_abort()

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
