from __future__ import print_function

import math
import time

from kortex_driver.msg import BaseCyclic_Feedback, ActionEvent

from robot_api import Robot_Api

import rospy
from keyboard_state import *


class Main:
    def __init__(self):
        self.left_arm = Robot_Api("my_left_arm")
        self.right_arm = Robot_Api("my_right_arm")
        self.right_x6 = 0.292414
        self.right_y6 = -0.137629

        self.left_x6 = 0.373
        self.left_y6 = -0.032

        self.start_time = 0.0
        self.end_time = 0.0

        self.string_heights = [0.0113, 0.0124, 0.013, 0.01330, 0.01317, 0.01296]
        for i in range(len(self.string_heights)):
            self.string_heights[i] = self.string_heights[i] - self.string_heights[5] + 0.0439
        self.right_z = self.string_heights
        # print(self.right_z)
        # [0.0428, 0.0432, 0.044, 0.0443, 0.0443, 0.0439]

        self.left_grades = [0.03635, 0.03635 + 0.03431, 0.03635 + 0.03431 + 0.03239]
        self.left_strings = [0.028666, 0.028666, 0.014333, 0.014333, 0, 0]

        dis_string = 0.05292 / 5
        self.right_strings = [dis_string * 5, dis_string * 4, dis_string * 3, dis_string * 2, dis_string * 1, 0]
        self.left_z_pressed = 0.014
        self.left_z_free = 0.024
        self.right_z_free = 0.048

    def init_start(self):
        feedback = rospy.wait_for_message("/" + self.right_arm.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.right_x6 = feedback.base.commanded_tool_pose_x
        self.right_y6 = feedback.base.commanded_tool_pose_y

        feedback = rospy.wait_for_message("/" + self.left_arm.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.left_x6 = feedback.base.commanded_tool_pose_x
        self.left_y6 = feedback.base.commanded_tool_pose_y

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

    def get_right_pose(self, pose, grade):
        right_dis_start = 0.004

        cos135 = -1.414
        sin135 = 1.414

        string = int(round(pose))
        k = -1 if pose > string else 1
        x = self.right_x6 + ((self.right_strings[string - 1] + k * right_dis_start) / cos135)
        y = self.right_y6 + ((self.right_strings[string - 1] + k * right_dis_start) / sin135)
        z = self.right_z[string - 1] - 0.001 / 3 * grade
        print(f"right pose string:{string} x:{x} y:{y} z:{z}")
        return x, y, z

    def play(self, left_key, right_key, beats):
        frame_len = len(left_key)

        assert len(left_key) == len(right_key) == len(beats)
        left_key.append(0)
        left_key.append(0)
        right_key.append(0)
        right_key.append(0)
        beats.append(1)
        beats.append(1)
        # state_left = 0
        # state_right = 0
        frame_left = 0
        frame_right = 0
        action_left = LEFT_MOVE
        action_right = RIGHT_MOVE
        left_res = True
        right_res = True
        pose_left = 0
        pose_right = 0
        right_x, right_y, right_z_down = 0, 0, 0
        left_x, left_y = 0, 0
        left_ready = [False] * (frame_len + 2)
        right_ready = [False] * (frame_len + 2)
        timewait = [False] * (frame_len + 2)
        start_time = time.time()
        # right_ready[-1] = True
        while True:
            if frame_left >= frame_len and frame_right >= frame_len:
                self.left_arm.wait_for_action_end_or_abort()
                self.right_arm.wait_for_action_end_or_abort()
                break
            if left_res:
                # print(f"LEFT frame:{frame_left} Action: {action_left}")
                # print(f"Right frame {frame_right} Action: {action_right}")
                if action_left == LEFT_MOVE:
                    pose_left = left_key[frame_left]
                    if pose_left[1] > 0:
                        left_x, left_y = self.get_left_pose(pose_left)
                        self.left_arm.go_to_pose(left_x, left_y, self.left_z_free)  # todo
                        action_left = LEFT_PRESS
                    else:
                        if frame_left < frame_len:
                            left_ready[frame_left] = True
                            frame_left += 1

                elif action_left == LEFT_PRESS:
                    left_ready[frame_left] = True
                    if right_ready[frame_left]:
                        if not timewait[frame_left]:
                            et = time.time()
                            dt = et - start_time
                            print(f"frame {frame_right - 1}: time: {dt}")
                            frame_time = 1.2 * beats[frame_right - 1]
                            if frame_time - dt > 0:
                                time.sleep(frame_time - dt)
                            start_time = time.time()
                            timewait[frame_left] = True
                        self.left_arm.go_to_pose(left_x, left_y, self.left_z_pressed)
                        action_left = LEFT_UP

                elif action_left == LEFT_UP:
                    self.left_arm.go_to_pose(left_x, left_y, self.left_z_free)
                    frame_left += 1
                    action_left = LEFT_MOVE

            if right_res:
                # print(f"right frame:{frame_right} Action: {action_right}")
                # print(f"left frame {frame_left} Action: {action_left}")
                if action_right == RIGHT_MOVE:
                    pose_right = right_key[frame_right]
                    if pose_right[1] > 0:
                        right_x, right_y = self.get_right_pose(pose_right)
                        self.right_arm.go_to_pose(right_x, right_y, self.right_z_free)  # todo
                        action_right = RIGHT_PRESS
                    else:
                        if frame_right < frame_len:
                            right_ready[frame_right] = True
                            frame_right += 1

                elif action_right == RIGHT_PRESS:
                    right_ready[frame_right] = True
                    if left_ready[frame_right]:
                        if not timewait[frame_right]:
                            et = time.time()
                            dt = et - start_time
                            print(f"frame {frame_left - 1}: time: {dt}")
                            frame_time = 1.2 * beats[frame_left - 1]
                            if frame_time - dt > 0:
                                time.sleep(frame_time - dt)
                            start_time = time.time()
                            timewait[frame_right] = True
                        self.right_arm.go_to_pose(right_x, right_y, self.right_z_pressed)
                        action_right = RIGHT_UP

                elif action_right == RIGHT_UP:
                    self.right_arm.go_to_pose(right_x, right_y, self.right_z_free)
                    frame_right += 1
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
                time.sleep(0.01)

    def test_right_arm(self):
        strings = [6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6]
        grades = [3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
        beats = [1] * len(strings)
        self.play(strings, grades, beats)

    def play_star(self):
        strings = [5, 5, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5]
        grades = [3, 3, 0, 0, 2, 2, 0, 3, 3, 2, 2, 0, 0, 3]
        beats = [1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2]
        self.play(strings, grades, beats)

    def play_secret_base(self):
        strings = [3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3]
        grades_ = [2, 3, 0, 0, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2]
        beats__ = [1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 2]
        self.play(strings, grades_, beats__)

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
            # self.right_arm.get_pose()
            # self.left_arm.get_pose()
            self.right_arm.go_to_pose(x=0.292414, y=-0.137692, z=self.right_z_free, theta_x=180, theta_y=0,
                                      theta_z=135, theta_change=True)
            self.left_arm.go_to_pose(0.373, -0.032, 0.045, theta_x=0, theta_y=180,
                                     theta_z=45, theta_change=True)
            # lx, ly = self.get_left_pose((6, 1))
            # self.left_arm.go_to_pose(lx, ly, self.left_z_free, theta_x=0, theta_y=180,
            #                          theta_z=45, theta_change=True)

            self.left_arm.wait_for_action_end_or_abort()
            self.right_arm.wait_for_action_end_or_abort()

            self.init_start()
            # self.test_right_arm()
            self.play_secret_base()
            # self.play_test()

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
