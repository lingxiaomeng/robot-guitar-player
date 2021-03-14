#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###
from __future__ import print_function

import sys
import rospy
import time
import math
from kortex_driver.srv import *
from kortex_driver.msg import *


class ExampleFullArmMovement:
    def __init__(self):
        try:
            rospy.init_node('example_full_arm_movement_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(
                self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(
                self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification,
                                                     self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name,
                                                                    SetCartesianReferenceFrame)

            play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
            rospy.wait_for_service(play_cartesian_trajectory_full_name)
            self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name,
                                                                PlayCartesianTrajectory)

            play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
            rospy.wait_for_service(play_joint_trajectory_full_name)
            self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(
                activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
            self.x6 = 0
            self.y6 = 0
            self.dx = 0
            self.dy = 0

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        self.last_action_notif_type = None
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)
        return True

    def example_send_cartesian_pose(self, x=0.0, y=0.0, z=0.0):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = feedback.base.commanded_tool_pose_x + x
        req.input.target_pose.y = feedback.base.commanded_tool_pose_y + y
        req.input.target_pose.z = feedback.base.commanded_tool_pose_z + z
        req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
        req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
        req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

        # req.input.target_pose.theta_x = 0.001
        # req.input.target_pose.theta_y = -0.007
        # req.input.target_pose.theta_z = 0.0001

        pose_speed = CartesianSpeed()
        pose_speed.translation = 0.1
        pose_speed.orientation = 15

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object :
        req.input.constraint.oneof_type.speed.append(pose_speed)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_joint_angles(self):
        self.last_action_notif_type = None
        # Create the list of angles
        req = PlayJointTrajectoryRequest()
        # Here the arm is vertical (all zeros)
        angles = [313, 295, 62, 85, 50, 112, 100]
        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle()
            temp_angle.joint_identifier = i
            temp_angle.value = 0.0
            req.input.joint_angles.joint_angles.append(temp_angle)

        # Send the angles
        rospy.loginfo("Sending the robot vertical...")

        try:
            self.play_joint_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayJointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def get_pose(self):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        print("x %f" % feedback.base.commanded_tool_pose_x)
        print("y %f" % feedback.base.commanded_tool_pose_y)
        print("z %f" % feedback.base.commanded_tool_pose_z)

        print("theta x %f" % feedback.base.commanded_tool_pose_theta_x)
        print("theta y %f" % feedback.base.commanded_tool_pose_theta_y)
        print("theta z %f" % feedback.base.commanded_tool_pose_theta_z)

    def go_to_pose(self, x, y, z):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
        req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
        req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

        req.input.target_pose.x = x
        req.input.target_pose.y = y
        req.input.target_pose.z = z

        pose_speed = CartesianSpeed()
        pose_speed.translation = 0.1
        pose_speed.orientation = 15

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object :
        req.input.constraint.oneof_type.speed.append(pose_speed)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def init_start(self):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.x6 = feedback.base.commanded_tool_pose_x
        self.y6 = feedback.base.commanded_tool_pose_y
        self.z_free = 0.035

    def play6(self):
        sx = -0.0055 / 1.4142
        sy = -0.0055 / 1.4142
        ex = 0.0055 / 1.4142
        ey = 0.0055 / 1.4142
        self.go_to_pose(self.x6 - sx, self.y6 + sy, self.z_free)
        self.go_to_pose(self.x6 - sx, self.y6 + sy, 0.0322)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, 0.0322)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, self.z_free)

    def play5(self):
        sx = 0.0055 / 1.4142
        sy = 0.0055 / 1.4142
        ex = 0.0165 / 1.4142
        ey = 0.0165 / 1.4142
        self.go_to_pose(self.x6 - sx, self.y6 + sy, self.z_free)
        self.go_to_pose(self.x6 - sx, self.y6 + sy, 0.0322)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, 0.0322)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, self.z_free)

    def play4(self):
        sx = 0.0165 / 1.4142
        sy = 0.0165 / 1.4142
        ex = 0.0268 / 1.4142
        ey = 0.0268 / 1.4142
        self.go_to_pose(self.x6 - sx, self.y6 + sy, self.z_free)
        self.go_to_pose(self.x6 - sx, self.y6 + sy, 0.0322)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, 0.0322)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, self.z_free)

    def play3(self):
        sx = 0.0268 / 1.4142
        sy = 0.0268 / 1.4142
        ex = 0.0371 / 1.4142
        ey = 0.0371 / 1.4142
        self.go_to_pose(self.x6 - sx, self.y6 + sy, self.z_free)
        self.go_to_pose(self.x6 - sx, self.y6 + sy, 0.0322)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, 0.0322)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, self.z_free)

    def play2(self):
        sx = 0.0371 / 1.4142
        sy = 0.0371 / 1.4142
        ex = 0.0474 / 1.4142
        ey = 0.0474 / 1.4142
        self.go_to_pose(self.x6 - sx, self.y6 + sy, self.z_free)
        self.go_to_pose(self.x6 - sx, self.y6 + sy, 0.0322)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, 0.0322)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, self.z_free)

    def play1(self):
        sx = 0.0474 / 1.4142
        sy = 0.0474 / 1.4142
        ex = 0.0577 / 1.4142
        ey = 0.0577 / 1.4142
        self.go_to_pose(self.x6 - sx, self.y6 + sy, self.z_free)
        self.go_to_pose(self.x6 - sx, self.y6 + sy, 0.0315)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, 0.0315)
        self.go_to_pose(self.x6 - ex, self.y6 + ey, self.z_free)

    def get_effort(self):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        print("torque %f" % feedback.actuators[0].torque)
        print("torque %f" % feedback.actuators[1].torque)
        print("torque %f" % feedback.actuators[2].torque)
        print("torque %f" % feedback.actuators[3].torque)
        print("torque %f" % feedback.actuators[4].torque)
        print("torque %f" % feedback.actuators[5].torque)


    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass

        if success:
            # *******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            # *******************************************************************************

            # *******************************************************************************
            # Activate the action notifications
            success &= self.example_subscribe_to_a_robot_notification()
            # *******************************************************************************
            self.get_pose()
            # *******************************************************************************
            # Move the robot to the Home position with an Action
            # success &= self.example_home_the_robot()
            # *******************************************************************************

            # *******************************************************************************
            # Example of gripper command
            # Let's fully open the gripper
            # if self.is_gripper_present:
            #     success &= self.example_send_gripper_command(0.0)
            # else:
            #     rospy.logwarn("No gripper is present on the arm.")
            # *******************************************************************************

            # *******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            # self.go_to_pose(0.338730, -0.092441, 0.035)

            self.init_start()



            # Example of cartesian pose
            # Let's make it move in Z
            # success &= self.example_send_cartesian_pose()

            # success &= self.example_send_cartesian_pose(0.00757, -0.00757, 0)
            # success &= self.example_send_cartesian_pose(0.00, 0.00, -0.001)
            # success &= self.example_send_cartesian_pose(0, 0.05, 0)
            # success &= self.example_send_cartesian_pose(-0.05, 0, 0)
            # success &= self.example_send_cartesian_pose(0, -0.05, 0)

            # *******************************************************************************

            # *******************************************************************************
            # Example of angular position
            # Let's send the arm to vertical position
            # success &= self.example_send_joint_angles()
            # *******************************************************************************
            self.get_pose()
            self.get_effort()
            # *******************************************************************************
            # Example of gripper command
            # Let's close the gripper at 50%
            # if self.is_gripper_present:
            #     success &= self.example_send_gripper_command(0.5)
            # else:
            #     rospy.logwarn("No gripper is present on the arm.")
            # *******************************************************************************

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    example = ExampleFullArmMovement()
    example.main()
