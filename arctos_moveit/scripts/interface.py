#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        # ROS initialization
        rospy.init_node('ros_node')
        rospy.Subscriber("ui_command", String, self.ui_command_callback)

        # MoveIt initialization
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

    def ui_command_callback(self, data):
        command = data.data.split(',')
        if command[0] == "go_to_joint_state":
            joint_state_values = [float(value) for value in command[1:]]
            self.go_to_joint_state(joint_state_values)

        elif command[0] == "plan_cartesian_path":
            cartesian_path_values = [float(value) for value in command[1:]]
            self.plan_cartesian_path(cartesian_path_values)

        elif command[0] == "open_gripper":
            self.open_gripper()

        elif command[0] == "close_gripper":
            self.close_gripper()





        else:
        # Unknown command
            rospy.logwarn("Unknown command received: %s", command)
    def go_to_joint_state(self, joint_state_values):
        self.group.go(joint_state_values, wait=True)
        self.group.stop()


    def plan_cartesian_path(self, cartesian_path_values):
        waypoints = []
        for i in range(0, len(cartesian_path_values), 3):
            pose = geometry_msgs.msg.Pose()
            pose.position.x, pose.position.y, pose.position.z = cartesian_path_values[i:i+3]
            waypoints.append(copy.deepcopy(pose))

        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.group.execute(plan, wait=True)


    def open_gripper(self):
        gripper_values = [0.0, 0.0]  # Predefined values for opening the gripper
        self.gripper_group.go(gripper_values, wait=True)
        self.gripper_group.stop()
        rospy.loginfo("Gripper opened")

    def close_gripper(self):
        gripper_values = [0.015, 0.015]  # Predefined values for closing the gripper
        self.gripper_group.go(gripper_values, wait=True)
        self.gripper_group.stop()
        rospy.loginfo("Gripper closed")







def main():
    try:
        tutorial = MoveGroupPythonIntefaceTutorial()
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()



