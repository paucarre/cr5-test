#!/usr/bin/env python 
from __future__ import print_function
from six.moves import input

import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import numpy as np
from math import pi, dist, fabs, cos, sqrt


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class MoveItRunner(object):

    def __init__(self):
        super(MoveItRunner, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_runner", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "cr5_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_planning_time(1000.0)
        move_group.clear_path_constraints()
        move_group.clear_pose_target("Link6")
        move_group.set_planner_id("ompl_interface/OMPLPlanner")

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

     
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def plan_cartesian_path(self, waypoints):
        move_group = self.move_group
        #waypoints_safe = [copy.deepcopy(waypoint) for waypoint in waypoints]
        #waypoints.append()        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 
            100.01, # - eef_step: step size of at most eef_step meters between end
                  # effector configurations of consecutive points in the result trajectory.
            100.0   # - jump_threshold: No more than jump_threshold is allowed as change in distance
                  # in the configuration space of the robot (this is to prevent 'jumps' in IK solutions)
        )
        return plan, fraction

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

def main():
    try:
        moveit_runner = MoveItRunner()
        pose = moveit_runner.move_group.get_current_pose().pose
        waypoints = []
        pose.position.z = 0.7
        plan, fraction = moveit_runner.plan_cartesian_path([pose])
        print(f"Fraction: {fraction}")
        if fraction == 1.0:
            moveit_runner.execute_plan(plan)
        pose = copy.deepcopy(pose)
        pose.position.x = 0.4
        plan, fraction = moveit_runner.plan_cartesian_path([pose])
        print(f"Fraction: {fraction}")
        moveit_runner.execute_plan(plan)
        radius = 0.4
        for angle in np.arange(0.0, math.pi, 0.2):
            current_pose = copy.deepcopy(pose)
            current_pose.position.x = radius * math.cos(angle)
            current_pose.position.y = radius * math.sin(angle)
            waypoints.append(current_pose)
        plan, fraction = moveit_runner.plan_cartesian_path(waypoints)
        print(f"Fraction: {fraction}")
        moveit_runner.execute_plan(plan)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
