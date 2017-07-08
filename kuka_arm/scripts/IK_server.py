#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

global px
global py
global pz

global roll
global pitch
global yaw

global t0_1
global t1_2
global t2_3

global q1
global q2
global q3
global q4
global q5
global q6

'''
Initialize link Lengths
'''


def init_link_lengths():
    link2_3 = 1.25
    link3_5 = sqrt(1.5 ** 2 + 0.054 ** 2)
    return link2_3, link3_5


'''
Initialize joint positions
'''


def init_joint_positions():
    joint2 = [0.35, 0, 0.75]
    joint3 = [0.35, 0, 2]
    joint5 = [1.85, 0, 1.946]
    return joint2, joint3, joint5


'''
Initialize θ_3(Theta3) which is the angle rotation around the z-axis.
'''


def init_q3():
    return pi / 2 + atan2(-0.054, 1.5)


def dh_transformation(alpha, a, q, d):
    return Matrix([[cos(q), -sin(q), 0, a],
                   [sin(q) * cos(alpha), cos(q) * cos(a), -sin(alpha), -sin(q) * d],
                   [sin(q) * sin(alpha), cos(q) * sin(a), cos(alpha), cos(q) * d],
                   [0, 0, 0, 1]])


def correction_matrix():
    # 90 degrees on the z-axis
    r_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                  [sin(pi), cos(pi), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    # 90 degrees on the y-axis
    r_y = Matrix([[cos(-pi / 2), 0, sin(-pi / 2), 0],
                  [0, 1, 0, 0],
                  [-sin(-pi / 2), 0, cos(-pi / 2), 0],
                  [0, 0, 0, 1]])
    return simplify(r_z * r_y)


def get_wrist_center(end_effector_points, end_effector_roll_pitch_yaw):
    px, py, pz = end_effector_points
    roll, pitch, yaw = end_effector_roll_pitch_yaw
    return


def do_rotation(axis, angle):
    if axis == "x":
        return Matrix([[1, 0, 0],
                       [0, cos(angle), -sin(angle)],
                       [0, sin(angle), cos(angle)]])

    if axis == "y":
        return Matrix([[cos(angle), 0, sin(angle)],
                       [0, 1, 0],
                       [-sin(angle), 0, cos(angle)]])
    if axis == 'z':
        return Matrix([[cos(angle), -sin(angle), 0],
                       [sin(angle), cos(angle), 0],
                       [0, 0, 1]])
    else:
        return None


def calc_inverse_Kinematics():
    joint2, joint3, joint5 = init_joint_positions();
    link2_3, link3_5 = init_link_lengths();
    q3 = init_q3()

    wrist_center, r_0_6 = f


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print
        "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            # Joint angles
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
            # Twist angles
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
            # Link lengths
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            # link offsets
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

            # Denavit–Hartenberg Parameters Symbols Dictionary
            s = {alpha0: 0, a0: 0, d1: 0.75,
                 alpha1: -np.pi / 2, a1: 0.35, d2: 0, q2: q2 - np.pi / 2,
                 alpha2: 0, a2: 1.25, d3: 0,
                 alpha3: -np.pi / 2, a3: -0.054, d4: 1.5,
                 alpha4: np.pi / 2, a4: 0, d5: 0,
                 alpha5: -np.pi / 2, a5: 0, d6: 0,
                 alpha6: 0, a6: 0, d7: 0.303, q7: 0}

            # Modified DH params



            # Define Modified DH Transformation matrix



            # Create individual transformation matrices

            # Between link 0 to 1
            t0_1 = dh_transformation(alpha0, a0, q1, d1).subs(s)
            # Between Link 1 to 2
            t1_2 = dh_transformation(alpha1, a1, q2, d2).subs(s)
            # Between Link 2 to 3
            t2_3 = dh_transformation(alpha2, a2, q3, d3).subs(s)
            # Between link 3 to 4
            t3_4 = dh_transformation(alpha3, a3, q4, d4).subs(s)
            # Between link 4 to 5
            t4_5 = dh_transformation(alpha4, a4, q5, d5).subs(s)
            # Between link 5 to 6
            t5_6 = dh_transformation(alpha5, a5, q6, d6).subs(s)
            # Between link 6 to 7
            t6_7 = dh_transformation(alpha6, a6, q7, d7).subs(s)
            # Symplify
            t0_7 = simplify(t0_1 * t1_2 * t2_3 * t3_4 * t4_5 * t5_6 * t6_7)

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method




            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [q1, q2, q3, q4, q5, q6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print
    "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
