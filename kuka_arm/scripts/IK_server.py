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


def calculate_rotation_for_roll_pitch_yaw(roll, pitch, yaw):
    R_roll = Matrix([[1, 0, 0],
                     [0, cos(roll), -sin(roll)],
                     [0, sin(roll), cos(roll)]])

    R_pitch = Matrix([[cos(pitch), 0, sin(pitch)],
                      [0, 1, 0],
                      [-sin(pitch), 0, cos(pitch)]])

    R_yaw = Matrix([[cos(yaw), -sin(yaw), 0],
                    [sin(yaw), cos(yaw), 0],
                    [0, 0, 1]])
    return R_roll, R_pitch, R_yaw


def dh_transformation(theta_x, d_dz, theta_z, d_dx):
    return Matrix([[cos(theta_z), -sin(theta_z), 0, d_dz],
                   [sin(theta_z) * cos(theta_x), cos(theta_z) * cos(theta_x), -sin(theta_x), -sin(theta_x) * d_dx],
                   [sin(theta_z) * sin(theta_x), cos(theta_z) * sin(theta_x), cos(theta_x), cos(theta_x) * d_dx],
                   [0, 0, 0, 1]])


def do_rotation(axis, angle=None):
    if axis == "x":
        return Matrix([[1, 0, 0],
                       [0, cos(angle), -sin(angle)],
                       [0, sin(angle), cos(angle)]])
    elif axis == "y":
        return Matrix([[cos(angle), 0, sin(angle)],
                       [0, 1, 0],
                       [-sin(angle), 0, cos(angle)]])
    elif axis == "z":
        return Matrix([[cos(angle), -sin(angle), 0],
                       [sin(angle), cos(angle), 0],
                       [0, 0, 1]])
    else:
        return None


def correction_matrix():
    r_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                  [sin(pi), cos(pi), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    r_y = Matrix([[cos(-pi / 2), 0, sin(-pi / 2), 0],
                  [0, 1, 0, 0],
                  [-sin(-pi / 2), 0, cos(-pi / 2), 0],
                  [0, 0, 0, 1]])
    return simplify(r_z * r_y)


def calculate_wrist_center(P_EE, R0_6):
    return simplify(P_EE - 0.303 * R0_6 * Matrix([[1], [0], [0]]))


def calculate_theta1(J5):
    return atan2(J5[1], J5[0])


def get_joint2(theta1):
    joint2_starting_point = [0.35, 0, 0.75]
    joint2 = [joint2_starting_point[0] * cos(theta1), joint2_starting_point[0] * sin(theta1), joint2_starting_point[2]]
    return joint2


def calculate_theta3(joint5, theta1):
    joint3_starting_point = [0.35, 0, 2]
    joint5_starting_point = [1.85, 0, 1.946]
    joint2 = get_joint2(theta1)
    link2_5_on_x = joint5[0] - joint2[0]
    link2_5_on_y = joint5[1] - joint2[1]
    link2_5_on_z = joint5[2] - joint2[2]
    link2_5 = sqrt(link2_5_on_x ** 2 + link2_5_on_y ** 2 + link2_5_on_z ** 2)
    link2_3_starting_point = 1.25
    link3_5_starting_point_on_x = joint5_starting_point[0] - joint3_starting_point[0]
    link3_5_starting_point_on_z = joint5_starting_point[2] - joint3_starting_point[2]
    link3_5_starting_point = sqrt(link3_5_starting_point_on_x ** 2 + link3_5_starting_point_on_z ** 2)
    distance = (link2_5 ** 2 - link2_3_starting_point ** 2 - link3_5_starting_point ** 2) / -(
        2 * link2_3_starting_point * link3_5_starting_point)
    theta3_internal = atan2(sqrt(1 - distance ** 2), distance)
    theta3 = pi / 2 - (
        atan2(sqrt(1 - distance ** 2), distance) - atan2(link3_5_starting_point_on_z, link3_5_starting_point_on_x))
    return theta3_internal, theta3


def calculate_theta2(J5, J2, theta3_internal):
    link2_3_starting_point = 1.25
    joint3_starting_point = [0.35, 0, 2]
    joint5_starting_point = [1.85, 0, 1.946]

    link3_5_starting_point_on_x = joint5_starting_point[0] - joint3_starting_point[0]
    link3_5_starting_point_on_z = joint5_starting_point[2] - joint3_starting_point[2]
    link3_5_starting_point = sqrt(link3_5_starting_point_on_x ** 2 + link3_5_starting_point_on_z ** 2)

    distance1 = link3_5_starting_point * sin(theta3_internal)
    distance2 = link2_3_starting_point - link3_5_starting_point * cos(theta3_internal)
    alpha = atan2(distance1, distance2)
    beta = atan2(J5[2] - J2[2], sqrt((J5[0] - J2[0]) ** 2 + (J5[1] - J2[1]) ** 2))
    theta2 = pi / 2 - alpha - beta
    return theta2


def calculate_theta6(rotation3_6):
    return atan2(rotation3_6[1, 0], rotation3_6[0, 0])  # rotation about Z-axis


def calculate_theta5(rotation3_6):
    return atan2(-rotation3_6[2, 0], sqrt(
        rotation3_6[0, 0] * rotation3_6[0, 0] + rotation3_6[1, 0] * rotation3_6[1, 0]))  # rotation about Y-axis


def calculate_theta4(rotation3_6):
    return atan2(rotation3_6[2, 1], rotation3_6[2, 2])  # rotation about X-axis


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        '''
              Initialize service response
        '''
        joint_trajectory_list = []
        rospy.loginfo("Initialized Service Response: %s" % len(joint_trajectory_list))

        '''
               Define DH param symbols
        '''
        # link lengths
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        # link offsets
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        # Twist Angles(Alpha)
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        rospy.loginfo("Defined DH parameter symbols")

        # Joint angle symbols(Theta)
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        rospy.loginfo("Defined Joint Angle symbols")

        '''
               Define Modified DH Transformation matrix
        '''
        s = {alpha0: 0, a0: 0, d1: 0.75,
             alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
             alpha2: 0, a2: 1.25, d3: 0,
             alpha3: -pi / 2, a3: -0.054, d4: 1.5,
             alpha4: pi / 2, a4: 0, d5: 0,
             alpha5: -pi / 2, a5: 0, d6: 0,
             alpha6: 0, a6: 0, d7: 0.303, q7: 0}
        rospy.loginfo("Created DH parameter symbols dictionary")

        '''
               Create individual transformation matrices
        '''
        t0_1 = dh_transformation(alpha0, a0, q1, d1).subs(s)
        rospy.loginfo("Created TM between base link 0 to 1")
        t1_2 = dh_transformation(alpha1, a1, q2, d2).subs(s)
        rospy.loginfo("Created TM between link 1 to 2")
        t2_3 = dh_transformation(alpha2, a2, q3, d3).subs(s)
        rospy.loginfo("Created TM between link 2 to 3")
        t3_4 = dh_transformation(alpha3, a3, q4, d4).subs(s)
        rospy.loginfo("Created TM between link 3 to 4")
        t4_5 = dh_transformation(alpha4, a4, q5, d5).subs(s)
        rospy.loginfo("Created TM between link 4 to 5")
        t5_6 = dh_transformation(alpha5, a5, q6, d6).subs(s)
        rospy.loginfo("Created TM between link 5 to 6")
        t6_7 = dh_transformation(alpha6, a6, q7, d7).subs(s)
        rospy.loginfo("Created TM between link 6 to 7")
        t0_7 = simplify(t0_1 * t1_2 * t2_3 * t3_4 * t4_5 * t5_6 * t6_7)
        rospy.loginfo("Completed TM from base link to End Effector")

        '''
               Calculated Forward Kinematics including correction for End-Effector alignment axis
        '''
        rotation_correction = correction_matrix()
        transform_forward_kinematics = simplify(t0_7 * rotation_correction)
        rospy.loginfo("Calculated Forward Kinematics including correction for End-Effector alignment axis")

        '''
               Extract rotational component of transform matrix
        '''
        t0_3 = simplify(t0_1 * t1_2 * t2_3)
        rotation0_3 = t0_3[0:3, 0:3]

        for x in xrange(0, len(req.poses)):
            rospy.loginfo("**Starting Inverse Kinematics**")
            rospy.loginfo("Start End Effector-poses %s" % str(x))
            joint_trajectory_point = JointTrajectoryPoint()

            '''
                   Extract end-effector position and orientation from request
            '''
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            rospy.loginfo("Extracted End-Effector position")
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])
            rospy.loginfo("Extracted End-Effector orientation")

            '''
                   Calculate joint angles using Geometric IK method Close-form
            '''
            rotation_row, rotation_pitch, rotation_yaw = calculate_rotation_for_roll_pitch_yaw(roll, pitch, yaw)
            rotation0_6 = simplify(rotation_row * rotation_pitch * rotation_yaw)
            rospy.loginfo("Constructed Rotation based on target roll, pitch and yaw angles of End_effector")

            p_end_effector = Matrix([[px], [py], [pz]])
            joint5 = calculate_wrist_center(p_end_effector, rotation0_6)
            rospy.loginfo("Calculated Wrist Center (WC) based on translation along z-axis from End_Effector location")

            '''
                   Calculate Theta 1
            '''
            theta1 = calculate_theta1(joint5)
            rospy.loginfo("Calculated Theta 1")

            '''
                   Calculate Theta 3
            '''
            theta3_internal, theta3 = calculate_theta3(joint5, theta1)
            rospy.loginfo("Calculated Theta 3")

            '''
                   Calculate Theta 2
            '''
            joint2 = get_joint2(theta1)
            theta2 = calculate_theta2(joint2, joint5, theta3_internal)
            rospy.loginfo("Calculated Theta 2")

            '''
                   Calculate The Inverse Rotation of Theta 1 Theta 2 and Theta 3
            '''
            rotation0_3_matrix = rotation0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            rotation0_3_matrix_inverse = rotation0_3_matrix ** -1

            '''
                   Calculate Theta 6
            '''
            rotation3_6 = rotation0_3_matrix_inverse * rotation0_6
            theta6 = calculate_theta6(rotation3_6)
            rospy.loginfo("Calculated Theta 6")

            '''
                   Calculate Theta 5
            '''
            theta5 = calculate_theta5(rotation3_6)
            rospy.loginfo("Calculated Theta 5")

            '''
                   Calculate Theta 4
            '''
            theta4 = calculate_theta4(rotation3_6)
            rospy.loginfo("Calculated Theta 4")

            '''
                  Populate response for the Inverse Kinematics Request            
            '''
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            '''
                 Display Forward Kinematics            
            '''
            forward_kinematics = transform_forward_kinematics.evalf(
                subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
            rospy.loginfo("**Printing Forward Kinematics**")
            print forward_kinematics

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
