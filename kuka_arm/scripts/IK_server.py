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


def find_wrist_center(end_effector_point, end_effector_roll_pitch_yaw):
    px, py, pz = end_effector_point
    roll_, pitch_, yaw_ = end_effector_roll_pitch_yaw
    rotation0_6 = simplify(do_rotation("x", roll_) * do_rotation("y", pitch_) * do_rotation("z", yaw_))
    return simplify(Matrix([[px], [py], [pz]]) - rotation0_6 * Matrix([[0.303], [0], [0]])), rotation0_6


def get_theta1():
    wrist_center, r_0_6 = find_wrist_center((px, py, pz), (roll, pitch, yaw))
    theta1_ = atan2(wrist_center[1], wrist_center[0])
    print"Get theta 1: " + str(theta1_)
    return theta1_


def get_joint2_coordinate(theta1):
    joint2_starting_point = [0.35, 0, 0.75]
    return [joint2_starting_point[0] * cos(theta1), joint2_starting_point[0] * sin(theta1), joint2_starting_point[2]]


def get_theta3(joint_2_end):
    a2_3 = 1.25
    a3_5 = sqrt(1.5 ** 2 + 0.054 ** 2)
    wrist_center, r_0_6 = find_wrist_center((px, py, pz), (roll, pitch, yaw))
    theta3_internal_init = pi / 2 + atan2(-0.054, 1.5)
    a2_5 = (wrist_center[0] - joint_2_end[0], wrist_center[1] - joint_2_end[1], wrist_center[2] - joint_2_end[2])
    distance_2_to_5 = sqrt(a2_5[0] ** 2 + a2_5[1] ** 2 + a2_5[2] ** 2)
    cos_theta3_internal = (distance_2_to_5 ** 2 - a2_3 ** 2 - a3_5 ** 2) / (-2 * a2_3 * a3_5)
    theta3_internal = atan2(sqrt(1 - cos_theta3_internal ** 2), cos_theta3_internal)
    theta3_ = theta3_internal_init - theta3_internal
    print"Get theta 3:" + str(theta3_)
    return theta3_


def get_theta2(joint_2_end):

    a3_5 = sqrt(1.5 ** 2 + 0.054 ** 2)
    a2_3 = 1.25

    wrist_center, r_0_6 = find_wrist_center((px, py, pz), (roll, pitch, yaw))
    a2_5 = (wrist_center[0] - joint_2_end[0], wrist_center[1] - joint_2_end[1], wrist_center[2] - joint_2_end[2])
    distance_2_to_5 = sqrt(a2_5[0] ** 2 + a2_5[1] ** 2 + a2_5[2] ** 2)
    cos_theta3_internal = (distance_2_to_5 ** 2 - a2_3 ** 2 - a3_5 ** 2) / (-2 * a2_3 * a3_5)
    theta3_internal = atan2(sqrt(1 - cos_theta3_internal ** 2), cos_theta3_internal)
    angle_delta1 = atan2(a2_3 - a3_5 * cos_theta3_internal, a3_5 * sin(theta3_internal))
    joint_2_wrist_center_dz = joint_2_end[2] - wrist_center[2]
    joint_2_wrist_center_dr = sqrt((wrist_center[0] - joint_2_end[0]) ** 2 + (wrist_center[1] - joint_2_end[1]) ** 2)
    angle_delta2 = atan2(joint_2_wrist_center_dz, joint_2_wrist_center_dr)
    theta2_ = angle_delta1 + angle_delta2
    print "Get theta 2:" + str(theta2_)
    return theta2_


def ge_theta4(r_3_6):
    print "Get theta 4"
    theta4_ = atan2(r_3_6[2, 1], r_3_6[2, 2])  # rotation about X-axis
    print "Get theta 4:" + str(theta4_)
    return theta4_


def get_theta5(r_3_6):
    print "Get theta 5"
    theta5_ = atan2(-r_3_6[2, 0], sqrt(r_3_6[0, 0] * r_3_6[0, 0] + r_3_6[1, 0] * r_3_6[1, 0]))  # rotation about Y-axis
    print "Get theta 5:" + str(theta5_)
    return theta5_


def get_theta6(r_3_6):
    print "Get theta 6"
    theta6_ = atan2(r_3_6[1, 0], r_3_6[0, 0])  # rotation about Z-axis
    print "Get theta 6:" + str(theta6_)
    return theta6_


def inverse_kinematics():
    """Calculate Inverse Kinematics in closed form"""
    global t0_1, t1_2, t2_3, px, py, pz, roll, pitch, yaw
    global theta1, theta2, theta3, theta4, theta5, theta6

    # region initial link lengths and joint positions
    # the first angle is straightforward
    theta1 = get_theta1()

    # find the coordinates of joint 2 which are a simple rotation by theta1 of
    # the initial joint
    joint_2_end = get_joint2_coordinate(theta1)

    # region find theta3
    # applying the cosines formula for the triangle joint_2-joint_3-joint-5
    theta3 = get_theta3(joint_2_end)
    # endregion

    # region find theta2
    theta2 = get_theta2(joint_2_end)
    # endregion

    # region find theta3-6
    # extract the rotational part of the transformation
    r_0_3 = simplify((t0_1 * t1_2 * t2_3)[0:3, 0:3])
    # since r_0_3 inverse is the transpose of the matrix
    wrist_center, r_0_6 = find_wrist_center((px, py, pz), (roll, pitch, yaw))
    r_3_6 = r_0_3.T * r_0_6

    # from the euler angle lesson we get the final rotations
    theta4 = ge_theta4(r_3_6)
    theta5 = get_theta5(r_3_6)
    theta6 = get_theta6(r_3_6)


def calculate_from_pose(pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw):
    """Calculate angles given a known pose"""
    global theta1, theta2, theta3, theta4, theta5, theta6
    global px, py, pz, roll, pitch, yaw

    s = {px: pose_x, py: pose_y, pz: pose_z,
         roll: pose_roll, pitch: pose_pitch, yaw: pose_yaw}
    t1, t2, t3 = theta1.evalf(subs=s), theta2.evalf(subs=s), theta3.evalf(subs=s)
    s.update({q1: t1, q2: t2, q3: t3})
    t4, t5, t6 = theta4.evalf(subs=s), theta5.evalf(subs=s), theta6.evalf(subs=s)
    return [t1, t2, t3, t4, t5, t6]


def apply_FK(thetas):
    print "Apply FK using known thetas"
    return t_FK.evalf(
        subs={
            q1: thetas[0], q2: thetas[1], q3: thetas[2], q4: thetas[3], q5: thetas[4], q6: thetas[5]})


def total_error(prediction, ground_truth):
    print "Error: prediction - ground_truth"
    return (prediction - ground_truth).norm()


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        joint_trajectory_errors = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px_ = req.poses[x].position.x
            py_ = req.poses[x].position.y
            pz_ = req.poses[x].position.z

            (roll_, pitch_, yaw_) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method
            robot_angles = calculate_from_pose(px_, py_, pz_, roll_, pitch_, yaw_)
            # use FK to calculate the EE position
            prediction = apply_FK(robot_angles) * Matrix([[0], [0], [0], [1]])
            ground_truth = Matrix([[px_], [py_], [pz_], [1]])
            error = total_error(prediction, ground_truth)

            joint_trajectory_point.positions = robot_angles
            joint_trajectory_list.append(joint_trajectory_point)
            joint_trajectory_errors.append(error)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        rospy.loginfo("average error in Joint Trajectory calculation: %s" % (
            sum(joint_trajectory_errors) / len(joint_trajectory_errors)))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


# define global variables and create the FK and IK solution once globally
# boosting performance instead of running it every time I have to
# calculate a series of poses


# Define DH param symbols
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # link lengths
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # link offsets
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # 'theta_z' joint angles
s0, s1, s2, s3, s4, s5, s6 = symbols('s0:7')  # 'theta_x' twist angles

px, py, pz = symbols('px py pz')
roll, pitch, yaw = symbols('roll pitch yaw')
theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1:7')

# DH Dictionary
s = {s0: 0, a0: 0, d1: 0.75,
     s1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
     s2: 0, a2: 1.25, d3: 0,
     s3: -pi / 2, a3: -0.054, d4: 1.5,
     s4: pi / 2, a4: 0, d5: 0,
     s5: -pi / 2, a5: 0, d6: 0,
     s6: 0, a6: 0, d7: 0.303, q7: 0}

# Forward Kinematics
t0_1 = dh_transformation(s0, a0, q1, d1).subs(s)
t1_2 = dh_transformation(s1, a1, q2, d2).subs(s)
t2_3 = dh_transformation(s2, a2, q3, d3).subs(s)
t3_4 = dh_transformation(s3, a3, q4, d4).subs(s)
t4_5 = dh_transformation(s4, a4, q5, d5).subs(s)
t5_6 = dh_transformation(s5, a5, q6, d6).subs(s)
t6_7 = dh_transformation(s6, a6, q7, d7).subs(s)
t0_7 = simplify(t0_1 * t1_2 * t2_3 * t3_4 * t4_5 * t5_6 * t6_7)

# forward kinematics including correction for end effector alignment axis
t_FK = t0_7 * correction_matrix()

# now the theta angles have been calculated in closed form.
# and are stored in theta_i symbolic variables
inverse_kinematics()
if __name__ == "__main__":
    print "Ready to receive an IK request"
    IK_server()
