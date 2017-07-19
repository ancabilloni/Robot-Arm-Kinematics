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

def H_transform(alpha, a, d, theta):
    A = Matrix([[            cos(theta),           -sin(theta),           0,             a],
                [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    return A

def RotationMatrix(roll, pitch, yaw):
    R_x = Matrix([[1,         0,          0],
                  [0, cos(roll), -sin(roll)],
                  [0, sin(roll),  cos(roll)]])

    R_y = Matrix([[ cos(pitch), 0, sin(pitch)],
                  [          0, 1,          0],
                  [-sin(pitch), 0, cos(pitch)]])

    R_z = Matrix([[cos(yaw),-sin(yaw), 0],
                  [sin(yaw), cos(yaw), 0],
                  [       0,        0, 1]])

    return R_z*R_y*R_x

def R_correction(roll, pitch, yaw):
    R_x = Matrix([[1,         0,          0],
                  [0, cos(roll), -sin(roll)],
                  [0, sin(roll),  cos(roll)]])

    R_y = Matrix([[ cos(pitch), 0, sin(pitch)],
                  [          0, 1,          0],
                  [-sin(pitch), 0, cos(pitch)]])

    R_z = Matrix([[cos(yaw),-sin(yaw), 0],
                  [sin(yaw), cos(yaw), 0],
                  [       0,        0, 1]])

    return R_y*R_z


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
            
            # Joint angle symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

      
            # Modified DH params

            l1  = 0.330
            l2z = 0.420
            l2x = 0.350
            l3  = 1.250
            l4  = 0.96
            l5  = 0.54
            l6  = 0.193
            l_ee = 0.11

            theta1 = q1
            theta2 = q2 - pi/2
            theta3 = q3
            theta4 = q4
            theta5 = q5
            theta6 = q6
            theta7 = q7

            s = {alpha0:     0, a0:   0   , d1: 0.75,
                 alpha1: -pi/2, a1: 0.35  , d2: 0,
                 alpha2:     0, a2: 1.25  , d3: 0,
                 alpha3: -pi/2, a3: -0.054, d4: 1.5,
                 alpha4:  pi/2, a4:   0   , d5: 0,
                 alpha5: -pi/2, a5:   0   , d6: 0,
                 alpha6:     0, a6:   0   , d7: 0.303, q7: 0}

            
            # Define Modified DH Transformation matrix
            T0_1 = H_transform(alpha0, a0, d1, theta1)
            T1_2 = H_transform(alpha1, a1, d2, theta2)
            T2_3 = H_transform(alpha2, a2, d3, theta3)
            T3_4 = H_transform(alpha3, a3, d4, theta4)
            T4_5 = H_transform(alpha4, a4, d5, theta5)
            T5_6 = H_transform(alpha5, a5, d6, theta6)
            T6_EE = H_transform(alpha6, a6, d7, theta7)

            # Create individual transformation matrices
            T0_1 = T0_1.subs(s)
            T1_2 = T1_2.subs(s)
            T2_3 = T2_3.subs(s)
            T3_4 = T3_4.subs(s)
            T4_5 = T4_5.subs(s)
            T5_6 = T5_6.subs(s)
            T6_EE = T6_EE.subs(s)
            # T0_6 = simplify(T0_1*T1_2*T2_3*T3_4*T4_5*T5_6)
            T0_3 = simplify(T0_1*T1_2*T2_3)
            
            # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            R0_EE = RotationMatrix(roll, pitch, yaw)

            R_corr = R_correction(0, pi/2, pi)

            R0_EE = R0_EE*R_corr
            
            wx = px - (l6 + l_ee)*R0_EE[0,2]
            wy = py - (l6 + l_ee)*R0_EE[1,2]
            wz = pz - (l6 + l_ee)*R0_EE[2,2]

            # wx = px - (l6 + l_ee)*r11
            # wy = py - (l6 + l_ee)*r21
            # wz = pz - (l6 + l_ee)*r31

            #Find q1, q2, q3
            
            r = sqrt(wx**2 + wy**2)
            phi = atan2(wz - 0.75, r - 0.35)
            l25 = sqrt((wz -0.75)**2 + (r-0.35)**2)
            # l3 = 1.25
            l45 = sqrt(1.5**2 + 0.054**2)

            angle3_2_5 = acos((l25**2 + l3**2 - l45**2)/(2*l25*l3))
            
            angle2_3_5 = acos((l3**2 + l45**2 - l25**2)/(2*l3*l45))

            theta1 = atan2(wy, wx)
            theta1 = theta1.evalf()
            theta2 = pi/2 - phi - angle3_2_5
            theta2 = theta2.evalf()
            theta3 = pi/2 - angle2_3_5 - atan2(0.054, 1.5)
            theta3 = theta3.evalf()

            T0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            R3_6 = T0_3[:3, :3].transpose() * R0_EE
            x4 = -R3_6[0,2]
            y6 = -R3_6[1,1]
            y5 = sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2])

            theta4 = atan2(R3_6[2,2], x4)
            theta4 = theta4.evalf()

            # theta5 = acos(R3_6[1,2])
            theta5 = atan2(y5,R3_6[1,2])
            theta5 = theta5.evalf()

            theta6 = atan2(y6, R3_6[1,0])
            theta6 = theta6.evalf()
            print ("Inverse Kinematic - Joint angles from 1 to 6: "),
            print (theta1, theta2, theta3, theta4, theta5, theta6)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

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
