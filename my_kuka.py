#!/usr/bin/env python

# import modules
import os
import tf
import sys
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt

from my_kuka_robot.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import mpmath
from sympy import *

# display debug messages
def debug_log(log):
    if (debug_mode):
        print(log)
    pass

# DH parameters (symbolic form)
al0, al1, al2, al3, al4, al5, al6 = symbols('alpha0:7')  	# link_twist
a0, a1, a2, a3, a4, a5, a6 	  = symbols('a0:7')  		# link_length
th1, th2, th3, th4, th5, th6, th7 = symbols('theta1:8')  	# joint_twist
d1, d2, d3, d4, d5, d6, d7 	  = symbols('d1:8')  		# joint_length

# TODO: fill in the DH table for the kuka arm
s = {al0:0     , a0:0        , d1:0.75  ,  
     al1:-pi/2 , a1:0.35     , d2:0     ,  th2: th2-pi/2,
     al2:0     , a2:1.25     , d3:0     ,  
     al3:-pi/2 , a3:-0.054   , d4:1.5   ,  
     al4:pi/2  , a4:0        , d5:0     ,  
     al5:-pi/2 , a5:0        , d6:0     ,  
     al6:0     , a6:0        , d7:0.303 ,  th7:0 }

def createMatrix(al, a, th, d):
    # TODO: Fill in the 2-4 rows of matrix
    mat =  Matrix([[             cos(th),           -sin(th),         0,          a ],
                   [     sin(th)*cos(al),    cos(th)*cos(al),  -sin(al),  -sin(al)*d],
                   [     sin(th)*sin(al),    cos(th)*sin(al),   cos(al),   cos(al)*d],
                   [           0,                  0,                 0,          1]])

    return mat


# Rotation Matrix about X
def rot_x(q):
    R_x = Matrix([[       1,       0,       0 ],
                  [       0,  cos(q), -sin(q) ],
                  [       0,  sin(q),  cos(q) ]])
    return R_x

# Rotation Matrix about Y
def rot_y(q):
    R_y = Matrix([[  cos(q),       0,  sin(q) ],
                  [       0,       1,       0 ],
                  [ -sin(q),       0,  cos(q) ]])
    return R_y

# Rotation Matrix about Z
def rot_z(q):
    R_z = Matrix([[  cos(q), -sin(q),       0 ],
                  [  sin(q),  cos(q),       0 ],
                  [       0,       0,       1 ]])
    return R_z


# Radians to Degree
def rtod(q):
    return q*180.0/np.pi

# Degree to Radians
def dtor(q):
    return q*np.pi/180.0

def checkInRange(val, low, high, name):
    if((val >= low) & (val <= high)):
        return True
    else:
        print ("        Out of range {}[{}], ({}, {})".format(name, val, low, high))
        return False;


class MyKukaSolver(object):


    def __init__(self):

        self.T0_1 = createMatrix(al0,a0, th1, d1)
        self.T0_1 = self.T0_1.subs(s)
        # TODO: Do the same for the remaining parts
        self.T1_2 = createMatrix(al1,a1, th2, d2)
        self.T1_2 = self.T1_2.subs(s)
        self.T2_3 = createMatrix(al2,a2, th3, d3)
        self.T2_3 = self.T2_3.subs(s)
        self.T3_4 = createMatrix(al3,a3, th4, d4)
        self.T3_4 = self.T3_4.subs(s)
        self.T4_5 = createMatrix(al4,a4, th5, d5)
        self.T4_5 = self.T4_5.subs(s)
        self.T5_6 = createMatrix(al5,a5, th6, d6)
        self.T5_6 = self.T5_6.subs(s)
        self.T6_G = createMatrix(al6,a6, th7, d7)
        self.T6_G = self.T6_G.subs(s)

        # TODO: Do the same for the remaining parts
        self.T0_2 = simplify(self.T0_1 * self.T1_2) # base_link to link 2
        self.T0_3 = simplify(self.T0_2 * self.T2_3) # base_link to link 3
        self.T0_4 = simplify(self.T0_3 * self.T3_4)
        self.T0_5 = simplify(self.T0_4 * self.T4_5)
        self.T0_6 = simplify(self.T0_5 * self.T5_6)
        self.T0_G = simplify(self.T0_6 * self.T6_G)

        # TODO: Correction Needed to account for orientation difference between definition
        # of gripper_link in URDF versus DH Convention
        self.R_corr = Matrix(simplify(rot_y(pi/2)*rot_z(pi)))
        
        # Compute complete transform for End effector
        R_corr2 = self.R_corr.row_insert(3, Matrix([[0, 0, 0]]))
        R_corr2 = R_corr2.col_insert(3, Matrix([0, 0, 0, 1]))
        self.T_total = simplify(self.T0_G * R_corr2)


        # Rotation transform between link 3 and 6, defined symbollically based 
        # on the Modified DH parameters. 
        self.R3_6_prime = (self.T3_4 * self.T4_5 * self.T5_6)
        self.R3_6_prime = self.R3_6_prime[:3,:3]

        # Memoization for theta4 and theta6
        self.old_th4 = 0
        self.old_th6 = 0
        pass



    def performFK(self, theta_t):
        theta_s = {th1: theta_t[0], th2: theta_t[1], th3: theta_t[2], th4: theta_t[3], th5: theta_t[4], th6:theta_t[5]}
        # theta_s = {q1: theta_t[0], q}
        origin = Matrix([[0], [0], [0], [1]])

        T0_2_prime = self.T0_2.evalf(subs=theta_s) 
        p2 = T0_2_prime*origin
        rpy2 = tf.transformations.euler_from_matrix(T0_2_prime.tolist())
        quat2 = tf.transformations.quaternion_from_matrix(T0_2_prime.tolist())
        debug_log("        Link 2 position : {}".format(p2.tolist()))

        T0_3_prime = self.T0_3.evalf(subs=theta_s) 
        p3 = T0_3_prime*origin
        rpy3 = tf.transformations.euler_from_matrix(T0_3_prime.tolist())
        quat3 = tf.transformations.quaternion_from_matrix(T0_3_prime.tolist())
        debug_log("        Link 3 position : {}".format(p3.tolist()))

        T0_5_prime = self.T0_5.evalf(subs=theta_s) 
        p5 = T0_5_prime*origin
        rpy5 = tf.transformations.euler_from_matrix(T0_5_prime.tolist())
        quat5 = tf.transformations.quaternion_from_matrix(T0_5_prime.tolist())
        debug_log("        Link 5/Wrist Center position : {}".format(p5.tolist()))

        T0_G_prime = self.T0_G.evalf(subs=theta_s) 
        pG = T0_G_prime*origin
        rpyG = tf.transformations.euler_from_matrix(T0_G_prime.tolist())
        quatG = tf.transformations.quaternion_from_matrix(T0_G_prime.tolist())
        debug_log("        Gripper/End Effector position : {}".format(pG.tolist()))

        # TODO: Find the FK result given theta_s values
        T_total_prime = self.T_total.evalf(subs=theta_s)
        pFinal = T_total_prime*origin
        rpyFinal = tf.transformations.euler_from_matrix(T_total_prime.tolist())
        quatFinal = tf.transformations.quaternion_from_matrix(T_total_prime.tolist())
        debug_log("    EE URDF position:    {}".format(pFinal.tolist()))
        debug_log("    EE URDF orientation: {}".format(quatFinal))

        result = Pose()
        result.position.x = pFinal[0]
        result.position.y = pFinal[1]
        result.position.z = pFinal[2]
        result.orientation.x = quatFinal[0]
        result.orientation.y = quatFinal[1]
        result.orientation.z = quatFinal[2]
        result.orientation.w = quatFinal[3]
        return result

    def performIK(self, pose):
        # -------For HW3------------
        # add something later

        # Populate response for the IK request
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = (0, 0, 0, 0, 0, 0)
        debug_log("    joint angles = {}".format(joint_trajectory_point.positions))
        return joint_trajectory_point

    def verifyIK(self, pose, joint_trajectory_point):
        # -------For HW3------------
        # add something later
        error = 1
        return error

    def handle_calculate_FK(self, req):
        if len(req.points) < 1: 
            debug_log("\nRECEIVED A CORRUPTED \"my_kuka_FK\" SERVICE REQUEST... EXIT\n")
            return -1
        else:
           debug_log("\nRECEIVED A \"my_kuka_FK\" SERVICE REQUEST!")
        
        # Initialize service response
        ee_pose_list = []

        for x in xrange(0, len(req.points)): 
            debug_log("\n    processing point {}".format(x))
            # extract robot joint angles
            theta_t = req.points[x].positions

            # compute FK
            pose = self.performFK(theta_t)
            ee_pose_list.append(pose)

        debug_log("\nRESPONDED TO \"my_kuka_FK\" SERVICE REQUEST! (length={})".format(len(ee_pose_list)))
        return robot_FKResponse(ee_pose_list)

    def handle_calculate_IK(self, req):
        if len(req.poses) < 1: 
            debug_log("\nRECEIVED A CORRUPTED \"my_kuka_IK\" SERVICE REQUEST... EXIT\n")
            return -1
        else:
            debug_log("\nRECEIVED A \"my_kuka_IK\" SERVICE REQUEST!")
        
        # Initialize service response
        joint_trajectory_list = []
        position_errors = []

        self.old_th4 = 0
        self.old_th6 = 0

        for x in xrange(0, len(req.poses)): 
            debug_log("\n    processing pose {}".format(x))
            # extract robot pose
            pose = req.poses[x]
                    
            # compute IK
            joint_trajectory_point = self.performIK(pose)
            joint_trajectory_list.append(joint_trajectory_point)

            # verify IK accuracy
            error = self.verifyIK(pose, joint_trajectory_point)
            position_errors.append(error)

        debug_log("\nRESPONDED TO \"my_kuka_IK\" SERVICE REQUEST! (length={})".format(len(joint_trajectory_list)))
        return robot_IKResponse(joint_trajectory_list)



if __name__ == "__main__":

    global debug_mode
    debug_mode = True

    debug_log("Step 1: initializing my_kuka solver...")
    rospy.init_node('my_kuka')
    solver = MyKukaSolver()

    debug_log("Step 2: creating a callable proxy to a service")
    # TODO: do this for both FK and IK services
    # rospy.Service('service name', service type, handler function)
    rospy.Service('my_kuka_FK',robot_FK,solver.handle_calculate_FK) # handle function == file name
    rospy.Service('my_kuka_IK',robot_IK,solver.handle_calculate_IK) # service type means where you store launch scripts and src
    # service: the service

    

    debug_log("Step 3: ready to receive requests")
    rospy.spin()
