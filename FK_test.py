#!/usr/bin/env python

# import modules
import tf
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt

from my_kuka_robot.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import mpmath
from sympy import *

def debug_log(log):
	if (debug_mode):
		print(log)
	pass

def rand_in_range(low, high):
	return low + np.random.rand()*(high - low)

# Radians to Degree
def rtod(q):
	return q*180.0/np.pi

# Degree to Radians
def dtor(q):
	return q*np.pi/180.0

class FKTest(object):

	def __init__(self):
		debug_log("Step 1: initializing test script...")
		# TODO
                rospy.init_node('FK_test')
                #rospy.init_node('my_kuka')
                #rospy.init_node('IK_test')
		debug_log("Step 2: waiting for service to be available.")
		# TODO
                rospy.wait_for_service('my_kuka_FK')
                #rospy.wait_for_service(my_kuka_IK)

		debug_log("Step 3: creating a callable proxy to a service")
		# TODO
                self.FKSolver=rospy.ServiceProxy('my_kuka_FK',robot_FK)
                #rospy.FKSolver=rospy.ServiceProxy(my_kuka_FK,robot_IK)

		self.FK_REQ = [	JointTrajectoryPoint(),
			JointTrajectoryPoint(),
			JointTrajectoryPoint()]
		self.FK_REQ[0].positions = (2.92, 0.01, -2.72, 2.94, 0.49, 3.65)
		self.FK_REQ[1].positions = (3.21, 1.16, -1.81, -4.04, -1.07, -3.72)
		self.FK_REQ[2].positions = (1.66, 0.84, -3.33, 1.27, -0.28, 2.96)

		self.FK_RESP = [Pose(),
			Pose(),
			Pose()]
		self.FK_RESP[0].position.x = 1.241097506
		self.FK_RESP[0].position.y = -0.308878104954
		self.FK_RESP[0].position.z = 2.66139826936
		self.FK_RESP[1].position.x = -2.94615069338
		self.FK_RESP[1].position.y = 0.00657260199077
		self.FK_RESP[1].position.z = 2.07021476465
		self.FK_RESP[2].position.x = 0.0908570554147
		self.FK_RESP[2].position.y = -0.118085650178
		self.FK_RESP[2].position.z = 2.69381803552	
		self.FK_RESP[0].orientation.x = 0.973569123941
		self.FK_RESP[0].orientation.y = -0.159235851633
		self.FK_RESP[0].orientation.z = 0.00124492841665
		self.FK_RESP[0].orientation.w = -0.163724019669
		self.FK_RESP[1].orientation.x = -0.25579584927
		self.FK_RESP[1].orientation.y = -0.53186509833
		self.FK_RESP[1].orientation.z = 0.772125532224
		self.FK_RESP[1].orientation.w = 0.23560594891
		self.FK_RESP[2].orientation.x = 0.220283358922
		self.FK_RESP[2].orientation.y = -0.434902191935
		self.FK_RESP[2].orientation.z = -0.378367271047
		self.FK_RESP[2].orientation.w = 0.786875805596
		pass

	def generate_FK_request(self):
		theta1 = rand_in_range(dtor(-185), dtor(185))
		theta2 = rand_in_range(dtor(-45), dtor(85))
		theta3 = rand_in_range(dtor(-210), dtor(65))
		theta4 = rand_in_range(dtor(-350), dtor(350))
		theta5 = rand_in_range(dtor(-125), dtor(125))
		theta6 = rand_in_range(dtor(-350), dtor(350))

		theta_t = (theta1, theta2, theta3, theta4, theta5, theta6)
		return theta_t

	def run_tests(self):
		errors = 0
		debug_log("Step 4: ready for testing.")
		debug_log("\n    Part 1: predefined tests")
		# TODO: Response: FK_resp // Request: self.FK_REQ
                FK_resp=self.FKSolver(self.FK_REQ)
		errors += self.verify_result(FK_resp)

		debug_log("\n    Part 2: random tests")
		FK_reqs = []
		for i in range(5):
			joint_trajectory_point = JointTrajectoryPoint()
			joint_trajectory_point.positions = self.generate_FK_request()
			FK_reqs.append(joint_trajectory_point)
		
		FK_resp = self.FKSolver(FK_reqs)
		self.show_result(FK_reqs, FK_resp)

		if(errors < 1):
			debug_log("FK test passed!")
		else:
			debug_log("FK test failed")

	def show_result(self,FK_reqs, FK_resp):
		for x in xrange(0, len(FK_resp.poses)): 
			pos_r = FK_resp.poses[x].position
			ori_r = FK_resp.poses[x].orientation
			debug_log("        FK_request  {}: joint angles = {}".format(x, FK_reqs[x].positions))
			debug_log("        FK_response {}: position =     {},{},{}".format(x, pos_r.x, pos_r.y, pos_r.z))	
			debug_log("        FK_response {}: orientation =  {},{},{},{}\n".format(x, ori_r.x,ori_r.y,ori_r.z,ori_r.w))

	def verify_result(self,FK_resp):
		errors = 0
		for x in xrange(0, len(FK_resp.poses)): 
			pos_r = FK_resp.poses[x].position
			ori_r = FK_resp.poses[x].orientation
			pos_e = self.FK_RESP[x].position
			ori_e = self.FK_RESP[x].orientation

			pos_r_mat = Matrix([[pos_r.x], [pos_r.y], [pos_r.z]])
			pos_e_mat = Matrix([[pos_e.x], [pos_e.y], [pos_e.z]])
			err = (pos_r_mat - pos_e_mat).norm()

			debug_log("        received FK_response {}: position =    {},{},{}".format(x, pos_r.x, pos_r.y, pos_r.z))	
			debug_log("                 FK_response {}: orientation = {},{},{},{}".format(x, ori_r.x,ori_r.y,ori_r.z,ori_r.w))
			debug_log("        expected FK_response {}: position =    {},{},{}".format(x, pos_e.x, pos_e.y, pos_e.z))	
			debug_log("                 FK_response {}: orientation = {},{},{},{}".format(x, ori_e.x,ori_e.y,ori_e.z,ori_e.w))
			debug_log("        error:   {}\n".format(err))
			errors += err
		return errors


if __name__ == "__main__":

	global debug_mode
	debug_mode = True

	my_kuka_FK_test = FKTest()
	my_kuka_FK_test.run_tests()

	rospy.spin()
