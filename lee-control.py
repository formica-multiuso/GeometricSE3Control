#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import sys
import tf
import PyKDL
import numpy as np

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
#from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

from tf import TransformListener
#from tf import TransformerROS
from tf.transformations import euler_from_quaternion
from math import sin, cos


e_p = Point()
e_v = Point()
r = Point()
r_T = Point()
r_dot = Point()
r_dot_T = Point()
ori_T = Point()
F_des = PointStamped()
u = JointState()
e_omega = Point()
b_omega = PointStamped()
b_omega_T = Point()
linear_acceleration = PointStamped()
angular_acceleration = PyKDL.Vector()


trajPos=Point()
trajEuler=Point()
trajLinearVel=Point()
trajAngularVel=PointStamped()

## Constant

m = 1.0
g = 9.8

quaternion = (0,0,0,0)

def kdl_to_mat(data):
        mat =  np.mat(np.zeros((3,3)))
        for i in range(0,3):
            for j in range(0,3):
                mat[i,j] = data[i,j]
        return mat

def subtract(a,b):
	res = PyKDL.Rotation()
	res[0,0] = a[0,0] - b[0,0]
	res[0,1] = a[0,1] - b[0,1]
	res[0,2] = a[0,2] - b[0,2]
	res[1,0] = a[1,0] - b[1,0]
	res[1,1] = a[1,1] - b[1,1]
	res[1,2] = a[1,2] - b[1,2]
	res[2,0] = a[2,0] - b[2,0]
	res[2,1] = a[2,1] - b[2,1]
	res[2,2] = a[2,2] - b[2,2]
	return res
#	return np.subtract(kdl_to_mat(a),kdl_to_mat(b))

def vee_map(matrix):
	return (-matrix[2,1],matrix[0,2],-matrix[1,0])

def PoseCallback(data):
	try:
		r.x = data.pose.position.x
		r.y = data.pose.position.y
		r.z = data.pose.position.z
		
		global quaternion

		quaternion = (data.pose.orientation.x,
                              data.pose.orientation.y,
                              data.pose.orientation.z,
                              data.pose.orientation.w)

	except:
		print "Error happens in PoseCallback registration"

def TwistCallback(data):
	try:
		r_dot.x = data.twist.linear.x
		r_dot.y = data.twist.linear.y
		r_dot.z = data.twist.linear.z
		## Here we use a PointStamped for a velocity to being able using the function transformPoint
		b_omega.point.x = data.twist.angular.x
		b_omega.point.y = data.twist.angular.y
		b_omega.point.z = data.twist.angular.z
#		b_omega.header.frame_id = "world"
#		print "TwistCallback ok"
	except:
		print "Error happens in TwistCallback registration"

def closestPointPosCallback(data):
	try:	
		r_T.x = data.x
		r_T.y = data.y
		r_T.z = data.z
#		print "closestPointPosCallback ok"
	except:
		print "Error happens in closestPointPosCallback registration"

def closestPointOriCallback(data):
	try:	
		ori_T.x = data.x
		ori_T.y = data.y
		ori_T.z = data.z		
#		print "closestPointOriCallback ok"
	except:
		print "Error happens in closestPointOriCallback registration"

def trajectoryPointPoseCallback(data):
	try:
		trajPos.x = data.pose.position.x
                trajPos.y = data.pose.position.y
                trajPos.z = data.pose.position.z

                quaternion_des = (data.pose.orientation.x,
                              data.pose.orientation.y,
                              data.pose.orientation.z,
                              data.pose.orientation.w)

                (trajEuler.x,trajEuler.y,trajEuler.z) = tf.transformations.euler_from_quaternion(quaternion_des)	
	#	print (trajEuler.x,trajEuler.y,trajEuler.z)
		
	except:
		print "Error happens in trajectoryPointPoseCallback registration"

def trajectoryPointTwistCallback(data):
        try:
                trajLinearVel.x = data.twist.linear.x
                trajLinearVel.y = data.twist.linear.y
                trajLinearVel.z = data.twist.linear.z

                trajAngularVel.point.x = data.twist.angular.x
                trajAngularVel.point.y = data.twist.angular.y
                trajAngularVel.point.z = data.twist.angular.z
		trajAngularVel.header.frame_id = "world"
		
               # quaternion = (data.pose.orientation.x,
               #               data.pose.orientation.y,
               #               data.pose.orientation.z,
               #               data.pose.orientation.w)

               # (trajEuler.x,trajEuler.y,trajEuler.z) = tf.transformations.euler_from_quaternion(quaternion)
                #print trajEuler.x, trajEuler.y, trajEuler.z
		
		

        except:
                print "Error happens in trajectoryPointTwistCallback registration"


def init():
	try:
		print "init ok"

	except:
		print "Error happens in init"

def controller():

	tfl = TransformListener()
	body_F_des = PointStamped()
	body_b_omega = PointStamped() # Angular Velocity of the body expressed in the body frame
	body_M_des = PointStamped()
	
	Kp = PyKDL.Vector(50,50,15)
	Kv = PyKDL.Vector(30,30,7)
	Kr = PyKDL.Vector(-0.001,-0.001,-0.01)
	Ko = PyKDL.Vector(-0.00001,-0.00001,-0.02)


####### M=4.34 J=[0.0820,0.0845,0.1377]
#	Kp = PyKDL.Vector(16,16,16)
#	Kv = PyKDL.Vector(5.6,5.6,5.6)
#	Kr = PyKDL.Vector(-8.81,-8.81,-8.81)
#	Ko = PyKDL.Vector(-2.54,-2.54,-2.54)


	while not rospy.is_shutdown():
        	
		t = rospy.Time(0)
	#	tfl.waitForTransform('quadrotor_0','world',t,rospy.Duration(4.0))		
	#	(position,quaternion) = tfl.lookupTransform("quadrotor_0","world",t) 
	
#		print quaternion
		R = PyKDL.Rotation.Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
#		print R
		b_omega.header.frame_id = "world"	
#		body_b_omega = tfl.transformPoint('quadrotor_0', b_omega)
		e_p = PyKDL.Vector(r.x - trajPos.x, r.y - trajPos.y, r.z - trajPos.z)
#		print e_p		
		
		velocity_W = PyKDL.Vector(r_dot.x,r_dot.y,r_dot.z)
		command_velocity_W = PyKDL.Vector(trajLinearVel.x,trajLinearVel.y,trajLinearVel.z)
		
		e_v = velocity_W - command_velocity_W

#		print e_v		

		linear_acceleration = PyKDL.Vector(e_p[0]*Kp[0],e_p[1]*Kp[1],e_p[2]*Kp[2]) + PyKDL.Vector(e_v[0]*Kv[0]/m,e_v[1]*Kv[1]/m,e_v[2]*Kv[2]/m) - PyKDL.Vector(0,0,g) - PyKDL.Vector(0,0,0) 
	
		lin_acc_kdl = PyKDL.Vector(linear_acceleration[0],linear_acceleration[1],linear_acceleration[2])
		print lin_acc_kdl

		b1_des = PyKDL.Vector(cos(trajEuler.z),sin(trajEuler.z),0.0)
		b3_des = -lin_acc_kdl / lin_acc_kdl.Norm()	# HERE Coupling between linear/rotational dynamics
		b2_des = b3_des * b1_des
		b2_des.Normalize()
		R_des = PyKDL.Rotation(b2_des*b3_des,b2_des,b3_des)
	#	print R_des
	#	R_des = PyKDL.Rotation(b1_des,b2_des,b3_des)
#		R_des = R_des.Inverse()	# I want the TRANSPOSE (that's also the INVERSE)

		angle_error_matrix = PyKDL.Rotation()

		angle_error_matrix = subtract(R_des.Inverse()*R,R.Inverse()*R_des)
		
	#	print angle_error_matrix
		angle_error_vee = vee_map(angle_error_matrix)
	#	print angle_error_vee
		angle_error = PyKDL.Vector(angle_error_vee[0]*0.5,angle_error_vee[1]*0.5,angle_error_vee[2]*0.5)
				
#		print angle_error
		
		angular_velocity = PyKDL.Vector(b_omega.point.x, b_omega.point.y, b_omega.point.z)
		angular_rate_des = PyKDL.Vector(0.0,0.0,trajAngularVel.point.z)

		angular_rate_error = angular_velocity - R.Inverse() * R_des * angular_rate_des

		angular_acceleration = PyKDL.Vector(angle_error[0]*Kr[0],angle_error[1]*Kr[1],angle_error[2]*Kr[2]) + PyKDL.Vector(angular_rate_error[0]*Ko[0],angular_rate_error[1]*Ko[1],angular_rate_error[2]*Ko[2]) + angular_velocity*angular_velocity
		
		#thrust = m*PyKDL.dot(lin_acc_kdl,R*PyKDL.Vector(0,0,1))


	
		u.effort = [angular_acceleration[0],angular_acceleration[1],angular_acceleration[2],-lin_acc_kdl[0],-lin_acc_kdl[1],-lin_acc_kdl[2]]

	#	u.effort = [0,0,angular_acceleration[2],thrust]
			
#		u.effort = [0,0,0,9.8]

#		u.effort = [0,0,0,-lin_acc_kdl[0],0,0]

	 	quad_input.publish(u)					

#		print body_F_des.point.z

		#print position,quaternion
	#	euler = euler_from_quaternion(quaternion)
		#print euler
		rate.sleep()


if __name__ == '__main__':
	try:
        	rospy.init_node('upenn_geometry_controller', anonymous=True)
        	quad_input = rospy.Publisher('/vrep/quadrotor_0/command', JointState, queue_size=10)

        	rospy.Subscriber('/vrep/quadpose', PoseStamped, PoseCallback)
        	rospy.Subscriber('/vrep/PathPointPose', PoseStamped, trajectoryPointPoseCallback)
		rospy.Subscriber('/vrep/PathPointTwist', TwistStamped, trajectoryPointTwistCallback)
		rospy.Subscriber('/vrep/quadtwist', TwistStamped, TwistCallback)
        	rospy.Subscriber('/closestPointPos', Point, closestPointPosCallback)
		rospy.Subscriber('/closestPointOri', Point, closestPointOriCallback)
        	rate = rospy.Rate(100) #10hz

        	init()
        	controller()

    	except rospy.ROSInterruptException:
        	pass

