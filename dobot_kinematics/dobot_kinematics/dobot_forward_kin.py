from PyKDL import Chain, Vector, Rotation, Frame, Joint, Segment, JntArray, ChainFkSolverPos_recursive
from math import radians

def calc_FwdKin(theta1, theta2, theta3):

	# Kinematic chain

	chain = Chain()
	base_link__link_1 = Joint(Joint.RotZ) 
	frame1 = Frame(Rotation.RPY(0, 0, 0),
		Vector(0, 0, 0)) 
	segment1 = Segment(base_link__link_1, frame1)
	chain.addSegment(segment1) 


	link_1__link_2 = Joint(Joint.RotY) 
	frame2 = Frame(Rotation.RPY(0, 0, 0),
		Vector(0, 0, 0.135))
	segment2=Segment(link_1__link_2, frame2)
	chain.addSegment(segment2)


	link_2__link_3 = Joint(Joint.RotY) 
	frame3 = Frame(Rotation.RPY(0, 0, 0),
		Vector(0.147, 0, 0))
	segment3=Segment(link_2__link_3,frame3)
	chain.addSegment(segment3)


	# Forward kinematics
	joint_angles=JntArray(3)
	joint_angles[0]= radians(theta1)
	joint_angles[1]= radians(theta2)
	joint_angles[2]= -(radians(theta2) - radians(theta3))


	# Recursive solver of simple kinematics

	fk=ChainFkSolverPos_recursive(chain)
	finalFrame=Frame()
	fk.JntToCart(joint_angles,finalFrame)
	xyz = finalFrame.p 

	return xyz*1000

