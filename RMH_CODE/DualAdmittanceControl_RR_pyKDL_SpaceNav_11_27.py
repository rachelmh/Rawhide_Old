#! /usr/bin/env python

import rospy
import argparse
import numpy
#import intera_interface
import math
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time
# from intera_motion_interface import (
#     MotionTrajectory,
#     MotionWaypoint,
#     MotionWaypointOptions
# )


from RobotRaconteur.Client import *


class DualAdmittanceClass():


	def __init__(self):
		self.poirot=RRN.ConnectService('tcp://localhost:35147/SawyerRMHServer/Sawyer')
	
		print('poirot')
		self.captain=RRN.ConnectService('tcp://localhost:42413/SawyerRMHServer/Sawyer')
		
		print('captain')
		self.poirot.setControlMode(0)
		self.captain.setControlMode(0)


		# functions beginning with get_ are functions that should be called in the main program. gets a value of a function, regardless of what the function is
		#other functions can be edited without fear of messing up the overall structure. Similar to changing the definition of a function. as long as
		#the changed function returns the same number and types of variables, the get_ function should still work


		#initialize variables
		self.CalibratePoint = [0,0,0,0,0,0,0]
		self.StartingPoint=[0,0,.1,.1,.1,.1,.1]
		self.neutralposition=[0,-1.18,0,2.175,0,.566,0]
		self.neutralposition=[0,-1.18,0,2.175,0,-1,0]
		self.calibratedelay=5.0
		self.neutralflag=1

		self.returntoneutral=0
		self.Fxe_raw=0
		self.Fye_raw=0
		self.Fze_raw=0
		self.prevjointvelocities=numpy.array([0,0,0,0,0,0])
		self.rate = .05  # Hz


		#offsets for force sensor calibration
		self.offsetFxe=0
		self.offsetFye=0
		self.offsetFze=0

		self.PrevInvJac=numpy.zeros((6,6))

		self.EEx_lvec_p=[]
		self.EEy_lvec_p=[]
		self.EEz_lvec_p=[]
		self.EEx_avec_p=[]
		self.EEy_avec_p=[]
		self.EEz_avec_p=[]

		self.avgxl_p=0
		self.avgyl_p=0
		self.avgzl_p=0
		self.avgxa_p=0
		self.avgya_p=0
		self.avgza_p=0


		self.EEx_lvec_c=[]
		self.EEy_lvec_c=[]
		self.EEz_lvec_c=[]
		self.EEx_avec_c=[]
		self.EEy_avec_c=[]
		self.EEz_avec_c=[]

		self.avgxl_c=0
		self.avgyl_c=0
		self.avgzl_c=0
		self.avgxa_c=0
		self.avgya_c=0
		self.avgza_c=0


		self.Jxl_raw=0
		self.Jyl_raw=0
		self.Jzl_raw=0

		self.Jxa_raw=0
		self.Jya_raw=0
		self.Jza_raw=0

		self.leftbutton=0 #use left button to move last joint on robot arm
		self.rightbutton=0 #hold right button to reset to neutral position
        print("init")

	def get_current_joint_positions(self,whichsawyer):
		
		return whichsawyer.joint_positions

	def get_current_joint_velocity(self,whichsawyer):
		return whichsawyer.joint_velocities

	def get_current_joint_torques(self,whichsawyer):
		return whichsawyer.joint_torques

	def get_ee_pos(self,whichsawyer):
		return whichsawyer.endeffector_positions

	def get_ee_orientation(self,whichsawyer):
		return whichsawyer.endeffector_orientations
	
	def get_ee_twist(self,whichsawyer):
		return whichsawyer.endeffector_twists

	def get_ee_wrench(self,whichsawyer):
		return whichsawyer.endeffector_wrenches

	def get_ee_vel(self,whichsawyer):
		return whichsawyer.endeffector_velocity

	def Set_Position(self,whichsawyer,positions):
		#whichsawyer.setControlMode(0)
		pos=numpy.array(positions)
		curr_pos=numpy.array(self.get_current_joint_positions(whichsawyer))
		curr_ee_vel=numpy.array(self.get_ee_vel(whichsawyer))
		curr_max_vel=max(abs(numpy.array(self.get_current_joint_velocity(whichsawyer))))
		vel_threshold=0.001
		prev_max=0
		max_counter=0
		ctr_ctr=0
		whichsawyer.setControlMode(0)
		whichsawyer.setPositionModeSpeed(0.2)
		
			
		whichsawyer.setJointCommand('right', positions)
		while max_counter<100 :# and ctr_ctr<100:
			#print('looping')
			curr_pos=numpy.array(self.get_current_joint_positions(whichsawyer))
			curr_max=max(abs(curr_pos-pos))
			vel_max=max(abs(curr_ee_vel))
			#print('looping',curr_max,vel_max)
			#if vel_max < vel_threshold:
			# position threshold from sawyer settings 0.0087
			if (curr_max) < .008:
				if curr_max_vel < .002:
					max_counter=max_counter+1
			#prev_max_ctr=max_counter

			#if prev_max_ctr==max_counter:
				#ctr_ctr=ctr_ctr+1
			 
			

			print(curr_max,max_counter,ctr_ctr,curr_max_vel)
		print('break')

	def Set_NonBlocking_Position(self,whichsawyer,positions):
		#whichsawyer.setControlMode(0)

		whichsawyer.setControlMode(0)
		whichsawyer.setPositionModeSpeed(0.2)
		
			
		whichsawyer.setJointCommand('right', positions)
		



	def SpaceNavJoyCallback(self):
		#roslaunch spacenav_node classic.launch
		spacenavdata=self.poirot.SpaceNavigatorJoy
		
        #I think these have to be self. variables due to the nature of callback. data is only accessed in the callback
		
		self.Jxl_raw=spacenavdata[0]
		self.Jyl_raw=spacenavdata[1]
		self.Jzl_raw=spacenavdata[2]

		self.Jxa_raw=spacenavdata[3]
		self.Jya_raw=spacenavdata[4]
		self.Jza_raw=spacenavdata[5]
		self.leftbutton=spacenavdata[6]
		self.rightbutton=spacenavdata[7]


	def get_raw_joystick(self):
		#shouldn't have to touch!
		self.SpaceNavJoyCallback()
		(Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)= (self.Jxl_raw,self.Jyl_raw,self.Jzl_raw,self.Jxa_raw,self.Jya_raw,self.Jza_raw)#self.ForceSensorCallback()
		#print("Jxel, Jyel, Jzel,Jxea, Jyea, Jzea",Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)
		return (Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)

	def HereWeGo(self):
		#sets the offset values for the force sensor calibration

		if self.neutralflag==0:
			self.Set_Position(self.poirot,self.StartingPoint)
			self.Set_Position(self.captain,self.StartingPoint)
		else:
			self.Set_Position(self.poirot,self.neutralposition)
			self.Set_Position(self.captain,self.neutralposition)
			

		
		t1=time.time()
		while (time.time() - t1 < self.calibratedelay):
			time.sleep(0.001)

		self.poirot.setControlMode(1)
		self.captain.setControlMode(1)




	def get_inverse_Jacobian(self,whichsawyer):
		#shouldn't have to touch!
		invJac1=numpy.asarray(whichsawyer.pseudoinverse_Jacobian)
		#print(invJac1)
		invJac=invJac1.reshape((7,6))
		#print(invJac)
		return invJac

	def TransformJoystickToGlobalCoordinates(self):

		(Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)=self.get_raw_joystick()
		#temporary placeholder
		#(Fxe_t,Fye_t,Fze_t)=(Fxe_calib, Fye_calib, Fze_calib)
		#in 0 position (all joints at angle 0), convert Force sensor alignment to end effector CS
		#Xee=+yFS
		#Yee=-XFs
		#Zee=-ZFS

		(Jxel_t1, Jyel_t1, Jzel_t1,Jxea_t1, Jyea_t1, Jzea_t1)=(-Jxel, -Jyel, Jzel,-Jxea, -Jyea, Jzea)



		transformed1=numpy.array([[Jxel_t1], [Jyel_t1], [Jzel_t1],[Jxea_t1], [Jyea_t1], [Jzea_t1]])
		#now need to convert the forces at the end effetor to global coordinates
		
		#RotMat1=self.getRotationMatrix()

		#transformed2=numpy.matmul(RotMat1,transformed1)


		Jxel_tF=transformed1[0]
		Jyel_tF=transformed1[1]
		Jzel_tF=transformed1[2]
		Jxea_tF=transformed1[3]
		Jyea_tF=transformed1[4]
		Jzea_tF=transformed1[5]

		transformed_data=[Jxel_tF, Jyel_tF, Jzel_tF,Jxea_tF, Jyea_tF, Jzea_tF]

		#debugging

		#print("transformed1",transformed1)
		#print("transformed2",Fxe_tF,Fye_tF,Fze_tF)

		#---------
		return transformed_data

	def get_transformed_joystick(self):

		#shouldn't have to touch!
		(Jxel_t1, Jyel_t1, Jzel_t1,Jxea_t1, Jyea_t1, Jzea_t1)=self.TransformJoystickToGlobalCoordinates()
		transformed_data=[Jxel_t1, Jyel_t1, Jzel_t1,Jxea_t1, Jyea_t1, Jzea_t1]

		return transformed_data

	def JoystickAdmittanceScalingLaw(self):
		(Jxel_t1, Jyel_t1, Jzel_t1,Jxea_t1, Jyea_t1, Jzea_t1)=self.get_transformed_joystick()
		x_l_gain=1
		y_l_gain=1
		z_l_gain=1
		x_a_gain=1
		y_a_gain=1
		z_a_gain=1

		x_l_max=0.6
		x_l_min=0.3

		y_l_max=0.6
		y_l_min=0.3

		z_l_max=0.6
		z_l_min=0.3

		x_a_max=0.6
		x_a_min=0.3

		y_a_max=0.6
		y_a_min=0.3

		z_a_max=0.6
		z_a_min=0.3

		def setscalinglawvel(reading,gain,minthreshold,maxthreshold):
			ScalingLaw_vel=round(gain*reading,2)

			if ScalingLaw_vel <= minthreshold and ScalingLaw_vel >= -minthreshold:
				ScalingLaw_vel = 0

			if ScalingLaw_vel > maxthreshold:
				ScalingLaw_vel=maxthreshold

			if ScalingLaw_vel < -maxthreshold:
				ScalingLaw_vel=-maxthreshold

			return ScalingLaw_vel

		x_linear_vel=setscalinglawvel(Jxel_t1,x_l_gain,x_l_min,x_l_max)
		y_linear_vel=setscalinglawvel(Jyel_t1,y_l_gain,y_l_min,y_l_max)
		z_linear_vel=setscalinglawvel(Jzel_t1,y_l_gain,z_l_min,z_l_max)

		x_angular_vel=setscalinglawvel(Jxea_t1,x_a_gain,x_a_min,x_a_max)
		y_angular_vel=setscalinglawvel(Jyea_t1,y_a_gain,y_a_min,y_a_max)
		z_angular_vel=setscalinglawvel(Jzea_t1,z_a_gain,z_a_min,z_a_max)

		ScalingLaw_mat1=numpy.array([x_linear_vel,y_linear_vel,z_linear_vel, x_angular_vel,y_angular_vel,z_angular_vel])

		return ScalingLaw_mat1

	def get_WHmidpoint_velocity(self):
		#shouldn't have to touch!
		#end_effector_velocities1=self.AdmittanceScalingLaw()
		midpoint_velocities1=self.JoystickAdmittanceScalingLaw()
		#print("EEVel",end_effector_velocities1)

		return midpoint_velocities1

	def Joystick_to_robotEE(self):

		midpoint_velocities=self.get_WHmidpoint_velocity()
		#print('mV',midpoint_velocities)
		vspx=midpoint_velocities[0] -midpoint_velocities[5]
		vspy=midpoint_velocities[1]
		vspz=midpoint_velocities[2] +midpoint_velocities[3]
		#vspwx=
		#vspwy=
		#vspwz=


		vscx=midpoint_velocities[0] +midpoint_velocities[5]
		vscy=midpoint_velocities[1]
		vscz=midpoint_velocities[2]-midpoint_velocities[3]

		#vscwx=
		#vscwy=
		#vscwz=

		return [vspx, vspy, vspz, vscx, vscy, vscz]

	def get_admittance_control_end_effector_velocities(self):
		#shouldn't have to touch!
		#end_effector_velocities1=self.AdmittanceScalingLaw()
		end_effector_velocities1=self.Joystick_to_robotEE()
		#print("EEVel",end_effector_velocities1)

		return end_effector_velocities1

	def calculate_admittance_control_joint_velocities(self):
		jointthreshold=0.3

		Inv_Jac_poirot=self.get_inverse_Jacobian(self.poirot)
		Inv_Jac_captain=self.get_inverse_Jacobian(self.captain)

		end_effector_velocities_j=numpy.array(self.get_admittance_control_end_effector_velocities())

		end_effector_velocities_poirot=numpy.concatenate((end_effector_velocities_j[0:3], numpy.array([0,0,0])),axis=None)
		end_effector_velocities_captain=numpy.concatenate((end_effector_velocities_j[3:6], numpy.array([0,0,0])),axis=None)
		#print("EE1",end_effector_velocities)
		#Moving Average


		def mean(nums):
			return float(sum(nums))/max(len(nums),1)

		max_samples=5

		self.EEx_lvec_p.append(end_effector_velocities_poirot[0])
		self.EEy_lvec_p.append(end_effector_velocities_poirot[1])
		self.EEz_lvec_p.append(end_effector_velocities_poirot[2])
		self.EEx_avec_p.append(end_effector_velocities_poirot[3])
		self.EEy_avec_p.append(end_effector_velocities_poirot[4])
		self.EEz_avec_p.append(end_effector_velocities_poirot[5])

		self.EEx_lvec_c.append(end_effector_velocities_captain[0])
		self.EEy_lvec_c.append(end_effector_velocities_captain[1])
		self.EEz_lvec_c.append(end_effector_velocities_captain[2])
		self.EEx_avec_c.append(end_effector_velocities_captain[3])
		self.EEy_avec_c.append(end_effector_velocities_captain[4])
		self.EEz_avec_c.append(end_effector_velocities_captain[5])

		self.avgxl_p=mean(self.EEx_lvec_p)
		self.avgyl_p=mean(self.EEy_lvec_p)
		self.avgzl_p=mean(self.EEz_lvec_p)
		self.avgxa_p=mean(self.EEx_avec_p)
		self.avgya_p=mean(self.EEy_avec_p)
		self.avgza_p=mean(self.EEz_avec_p)

		self.avgxl_c=mean(self.EEx_lvec_c)
		self.avgyl_c=mean(self.EEy_lvec_c)
		self.avgzl_c=mean(self.EEz_lvec_c)
		self.avgxa_c=mean(self.EEx_avec_c)
		self.avgya_c=mean(self.EEy_avec_c)
		self.avgza_c=mean(self.EEz_avec_c)


		if len(self.EEx_lvec_p)==max_samples:
			self.EEx_lvec_p.pop(0)
		if len(self.EEy_lvec_p)==max_samples:
			self.EEy_lvec_p.pop(0)
		if len(self.EEz_lvec_p)==max_samples:
			self.EEz_lvec_p.pop(0)

		if len(self.EEx_avec_p)==max_samples:
			self.EEx_avec_p.pop(0)
		if len(self.EEy_avec_p)==max_samples:
			self.EEy_avec_p.pop(0)
		if len(self.EEz_avec_p)==max_samples:
			self.EEz_avec_p.pop(0)


		if len(self.EEx_lvec_c)==max_samples:
			self.EEx_lvec_c.pop(0)
		if len(self.EEy_lvec_c)==max_samples:
			self.EEy_lvec_c.pop(0)
		if len(self.EEz_lvec_c)==max_samples:
			self.EEz_lvec_c.pop(0)

		if len(self.EEx_avec_c)==max_samples:
			self.EEx_avec_c.pop(0)
		if len(self.EEy_avec_c)==max_samples:
			self.EEy_avec_c.pop(0)
		if len(self.EEz_avec_c)==max_samples:
			self.EEz_avec_c.pop(0)

		end_effector_velocities_avg_poirot=numpy.array([self.avgxl_p,self.avgyl_p, self.avgzl_p,self.avgxa_p,self.avgya_p, self.avgza_p])
		end_effector_velocities_avg_captain=numpy.array([self.avgxl_c,self.avgyl_c, self.avgzl_c,self.avgxa_c,self.avgya_c, self.avgza_c])
		#print("EE",end_effector_velocities_avg)



		admittance_control_joint_velocities1_poirot=numpy.matmul(Inv_Jac_poirot,end_effector_velocities_avg_poirot)
		admittance_control_joint_velocities1_captain=numpy.matmul(Inv_Jac_captain,end_effector_velocities_avg_captain)
		#print('advel',admittance_control_joint_velocities1)
		#print('advel',admittance_control_joint_velocities1)
		tempp=abs(numpy.matmul(Inv_Jac_poirot,end_effector_velocities_avg_poirot))
		tempc=abs(numpy.matmul(Inv_Jac_captain,end_effector_velocities_avg_captain))
		#print('Temp',temp)
		maxvalp=numpy.max(tempp)
		maxvalc=numpy.max(tempc)
		#print('max',maxval)

		if maxvalp==0:
			admittance_control_joint_velocities2_poirot=admittance_control_joint_velocities1_poirot
		else:

			admittance_control_joint_velocities2_poirot=numpy.divide(admittance_control_joint_velocities1_poirot, maxvalp)

		admittance_control_joint_velocities3_poirot=numpy.multiply(admittance_control_joint_velocities2_poirot,jointthreshold)

		admittance_control_joint_velocities4_poirot=numpy.around(admittance_control_joint_velocities3_poirot,decimals=2)


		if maxvalc==0:
			admittance_control_joint_velocities2_captain=admittance_control_joint_velocities1_captain
		else:

			admittance_control_joint_velocities2_captain=numpy.divide(admittance_control_joint_velocities1_captain, maxvalc)

		admittance_control_joint_velocities3_captain=numpy.multiply(admittance_control_joint_velocities2_captain,jointthreshold)

		admittance_control_joint_velocities4_captain=numpy.around(admittance_control_joint_velocities3_captain,decimals=2)
		






		return [admittance_control_joint_velocities4_poirot, admittance_control_joint_velocities4_captain]





	def get_admittance_control_joint_velocities(self):
		#shouldn't have to touch!
		admittance_control_joint_velocities2=self.calculate_admittance_control_joint_velocities()
		return admittance_control_joint_velocities2

	def set_admittance_control_joint_velocities(self):
		self.SpaceNavJoyCallback()
		if self.leftbutton==0:
			if self.rightbutton==0:
				all_vels=self.get_admittance_control_joint_velocities()
				
				admittance_control_joint_velocities_poirot=all_vels[0]
				
				admittance_control_joint_velocities_captain=all_vels[1]

				admittance_control_joint_velocities_poirot=numpy.around(admittance_control_joint_velocities_poirot,decimals=2)
				admittance_control_joint_velocities_captain=numpy.around(admittance_control_joint_velocities_captain,decimals=2)
				#print(admittance_control_joint_velocities_poirot)
				v0p=admittance_control_joint_velocities_poirot[0]
				v1p=admittance_control_joint_velocities_poirot[1]
				v2p=admittance_control_joint_velocities_poirot[2]
				v3p=admittance_control_joint_velocities_poirot[3]
				v4p=admittance_control_joint_velocities_poirot[4]
				v5p=admittance_control_joint_velocities_poirot[5]
				v6p=admittance_control_joint_velocities_poirot[6]

				v0c=admittance_control_joint_velocities_captain[0]
				v1c=admittance_control_joint_velocities_captain[1]
				v2c=admittance_control_joint_velocities_captain[2]
				v3c=admittance_control_joint_velocities_captain[3]
				v4c=admittance_control_joint_velocities_captain[4]
				v5c=admittance_control_joint_velocities_captain[5]
				v6c=admittance_control_joint_velocities_captain[6]

				print("joint velocities poirot: ", admittance_control_joint_velocities_poirot)
				print("joint velocities captain: ", admittance_control_joint_velocities_captain)
				#print("joint velocities: ", v0,v1,v2,v3,v4,v5,v6)\
				#print("jointangles",self.limb.joint_angles())
				#self.limb.set_joint_velocities({'right_j6':v6, 'right_j5': v5, 'right_j4': v4, 'right_j3': v3, 'right_j2': v2, 'right_j1': v1, 'right_j0': v0})
				self.poirot.setJointCommand('right', [v0p,v1p,v2p,v3p,v4p,v5p,v6p])
				self.captain.setJointCommand('right', [v0c,v1c,v2c,v3c,v4p,v5c,v6c])

			if self.rightbutton==1:
				v0p=0
				v1p=0
				v2p=0
				v3p=0
				v4p=0
				v5p=0
				v6p=0

				v0c=0
				v1c=0
				v2c=0
				v3c=0
				v4c=0
				v5c=0
				v6c=0
				self.poirot.setJointCommand('right', [v0p,v1p,v2p,v3p,v4p,v5p,v6p])
				self.captain.setJointCommand('right', [v0c,v1c,v2c,v3c,v4p,v5c,v6c])
				self.returntoneutral=self.returntoneutral+1
				print('ret to neutral',self.returntoneutral)

				if self.returntoneutral == 50:
					print('POIROT SET')
					self.Set_Position(self.poirot,self.neutralposition)
					print('CAPTAIN SET')
					self.Set_Position(self.captain,self.neutralposition)
					self.returntoneutral =0
					self.poirot.setControlMode(1)
					self.captain.setControlMode(1)

		else: 	
				# v0p=0
				# v1p=0
				# v2p=0
				# v3p=0
				# v4p=0
				# v5p=0
				# v6p=0

				# v0c=0
				# v1c=0
				# v2c=0
				# v3c=0
				# v4c=0
				# v5c=0
				# v6c=0
				# print('left button', self.leftbutton,'rightbutton',self.rightbutton)

				all_vels=self.get_admittance_control_joint_velocities()
				
				admittance_control_joint_velocities_poirot=all_vels[0]
				
				admittance_control_joint_velocities_captain=all_vels[1]

				admittance_control_joint_velocities_poirot=numpy.around(admittance_control_joint_velocities_poirot,decimals=2)
				admittance_control_joint_velocities_captain=numpy.around(admittance_control_joint_velocities_captain,decimals=2)
				#print(admittance_control_joint_velocities_poirot)
				v0p=admittance_control_joint_velocities_poirot[0]
				v1p=0#admittance_control_joint_velocities_poirot[1]
				v2p=0#admittance_control_joint_velocities_poirot[2]
				v3p=0#admittance_control_joint_velocities_poirot[3]
				v4p=0#admittance_control_joint_velocities_poirot[4]
				v5p=0#admittance_control_joint_velocities_poirot[5]
				v6p=0#admittance_control_joint_velocities_poirot[6]

				v0c=-1*admittance_control_joint_velocities_captain[0]
				v1c=0#admittance_control_joint_velocities_captain[1]
				v2c=0#admittance_control_joint_velocities_captain[2]
				v3c=0#admittance_control_joint_velocities_captain[3]
				v4c=0#admittance_control_joint_velocities_captain[4]
				v5c=0#admittance_control_joint_velocities_captain[5]
				v6c=0#admittance_control_joint_velocities_captain[6]

				print("joint velocities poirot: ", admittance_control_joint_velocities_poirot)
				print("joint velocities captain: ", admittance_control_joint_velocities_captain)
				#print("joint velocities: ", v0,v1,v2,v3,v4,v5,v6)\
				#print("jointangles",self.limb.joint_angles())
				#self.limb.set_joint_velocities({'right_j6':v6, 'right_j5': v5, 'right_j4': v4, 'right_j3': v3, 'right_j2': v2, 'right_j1': v1, 'right_j0': v0})
				self.poirot.setJointCommand('right', [v0p,v1p,v2p,v3p,v4p,v5p,v6p])
				self.captain.setJointCommand('right', [v0c,v1c,v2c,v3c,v4p,v5c,v6c])

		# else:
		# 		print("end rot only")
		# 		admittance_control_joint_velocities=self.get_admittance_control_joint_velocities()
		# 		admittance_control_joint_velocities=numpy.around(admittance_control_joint_velocities,decimals=2)
		# 		v0=0
		# 		v1=0
		# 		v2=0
		# 		v3=0#1*self.ScalingLaw_vel
		# 		v4=0
		# 		v5=admittance_control_joint_velocities[5]
		# 		v6=0
		# 		self.limb.set_joint_velocities({'right_j6':v6, 'right_j5': v5, 'right_j4': v4, 'right_j3': v3, 'right_j2': v2, 'right_j1': v1, 'right_j0': v0})






	def ROBOTCONTROLLER(self):
		print("Main")
		#self.CalibrateForceSensor()
		
		while not rospy.is_shutdown():
			self.set_admittance_control_joint_velocities()
			
			t1=time.time()
			

			while (abs(time.time() - t1) < self.rate):
				
				time.sleep(0.001)


def main():


	print("Initializing node... ")
	

	Controllerobj = DualAdmittanceClass()

	Controllerobj.HereWeGo()

	Controllerobj.ROBOTCONTROLLER()

	print("Done Calibrating")

if __name__ == '__main__':
	print('running main')
	main()


