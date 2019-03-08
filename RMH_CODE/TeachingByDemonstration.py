#! /usr/bin/env python

import rospy
import argparse
import numpy

import math
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

from RobotRaconteur.Client import *




class TeachingByDemonstration():


	def __init__(self):
		self.poirot=RRN.ConnectService('tcp://localhost:45285/SawyerRMHServer/Sawyer')
	
		print('poirot')
		#self.captain=RRN.ConnectService('tcp://localhost:42413/SawyerRMHServer/Sawyer')
		
		print('captain')
		self.poirot.setControlMode(0)
		#self.captain.setControlMode(0)
		self.currpos=self.poirot.joint_positions
		self.rate = .05  # Hz
		# functions beginning with get_ are functions that should be called in the main program. gets a value of a function, regardless of what the function is
		#other functions can be edited without fear of messing up the overall structure. Similar to changing the definition of a function. as long as
		#the changed function returns the same number and types of variables, the get_ function should still work
		self.countdown=50
		self.recordflag=0
		self.jointrecordedposition=[]
		self._done = False
		self.recrate=.005
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

	def Move_To_Position(self,whichsawyer,positions):
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
			print('looping',curr_max,vel_max)
			#if vel_max < vel_threshold:
			# position threshold from sawyer settings 0.0087
			if (curr_max) < .0087:
				if curr_max_vel < .08:
					max_counter=max_counter+1
			#prev_max_ctr=max_counter

			#if prev_max_ctr==max_counter:
				#ctr_ctr=ctr_ctr+1
			 
			

			print(curr_max,max_counter,ctr_ctr,curr_max_vel)
		print('break')

	def Set_Position(self,whichsawyer,positions):
		#whichsawyer.setControlMode(0)

		whichsawyer.setControlMode(0)
		whichsawyer.setPositionModeSpeed(10)			
		whichsawyer.setJointCommand('right', positions)

	def Set_Velocity(self,whichsawyer,velocities):
		#whichsawyer.setControlMode(0)

		whichsawyer.setControlMode(1)		
		whichsawyer.setJointCommand('right', velocities)

	def Set_Torques(self,whichsawyer,torques):
		#whichsawyer.setControlMode(0)

		whichsawyer.setControlMode(2)	
		whichsawyer.setJointCommand('right', torques)
		

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


	def get_raw_SpaceNav(self):
		#shouldn't have to touch!
		self.SpaceNavJoyCallback()
		(Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)= (self.Jxl_raw,self.Jyl_raw,self.Jzl_raw,self.Jxa_raw,self.Jya_raw,self.Jza_raw)#self.ForceSensorCallback()
		#print("Jxel, Jyel, Jzel,Jxea, Jyea, Jzea",Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)
		return (Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)


	def get_inverse_Jacobian(self,whichsawyer):
		#shouldn't have to touch!
		invJac1=numpy.asarray(whichsawyer.pseudoinverse_Jacobian)
		#print(invJac1)
		invJac=invJac1.reshape((7,6))
		#print(invJac)
		return invJac

	


	def time_stamp(self):
		ts=time.time() - self._start_time
		return ts

	def stop(self):
	
		self._done = True


	def record(self,filename):
		"""
		Records the current joint positions to a csv file if outputFilename was
		provided at construction this function will record the latest set of
		joint angles in a csv format.

		If a file exists, the function will overwrite existing file.
        """
		for i in xrange(5,0,-1):
			time.sleep(1)
			print(i)

		print('recording now!')
		#print(self._done, self.poirot.OkWheelButton)
		#time.sleep(3)
		self.currpos=self.poirot.joint_positions
		self.poirot.setControlMode(0)
		if filename:
		    
			with open(filename, 'w') as f:
				self._start_time=time.time()
				self.poirot.setControlMode(2)
				while not self._done:
					
					print('recording')
					temp_str =  '\n'
					angles_right = self.get_current_joint_positions(self.poirot)
					#print(angles_right)
					f.write("%f," % (self.time_stamp(),))
					f.write(','.join([str(x) for x in angles_right]) +  temp_str)
					# f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)
					if self.poirot.OkWheelButton==1:
						self._done=True

					time.sleep(self.recrate)
				self._done=False
				print("done!")
				self.currpos=self.poirot.joint_positions
				self.poirot.setControlMode(0)
				time.sleep(2)


	def ROBOTCONTROLLER(self):
		print("Main")
		# #self.CalibrateForceSensor()
		# prevwheel=0
		# while not rospy.is_shutdown():

		# 	curr_wheel=self.poirot.OkWheelButton

		# 	print(self.poirot.CuffButton,self.poirot.OkWheelButton)
		# 	if self.poirot.CuffButton==1:
		# 		self.currpos=self.poirot.joint_positions
		# 		#print("cuffbutton")
		# 	else:
		# 		self.poirot.setJointCommand('right',self.currpos)
		# 		#print("nocuffbutton")
			
		# 	t1=time.time()
			
		# 	# if self.poirot.navbutton ==1:
		# 	# 	print("Navbutton!")
		# 	# else:
		# 	# 	print("NoNavButton")

			
			

		# 	#curr_wheel=self.poirot.OkWheelButton

		# 	if prevwheel==0 and curr_wheel==1:
		# 		print("button pressed")
		# 	if prevwheel==1 and curr_wheel==0:
				#print("button released")

			# if self.poirot.OkWheelButton ==1 and self.recordflag==0:
			# 	self.countdown=self.countdown-1
			# 	print("countdown: ",self.countdown)


			# if self.poirot.OkWheelButton ==1 and self.recordflag==1:
			# 	self.countdown=50
			# 	self.recordflag=0

			# if self.countdown==0:
			# 	self.recordflag=1

			# if self.recordflag==1:

			# 	self.jointrecordedposition.append(self.get_current_joint_positions(self.poirot))

			# if self.recordflag==0 and len(self.jointrecordedposition) != 0:
			# 	print("Done Recording", len(self.jointrecordedposition))




			# prevwheel=curr_wheel

			# while (abs(time.time() - t1) < self.rate):
				
			# 	time.sleep(0.001)

def main():


	print("Initializing node... ")
	

	rob = TeachingByDemonstration()
	#recorder=JointRecorder('test',rob.rate)
	


	print("Main")
	#self.CalibrateForceSensor()
	prevwheel=0
	fileiter=0
	filename_list=[]
	playback_flag=0
	totallydoneflag=0
	while not rospy.is_shutdown():

		curr_wheel=rob.poirot.OkWheelButton

		print(rob.poirot.CuffButton,rob.poirot.OkWheelButton)
		if rob.poirot.CuffButton==1:
			rob.currpos=rob.poirot.joint_positions
			#print("cuffbutton")
		else:
			rob.poirot.setJointCommand('right',rob.currpos)
			#print("nocuffbutton")
		
		t1=time.time()
		
		# if self.poirot.navbutton ==1:
		# 	print("Navbutton!")
		# else:
		# 	print("NoNavButton")

		if playback_flag==0:
		
			recording_flag=0
			#curr_wheel=self.poirot.OkWheelButton
			print(prevwheel,curr_wheel,recording_flag)
			if prevwheel==0 and curr_wheel==1:
				print("button pressed")
				if recording_flag==0:
					recording_flag=1
				elif recording_flag==1:
					recording_flag==0

				if recording_flag==0:
					print('stopping recording')
					self._done=False #rob.stop()
					#print('stopping recording')
				elif recording_flag==1:
					print("starting recording")
					time.sleep(1)
					print("get ready")
					fileName = ("log_%d.txt" % (fileiter))

					rob.record(fileName)
					filename_list.append(fileName)
					fileiter=fileiter+1

					recording_flag=0
			if rob.poirot.XButton ==1:
				playback_flag=1
				print('playback!',playback_flag)

		if playback_flag==1:

			print('playback!',playback_flag)
			rob.poirot.setControlMode(0)

			if len(filename_list)==0:
				print('no recorded paths!')

			else:
				#import pdb;
				#pdb.set_trace()
				rob.recrate=.01
				print(filename_list)
				time.sleep(2)
				for fn in filename_list:
					raw_input("Press Enter to continue...")
					time.sleep(1)
					print(fn)
					with open(fn, 'r') as f:
						lines = f.readlines()
					jointgoal=[]
					currjointgoal=[]
					currtime=0

					startpos=numpy.array(lines[0].rstrip().split(','))
					startjointgoal=startpos.astype(numpy.float)
					print('moving to pos')
					rob.Move_To_Position(rob.poirot,startjointgoal[1:])
					
					
					for l in lines:
						currtime=time.time()
						nums=numpy.array(l.rstrip().split(','))
						
						currjointgoal=nums.astype(numpy.float)
						jointgoal.append(currjointgoal)
					
					time.sleep(0.5)
					rob.poirot.setControlMode(0)
					starttime=time.time()
					for goal in jointgoal:
						print('command',goal)
						#rob.Set_Position(rob.poirot,goal[1:])
						rob.poirot.setJointCommand('right', goal[1:])
						#while (rospy.get_time() - start_time) < values[0]:
						print('time',abs(time.time() - starttime), goal[0])
						while (abs(time.time() - starttime) <goal[0]):
							
							time.sleep(0.001)
						#time.sleep(.01)
					playback_flag=0
					totallydoneflag=1
					#print(jointgoal)

		# elif prevwheel==1 and curr_wheel==0:
		# 	print("button released")

		if totallydoneflag==1:
			rob.poirot.setControlMode(0)
			print("totally Done!")
			while not rospy.is_shutdown():
			
				time.sleep(0.1)

		prevwheel=curr_wheel

		while (abs(time.time() - t1) <rob.rate):
			
			time.sleep(0.001)








	#rob.ROBOTCONTROLLER()


	#rospy.on_shutdown(recorder.stop)

	print("Done Calibrating")

if __name__ == '__main__':
	print('running main')
	main()


