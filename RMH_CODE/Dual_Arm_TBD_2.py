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

class RobotArm_TBD():


	def __init__(self, filename):

		#self.robot=RRN.ConnectService('tcp://localhost:45179/SawyerRMHServer/Sawyer')
		self.robot=RRN.ConnectService(filename)
		#self.captain=RRN.ConnectService('tcp://localhost:42413/SawyerRMHServer/Sawyer')
		
		self.robot.controlmode=0
		self.robot.setControlMode(self.robot.controlmode)
		#self.captain.setControlMode(0)
		self.currpos=self.robot.joint_positions
		self.rate = .05  # Hz
		self.recrate=.01
		# functions beginning with get_ are functions that should be called in the main program. gets a value of a function, regardless of what the function is
		#other functions can be edited without fear of messing up the overall structure. Similar to changing the definition of a function. as long as
		#the changed function returns the same number and types of variables, the get_ function should still work
		self.countdown=50
		self.recordflag=0
		self.jointrecordedposition=[]
		self._done = False
		self.doneflag=False
		


	def get_current_joint_positions(self):
		
		return self.robot.joint_positions

	def get_current_joint_velocity(self):
		return self.robot.joint_velocities

	def get_current_joint_torques(self):
		return self.robot.joint_torques

	def get_ee_pos(self):
		return self.robot.endeffector_positions

	def get_ee_orientation(self):
		return self.robot.endeffector_orientations
	
	def get_ee_twist(self):
		return self.robot.endeffector_twists

	def get_ee_wrench(self):
		return self.robot.endeffector_wrenches

	def get_ee_vel(self):
		return self.robot.endeffector_velocity
	def test(self):
		return self.get_current_joint_positions()

	def Move_To_Position(self,positions):
		#whichsawyer.setControlMode(0)
		pos=numpy.array(positions)
		curr_pos=numpy.array(self.get_current_joint_positions())
		curr_ee_vel=numpy.array(self.get_ee_vel())
		curr_max_vel=max(abs(numpy.array(self.get_current_joint_velocity())))
		vel_threshold=0.001
		prev_max=0
		max_counter=0
		ctr_ctr=0
		if self.robot.controlmode != 0:
			self.SetControlMode(0)
			self.SetPositionModeSpeed(0.2)
		
			
		self.Set_Position( positions)
		while max_counter<100 :# and ctr_ctr<100:
			#print('looping')
			curr_pos=numpy.array(self.get_current_joint_positions())
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


	def Set_Position(self,positions,speed):
		#whichsawyer.setControlMode(0)
		if self.robot.controlmode != 0:
			print('change')
			self.robot.setControlMode(0)
		self.robot.setPositionModeSpeed(speed)			
		self.robot.setJointCommand('right', positions)

	def Set_Velocity(self,velocities):
		#whichsawyer.setControlMode(0)
		if self.robot.controlmode != 1:
			self.robot.setControlMode(1)
				
		self.robot.setJointCommand('right', velocities)

	def Set_Torques(self,torques):
		#whichsawyer.setControlMode(0)

		if self.robot.controlmode != 2:
			self.robot.setControlMode(2)	
		self.robot.setJointCommand('right', torques)

	def SpaceNavJoyCallback(self):
		#roslaunch spacenav_node classic.launch
		spacenavdata=self.robot.SpaceNavigatorJoy
		
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
		self.robot.SpaceNavJoyCallback()
		(Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)= (self.Jxl_raw,self.Jyl_raw,self.Jzl_raw,self.Jxa_raw,self.Jya_raw,self.Jza_raw)#self.ForceSensorCallback()
		#print("Jxel, Jyel, Jzel,Jxea, Jyea, Jzea",Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)
		return (Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)

	def get_inverse_Jacobian(self):
		#shouldn't have to touch!
		invJac1=numpy.asarray(self.robot.pseudoinverse_Jacobian)
		#print(invJac1)
		invJac=invJac1.reshape((7,6))
		#print(invJac)
		return invJac

	def SetControlMode(self,Mode):
		self.robot.controlmode=Mode
		self.robot.setControlMode(Mode)

	def SetPositionModeSpeed(self,speed):
		self.robot.setPositionModeSpeed(speed)
		





def DA_Move_To_Position(robot1,robot2,positions1,positions2):
	#whichsawyer.setControlMode(0)
	despos1=numpy.array(positions1)
	despos2=numpy.array(positions2)
	curr_pos1=numpy.array(robot1.get_current_joint_positions())
	curr_pos2=numpy.array(robot2.get_current_joint_positions())
	curr_ee_vel1=numpy.array(robot1.get_ee_vel())
	curr_ee_vel2=numpy.array(robot2.get_ee_vel())
	curr_max_vel1=max(abs(numpy.array(robot1.get_current_joint_velocity())))
	curr_max_vel2=max(abs(numpy.array(robot2.get_current_joint_velocity())))

	vel_threshold=0.001
	prev_max=0
	max_counter=0
	ctr_ctr=0


	robot1.SetControlMode(0)
	robot2.SetControlMode(0)
	
		
	robot1.Set_Position(positions1,.2)
	robot2.Set_Position(positions2,.2)
	start=time.time()
	while max_counter<100 :# and ctr_ctr<100:
		#print('looping')

		curr_pos1=numpy.array(robot1.get_current_joint_positions())
		curr_pos2=numpy.array(robot2.get_current_joint_positions())
		curr_ee_vel1=numpy.array(robot1.get_ee_vel())
		curr_ee_vel2=numpy.array(robot2.get_ee_vel())


		#curr_pos1=numpy.array(self.get_current_joint_positions(self.robot))
		curr_max1=max(abs(curr_pos1-despos1))
		curr_max2=max(abs(curr_pos2-despos2))
		vel_max1=max(abs(curr_ee_vel1))
		vel_max2=max(abs(curr_ee_vel2))
		print('looping',time.time()-start,max_counter,curr_max1,vel_max1, curr_max2, vel_max2)
		#if vel_max < vel_threshold:
		# position threshold from sawyer settings 0.0087
		# if (curr_max1) < .0087 and (curr_max1) < .0087 :
		if (curr_max1) < .01 and (curr_max2) < .01 :
			if curr_max_vel1 < .08 and curr_max_vel2 < .08:
				max_counter=max_counter+1

		

		if time.time()-start > 7  and (curr_max1) < .02 and (curr_max2) < .02 and curr_max_vel1 < .08 and curr_max_vel2 < .08:
			max_counter=100
			print('timeout')
		

def time_stamp(start_time):
	ts=time.time() - start_time
	return ts



def record(robot1,robot2,filename):
	"""
	Records the current joint positions to a csv file if outputFilename was
	provided at construction this function will record the latest set of
	joint angles in a csv format.

	If a file exists, the function will overwrite existing file.
    """

	recrate=.003

	for i in xrange(3,0,-1):
		time.sleep(1)
		print(i)

	print('recording now!')
	#print(self._done, self.poirot.OkWheelButton)
	#time.sleep(3)
	robot1.currpos=robot1.get_current_joint_positions()
	robot2.currpose=robot2.get_current_joint_positions()
	robot1.SetControlMode(0)
	robot2.SetControlMode(0)
	robot1.doneflag=False
	if filename:
	    
		with open(filename, 'w') as f:
			start_time=time.time()
			#robot1.SetControlMode(0)
			robot1.SetControlMode(2)
			robot2.SetControlMode(2)
			

			while not robot1.doneflag:
				robot1.setDisplayImage('/home/rachel/Desktop/RMH_CODE/TBD_images/stoprecord.png')
				
				#robot1.Set_Torques([0,0,0,0,0,0,0])
				#robot2.Set_Torques([0,0,0,0,0,0,0])
				print('recording',robot1.robot.controlmode)
				temp_str =  '\n'
				angles_rob1 = robot1.get_current_joint_positions()
				angles_rob2 = robot2.get_current_joint_positions()
				angles_right=numpy.concatenate((angles_rob1, angles_rob2))
				#print(angles_right)
				f.write("%f," % (time_stamp(start_time),))
				f.write(','.join([str(x) for x in angles_right]) +  temp_str)
				# f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)
				if robot1.robot.OkWheelButton==1:
					robot1.doneflag=True

				time.sleep(recrate)



			robot1.doneflag=False
			print("done!")
			robot1.currpos=robot1.get_current_joint_positions()
			robot2.currpos=robot2.get_current_joint_positions()

		# print('setting control mode to 0')
			robot1.SetControlMode(0)
			robot2.SetControlMode(0)
		# robot1.Set_Position(robot1.currpos,.2)
		# robot2.Set_Position(robot2.currpos,.2)
		#import pdb;
		#pdb.set_trace()
			time.sleep(2)
	else:
		print("error with filename")


def playback(robot1,robot2,filename_list):
	for fn in filename_list:
		tempflag=1
		print('press ok wheel to continue')
		while tempflag:
			if robot1.robot.OkWheelButton==1:
					rtempflag=0


		#raw_input("Press Enter to continue...")
		time.sleep(1)
		print(fn)
		with open(fn, 'r') as f:
			lines = f.readlines()
		jointgoal=[]
		currjointgoal=[]
		currtime=0
		#lines[-1] to get timestamp
		startpos=numpy.array(lines[0].rstrip().split(','))
		startjointgoal=startpos.astype(numpy.float)
		print('moving to starting pos')
		DA_Move_To_Position(robot1,robot2,startjointgoal[1:8],startjointgoal[8:15])
		
		#for rev playback,extract timestamp of reversed(lines)[0], this is offset time. 
		#so then in the forloop, need to edit currjointgoal[0] = abs(currjointgoal[0]-offsettime).
		#rev playback will be to go from A2 to A, if big changes. 
		#should see how it goes with small changes, just for demo.
		#using collision checking + discrete 0.5 s waypoints may also help. cause then you can just change waypoints
		#but this is an issue for after the demo, pending nothing big goes wrong with the simplest implementation


		for l in lines:  #in reversed(lines) for reverse playback?
			currtime=time.time()
			nums=numpy.array(l.rstrip().split(','))
			
			currjointgoal=nums.astype(numpy.float)
			jointgoal.append(currjointgoal)
		
		time.sleep(0.5)
		robot1.robot.setControlMode(0)
		robot2.robot.setControlMode(0)
		starttime=time.time()
		for goal in jointgoal:
			#print('command',goal)
			#rob.Set_Position(rob.poirot,goal[1:])
			robot1.robot.setJointCommand( 'right',goal[1:8])
			robot2.robot.setJointCommand('right', goal[8:15])
			#while (rospy.get_time() - start_time) < values[0]:
			#print('time',abs(time.time() - starttime), goal[0])
			while (abs(time.time() - starttime) <goal[0]):
				
				time.sleep(0.001)
							#time.sleep(.01)
	robot1.currpos=robot1.get_current_joint_positions()			
	robot2.currpos=robot2.get_current_joint_positions()	
	robot1.robot.setJointCommand('right',robot1.currpos)
	robot2.robot.setJointCommand('right',robot2.currpos)

def main():

	#NEUTRAL POSITION {'right_j6': 3.3119794921875, 'right_j5': 0.566505859375, 'right_j4': 0.0049091796875, 'right_j3': 2.177044921875, 'right_j2': -0.00241796875, 'right_j1': -1.178189453125, 'right_j0': 0.001283203125}
	print("Lets get it on")
	poirot=RobotArm_TBD('tcp://localhost:41883/SawyerRMHServer/Sawyer')
	captain=RobotArm_TBD('tcp://localhost:35429/SawyerRMHServer/Sawyer')

	prevwheel=0
	fileiter=0
	filename_list=[]
	playback_flag=0
	totallydoneflag=0
	#DA_Move_To_Position(poirot,captain,[0,.1,.1,0,0,0,0],[0,.1,.1,0,0,0,0])
	DA_Move_To_Position(poirot,captain,[0, -1.18, 0, 2.2, 0, -.5, 3.3],[0, -1.18, 0, 2.2, 0,-.5,3.3])
	time.sleep(1)
	
	poirot.currpos=poirot.get_current_joint_positions()
	captain.currpos=captain.get_current_joint_positions()

	while not rospy.is_shutdown():

		curr_wheel=poirot.robot.OkWheelButton

		#print(poirot.robot.CuffButton,poirot.robot.OkWheelButton)

		if poirot.robot.CuffButton==1:
			poirot.currpos=poirot.get_current_joint_positions()
			captain.currpos=captain.get_current_joint_positions()
			print("cuffbutton")
		else:
			poirot.Set_Position(poirot.currpos,.2)
			captain.Set_Position(captain.currpos,.2)
			print("nocuffbutton")
		
		t1=time.time()

		print("currwhl:", curr_wheel,"prevwhl: ", prevwheel,"cuff:",poirot.robot.CuffButton,"OKWheel:",poirot.robot.OkWheelButton,'playback_flag',playback_flag,"controlmode",poirot.robot.controlmode)

		if playback_flag==0:
		
			recording_flag=0
			curr_wheel=poirot.robot.OkWheelButton
			poirot.setDisplayImage('/home/rachel/Desktop/RMH_CODE/TBD_images/torecord_toplayback.png')

			#print(prevwheel,curr_wheel,recording_flag)
			if prevwheel==0 and curr_wheel==1:
				print("button pressed")
				if recording_flag==0:
					recording_flag=1
				elif recording_flag==1:
					recording_flag==0

				if recording_flag==0:
					print('stopping recording')
					#poirot.doneflag=False #rob.stop()
					#print('stopping recording')
				elif recording_flag==1:
					print("starting recording")
					time.sleep(1)
					print("get ready")
					fileName = ("log_%d.txt" % (fileiter))

					record(poirot,captain,fileName)
					poirot.SetControlMode(0)
					captain.SetControlMode(0)
					#import pdb;
					#pdb.set_trace()
					poirot.currpos=poirot.get_current_joint_positions()
					captain.currpos=captain.get_current_joint_positions()
					filename_list.append(fileName)
					fileiter=fileiter+1

					recording_flag=0
			if poirot.robot.XButton ==1:
				playback_flag=1
				print('playback!',playback_flag)

		elif playback_flag==1:

			print('playback!',playback_flag)
			poirot.SetControlMode(0)
			captain.SetControlMode(0)
			if len(filename_list)==0:
				print('no recorded paths!')

			else:
				#import pdb;
				#pdb.set_trace()
				#poirot.recrate=.01
				print(filename_list)
				time.sleep(2)
				playback(poirot,captain,filename_list)

				playback_flag=0
				totallydoneflag=1


		if totallydoneflag==1:
			poirot.SetControlMode(0)
			captain.SetControlMode(0)
			print("totally Done!")

			while not rospy.is_shutdown():
			
				time.sleep(0.1)

		prevwheel=curr_wheel

		while (abs(time.time() - t1) <poirot.rate):
			
			time.sleep(0.001)


	RRN.disconnect(poirot)
	RRN.disconnect(captain)

if __name__ == '__main__':
	print('running main')
	main()