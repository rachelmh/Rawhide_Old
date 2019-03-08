from stl import mesh
import os
import tf
import trimesh
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
import fcl
import numpy
request = fcl.CollisionRequest()
result = fcl.CollisionResult()
from tf.transformations import quaternion_from_euler
import math
import rospy

import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

#pub=rospy.Publisher('joint_states', JointState, queue_size=10) 
pub=rospy.Publisher('sawyer/joint_states', JointState, queue_size=10) 
rospy.init_node('MYjoint_state_publisher')
rate=rospy.Rate(100)

h=JointState()
h.header=Header()
h.name=['right_j01','head_pan1', 'right_j11', 'right_j21', 'right_j31', 'right_j41', 'right_j51', 'right_j61','right_j0', 'head_pan', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
h.position=[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
h.velocity=[]
h.effort=[]


listener = tf.TransformListener()
#(trans,rot)=llistener.lookupTransform('right_l1','right_l6',rospy.Time(0))


#NEED TO SET UP A JOINTSTATE PUBLISHER, Otherwise trees won't work
#listener.lookupTransform('base', 'base1', rospy.Time(0))

#print(trans,rot)

#1. Set up jointstate publisher



pub.publish(h)
time.sleep(5)
while not rospy.is_shutdown():
 	h.header.stamp=rospy.Time.now()
 	print('pub')
	pub.publish(h)
	rate.sleep()

#time.sleep(5)


#<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" ></node>


