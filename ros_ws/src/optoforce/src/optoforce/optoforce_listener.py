#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
 
def callback(data):
	print(data.wrench.force.x)
	#rospy.loginfo(  "I heard ", data.force[1][0])
    
def optoforcelistener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('optoforcelistener1', anonymous=True)

	rospy.Subscriber("optoforce_0", WrenchStamped, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	optoforcelistener()