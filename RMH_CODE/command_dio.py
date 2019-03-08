#! /usr/bin/env python

import rospy

from intera_core_msgs.msg import IOComponentCommand

#rostopic pub -r 100 /io/comms/io/command intera_core_msgs/IOComponentCommand '{  time : now,  op : "set", args : "{ \"signals\" : { \"'port_sink_0'\" : { \"format\" : {  \"type\" : \"'int'\",  \"dimensions\" : [ '1' ] }, \"data\" : [ '1' ] } } }" }'
pub=rospy.Publisher('/io/comms/io/command', IOComponentCommand, queue_size=10) 
rospy.init_node('my_iocommand_publisher')
rate=rospy.Rate(100)

h=IOComponentCommand()
h.time=rospy.Time.now()
h.op="set"
name='port_sink_0'
cmd=1
#h.args= "{ \"signals\" : { \"port_sink_0\" : { \"format\" : {  \"type\" : \"int\",  \"dimensions\" : [ 1] }, \"data\" : [ 1 ] } } }" 
#h.args= "{ \"signals\" : { \"{}\" : { \"format\" : {  \"type\" : \"int\",  \"dimensions\" : [ 1] }, \"data\" : [ {} ] } } }".format(name,cmd)
h.args= "{ \"signals\" : { \"%s\" : { \"format\" : {  \"type\" : \"int\",  \"dimensions\" : [ 1] }, \"data\" : [ %d ] } } }"  %(name, cmd)
while not rospy.is_shutdown():
 	h.time=rospy.Time.now()
 	print('pub')
	pub.publish(h)
	rate.sleep()