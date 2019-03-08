#! /usr/bin/env python

#Pybullet test

import pybullet
import time
#pybullet.connect(pybullet.DIRECT)
pybullet.connect(pybullet.GUI)
pybullet.resetSimulation()
import pybullet_data
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = pybullet.loadURDF("plane.urdf",basePosition=[0,0,-1])
sawyer1=pybullet.loadURDF('/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/urdf/sawyer_rmh.urdf',basePosition=[0,-.35,0])
sawyer2=pybullet.loadURDF('/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/urdf/sawyer_rmh.urdf',basePosition=[0,.35,0])
cabinet=pybullet.loadURDF('/home/rachel/Desktop/RMH_CODE/cylinder_wire.urdf',basePosition=[1,0,0])
#cabinet=pybullet.loadURDF('/home/rachel/Desktop/RMH_CODE/cabinet_test.urdf',basePosition=[1,1,0])
# target1=[j[0] for j in pybullet.getJointStates(sawyer1, sj)]
targetpos1_1=[0,0,0,0,0,0,0]
targetpos1_2=[1.0, 0.1747805363098419, 0.012476163065545524, 0.012545413538967381, 0.008249133161252753, -0.008188858277728684, 0.0024590787083564893]
targetpos1_3=[1.0, 0.1747805363098419, 0.012476163065545524, 0.012545413538967381, 0.008249133161252753, -0.008188858277728684, 0.0024590787083564893]



targetpos2_1=[0,0,0,0,0,0,0]
targetpos2_2=[-1.0, 0.1, -0.0025443403082308523, 0.0031069771595375025, -0.009521088324357612, -0.0017516107829193654, 2.0782368786884374e-05]
targetpos2_3=[-1.0, -0.1, -0.0025443403082308523, 0.0031069771595375025, -0.009521088324357612, -0.0017516107829193654, 2.0782368786884374e-05]
target1=[targetpos1_1,targetpos1_2,targetpos1_3]
target2=[targetpos2_1, targetpos2_2, targetpos2_3]
sj=[3,8,9,10,11,13,16]
time.sleep(10)
# import pdb;
# pdb.set_trace()
for i in range(len(target2)):
    for j in range(len(target2[i])):


        pybullet.resetJointState(sawyer1,sj[j],target1[i][j])
        pybullet.resetJointState(sawyer2,sj[j],target2[i][j])
        pybullet.resetBasePositionAndOrientation(cabinet,[1,0,0],[0,0,0,1])
    time.sleep(5)
    print('reset')
    pybullet.setJointMotorControlArray(sawyer1,sj,pybullet.POSITION_CONTROL,targetPositions=target1[i])
    pybullet.setJointMotorControlArray(sawyer2,sj,pybullet.POSITION_CONTROL,targetPositions=target1[i])

    pybullet.stepSimulation()
    cp= pybullet.getContactPoints(sawyer1,sawyer2)
    cp2= pybullet.getContactPoints(sawyer1,cabinet)
    cp3= pybullet.getContactPoints(sawyer2,cabinet)

    if len(cp)>0:
        print('Collision!')
    else:
        print('no collision')

    if len(cp2)>0:
        print('Sawyer 1 Cabinet Collision!')
    else:
        print('no Sawyer 1 Cabinet collision')

    if len(cp3)>0:
        print('Sawyer 2 Cabinet Collision!')
    else:
        print('no Sawyer 2 Cabinet collision')

    #time.sleep(5)

pybullet.resetBasePositionAndOrientation(cabinet,[1,0,0],[0,0,0,1])




while 1:
    pass


# sw1_ee_pos=pybullet.getLinkState(sawyer1,16)[0]
# sw1_ee_quat=pybullet.getLinkState(sawyer1,16)[1]
# sw2_ee_pos=pybullet.getLinkState(sawyer2,16)[0]
# sw2_ee_quat=pybullet.getLinkState(sawyer2,16)[1]

#import tf
#quaternion_vec=tf.transformations.quaternion_from_euler(.1,.1,.1)
#import math
# cylx=(sw1_ee_pos[0]+sw2_ee_pos[0])/2
# cyly=(sw1_ee_pos[1]+sw2_ee_pos[1])/2
# cylz=(sw1_ee_pos[2]+sw2_ee_pos[2])/2
# d=math.sqrt( (sw1_ee_pos[0]sw2_ee_pos[0])**2  + (sw1_ee_pos[1]-sw2_ee_pos[1])**2 +(sw1_ee_pos[2]-sw2_ee_pos[2])**2)
# cyltx=asin((sw_ee_pos[2]-cylxz)/(d/2))
# cylty=0
# cyltz=asin((sw_ee_pos[0]-cylx)/(d/2))

# pos=[cylx, cyly, cylz]
# quat=tf.transformations.quaternion_from_euler(cyltx,cylty,cyltz)
#colcylId = pybullet.createCollisionShape(pybullet.GEOM_CYLINDER,halfExtents=[.5,.5,.5])
#sphereUid = pybullet.createMultiBody(0,colcylId,-1,basePosition=[1,1,1],baseOrientation=[.1,.1,.1,.1], useMaximalCoordinates=0)    #quaternion
#pybullet.removeBody(sphereUid)

#import tf
#quaternion_vec=tf.transformations.quaternion_from_euler(.1,.1,.1)

#pybullet.loadSDF('/home/rachel/Desktop/RMH_CODE/bullet3-master/data/kiva_shelf/model.sdf')

#You can create a simple URDF file with a visual or collision mesh that points to a .STL file.

#Alternatively, you can use the pybullet.createCollisionShape with GEOM_MESH type pointing to that STL file (or .OBJ). Then use the pybullet.createMultiBody to create an actual object. See the pybullet quickstart guide for details at http://pybullet.org