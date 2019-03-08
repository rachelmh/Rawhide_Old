#! /usr/bin/env python
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

def pose_stamped2list(msg):
    return [msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
            ]

def visualize_object(pose, filepath="package://config/descriptions/yumi/meshes/object.stl",name = "/object", color = (0., 0., 1., 1.), frame_id = "/yumi_base_link"):
    marker_pub = rospy.Publisher(name, Marker, queue_size=1)
    marker_type = Marker.MESH_RESOURCE

    marker = Marker()
    marker.header.frame_id  = frame_id
    marker.ns = name
    marker.header.stamp = rospy.Time(0)
    marker.action =  Marker.ADD
    marker.pose.orientation.w =  1.0
    marker.type = marker_type
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.lifetime.secs  = 1
    marker.pose = pose.pose
    marker.mesh_resource = filepath
    marker.lifetime = rospy.Duration(10000)
    for i in range(10):
        marker_pub.publish(marker)
        # rospy.sleep(.02)

def matrix_from_pose(pose):
    pose = pose_stamped2list(pose)
    trans = pose[0:3]
    quat = pose[3:7]
    T = tf.transformations.quaternion_matrix(quat)
    T[0:3,3] = trans
    return T

def initialize_collision_object(tris, verts):
    m = fcl.BVHModel()
    m.beginModel(len(verts), len(tris))
    m.addSubModel(verts, tris)
    m.endModel()
    t = fcl.Transform()
    return fcl.CollisionObject(m, t)

def is_collision(body1, body2):
    return fcl.collide(body1, body2, request, result)


class Body(object):
    def __init__(self, mesh_path):
        self.mesh = mesh.Mesh.from_file(mesh_path)
        self.trimesh = trimesh.load(mesh_path)
        self.faces = list(self.mesh.vectors)
        self.normals = list(self.mesh.normals)
        self.collision_object = initialize_collision_object(self.trimesh.faces, self.trimesh.vertices)

    def transform_object(self, pose, type="PoseStamped"):
        T = matrix_from_pose(pose)
        euler = tf.transformations.euler_from_matrix(T, 'sxyz')
        trans = tf.transformations.translation_from_matrix(T)
        mesh_new = deepcopy(self.mesh)
        # mesh_new.rotate([0.5, 0.0,0.0], euler[0])
        # mesh_new.rotate([0.0, 0.5 ,0.0], euler[1])
        # mesh_new.rotate([0.0, 0.0, 0.5], euler[2])
        mesh_new.rotate([1, 0.0,0.0], euler[0])
        mesh_new.rotate([0.0, 1 ,0.0], euler[1])
        mesh_new.rotate([0.0, 0.0, 1], euler[2])
        mesh_new.x += trans[0]
        mesh_new.y += trans[1]
        mesh_new.z += trans[2]
        return mesh_new

    def plot_meshes(self, mesh_list):
        # Optionally render the rotated cube faces
        from matplotlib import pyplot
        from mpl_toolkits import mplot3d
        # Create a new plot
        figure = pyplot.figure()
        axes = mplot3d.Axes3D(figure)
        # axes.set_xlim(2)
        # axes.set_ylim(2)
        # axes.set_zlim(2)
        # Render the cube
        for mesh in mesh_list:
            axes.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh.vectors))
            axes.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh.vectors))
        # Auto scale to the mesh size
        scale = mesh.points.flatten(-1)
        axes.auto_scale_xyz(scale, scale, scale)

        pyplot.xlabel('x')
        pyplot.ylabel('y')
        
        # Show the plot to the screen
        pyplot.show()

    def setCollisionPose(self, collision_object, pose_world):
        T_pose_world = matrix_from_pose(pose_world)
        R = T_pose_world[0:3, 0:3]
        t = T_pose_world[0:3, 3]
        collision_object.setRotation(R)
        collision_object.setTranslation(t)
        self.mesh = self.transform_object(pose_world)
        self.collision_object = collision_object


    def setPoseStamp(self,frame_id_name,pos_orn):
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = frame_id_name
        obj_pose.pose.position.x=pos_orn[0][0]
        obj_pose.pose.position.y=pos_orn[0][1]
        obj_pose.pose.position.z=pos_orn[0][2]
        #orns=tf.transformations.quaternion_from_euler(pos_orn[3],pos_orn[4],pos_orn[5])
        obj_pose.pose.orientation.x=pos_orn[1][0]
        obj_pose.pose.orientation.y=pos_orn[1][1]
        obj_pose.pose.orientation.z=pos_orn[1][2]
        obj_pose.pose.orientation.w=pos_orn[1][3]
        return obj_pose

    def RMHTransform(self,transformobject):

        self.collision_object.setTransform(transformobject)






#def CollisionChecker():

pub=rospy.Publisher('sawyer/joint_states', JointState, queue_size=10) 
rospy.init_node('MYjoint_state_publisher')
rate=rospy.Rate(100)

h=JointState()
h.header=Header()
h.name=['right_j01','head_pan1', 'right_j11', 'right_j21', 'right_j31', 'right_j41', 'right_j51', 'right_j61','right_j0', 'head_pan', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
h.position=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
h.velocity=[]
h.effort=[]

listener = tf.TransformListener()
pub.publish(h)




#pedestal_link=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/pedestal.STL")
pedestal_link=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_pv/pedestal.STL")
#pedetal_pos_orn=listener.lookupTransform('pedestal', 'world', rospy.Time(0))
pedetal_pos_orn=listener.lookupTransform('world','pedestal', rospy.Time(0))


pedestal_link2=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_pv/pedestal.STL")
#pedetal_pos_orn=listener.lookupTransform('pedestal', 'world', rospy.Time(0))
pedetal_pos_orn=listener.lookupTransform('world','pedestal', rospy.Time(0))
pedestal_pose_stamp2 = PoseStamped()
pedestal_pose_stamp2.header.frame_id = "world"
pedestal_pose_stamp2.pose.position.x = 0
pedestal_pose_stamp2.pose.position.y = 0
pedestal_pose_stamp2.pose.position.z = 0
pedestal_pose_stamp2.pose.orientation.x = .5
pedestal_pose_stamp2.pose.orientation.y = -.5
pedestal_pose_stamp2.pose.orientation.z = -.5
pedestal_pose_stamp2.pose.orientation.w = .5

pedestal_link2.mesh.rotate([1, 0.0,0.0], -1.57)
pedestal_link2.mesh.rotate([0.0, 1 ,0.0], 0)
pedestal_link2.mesh.rotate([0.0, 0.0, 1], 1.57)
pedestal_link2.mesh.x += .26
pedestal_link2.mesh.y += .345
pedestal_link2.mesh.z += 0
#pedestal_link2.setCollisionPose(pedestal_link2.collision_object, pedestal_pose_stamp2)




# import pdb;
# pdb.set_trace()
pedestal_pose_stamp=pedestal_link.setPoseStamp("world",pedetal_pos_orn) #([0.2886357976148024, 1.301784059511851, -1.015490950082993], [-0.5416766195837563, 0.45452185095921926, -0.5416726402219519, -0.4545185118693182])
#pedestal_link.mesh=pedestal_link.transform_object( pedestal_pose_stamp, type="PoseStamped")
print('pedestal',pedetal_pos_orn)
pedestal_link.setCollisionPose(pedestal_link.collision_object, pedestal_pose_stamp)

#right_arm_base_link=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/base.STL")
right_arm_base_link=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_pv/base.STL")
#right_arm_base_link_pos_orn=listener.lookupTransform('right_arm_base_link', 'world', rospy.Time(0))
right_arm_base_link_pos_orn=listener.lookupTransform('world','right_arm_base_link', rospy.Time(0))
right_arm_base_link_pose_stamp=pedestal_link.setPoseStamp("world",right_arm_base_link_pos_orn)
print('ra base link',right_arm_base_link_pos_orn)

#right_l0=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_mp3/l0.STL")
right_l0=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_mp3/l0.STL")
#right_l0_pos_orn=listener.lookupTransform('right_l0', 'world', rospy.Time(0))
right_l0_pos_orn=listener.lookupTransform('world','right_l0',  rospy.Time(0))
right_l0_pose_stamp=pedestal_link.setPoseStamp("world",right_l0_pos_orn)
print('rl0',right_l0_pos_orn)

#head=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/head.STL")
head=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_pv/head.STL")
head_pos_orn=listener.lookupTransform('world','head',  rospy.Time(0))
head_pose_stamp=pedestal_link.setPoseStamp("world",head_pos_orn)

#right_l1=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_mp3/l1.STL")
right_l1=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_mp3/l1.STL")
right_l1_pos_orn=listener.lookupTransform('world','right_l1',  rospy.Time(0))
right_l1_pose_stamp=pedestal_link.setPoseStamp("world",right_l1_pos_orn)
print('rl1',right_l1_pos_orn)




#right_l2=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/l2.STL")
right_l2=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_pv/l2.STL")
right_l2_pos_orn=listener.lookupTransform('world','right_l2',  rospy.Time(0))
right_l2_pose_stamp=pedestal_link.setPoseStamp("world",right_l2_pos_orn)

#right_l3=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/l3.STL")
right_l3=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_pv/l3.STL")
right_l3_pos_orn=listener.lookupTransform('world','right_l3',  rospy.Time(0))
right_l3_pose_stamp=pedestal_link.setPoseStamp("world",right_l3_pos_orn)

#right_l4=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/l4.STL")
right_l4=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_pv/l4.STL")
right_l4_pos_orn=listener.lookupTransform('world','right_l4',  rospy.Time(0))
right_l4_pose_stamp=pedestal_link.setPoseStamp("world",right_l4_pos_orn)

#right_l5=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/l5.STL")
right_l5=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_pv/l5.STL")
right_l5_pos_orn=listener.lookupTransform('world','right_l5',  rospy.Time(0))
right_l5_pose_stamp=pedestal_link.setPoseStamp("world",right_l5_pos_orn)

#right_l6=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_mp1/l6.STL")
right_l6=Body("/home/rachel/catkin_ws/src/RTKtest/sawyer_description/meshes/sawyer_mp1/l6.STL")
right_l6_pos_orn=listener.lookupTransform('world','right_l6',  rospy.Time(0))
right_l6_pose_stamp=pedestal_link.setPoseStamp("world",right_l6_pos_orn)


# import pdb;
# pdb.set_trace()



# table_pose = PoseStamped()
# table_pose.header.frame_id = "world"
# table_pose.pose.position.x = 1
# table_pose.pose.position.y = 0
# table_pose.pose.position.z = 0
# table_pose.pose.orientation.x = 0
# table_pose.pose.orientation.y = 0
# table_pose.pose.orientation.z = 0
# table_pose.pose.orientation.w = 1



right_arm_base_link.setCollisionPose(right_arm_base_link.collision_object,right_arm_base_link_pose_stamp)
head.setCollisionPose(head.collision_object,head_pose_stamp)
right_l0.setCollisionPose(right_l0.collision_object,right_l0_pose_stamp)
right_l1.setCollisionPose(right_l1.collision_object,right_l1_pose_stamp)
right_l2.setCollisionPose(right_l2.collision_object,right_l2_pose_stamp)
right_l3.setCollisionPose(right_l3.collision_object,right_l3_pose_stamp)
right_l4.setCollisionPose(right_l4.collision_object,right_l4_pose_stamp)
right_l5.setCollisionPose(right_l5.collision_object,right_l5_pose_stamp)
right_l6.setCollisionPose(right_l6.collision_object,right_l6_pose_stamp)

#pedestal_link.plot_meshes([ pedestal_link.mesh, right_arm_base_link.mesh,right_l0.mesh,head.mesh,right_l1.mesh,right_l2.mesh,right_l3.mesh,right_l4.mesh, right_l5.mesh, right_l6.mesh]) 
pedestal_link.plot_meshes([ pedestal_link.mesh,pedestal_link2.mesh])#,head.mesh,right_l1.mesh,right_l2.mesh,right_l3.mesh,right_l4.mesh, right_l5.mesh, right_l6.mesh]) 
#pedestal_link.plot_meshes([  right_arm_base_link.mesh]) 