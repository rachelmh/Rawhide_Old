
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
# def update_stl(joints):
#     for body in bodies:
#         #1. find pose of that body
#         #2. update pose of collision object

# def check_collision(env, robot):
#     update_stl(robot.joints)
#     return collision

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
        mesh_new.rotate([0.5, 0.0,0.0], euler[0])
        mesh_new.rotate([0.0, 0.5 ,0.0], euler[1])
        mesh_new.rotate([0.0, 0.0, 0.5], euler[2])
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
        obj_pose.pose.position.x=pos_orn[0]
        obj_pose.pose.position.y=pos_orn[1]
        obj_pose.pose.position.z=pos_orn[2]
        orns=tf.transformations.quaternion_from_euler(pos_orn[3],pos_orn[4],pos_orn[5])
        obj_pose.pose.orientation.x=orns[0]
        obj_pose.pose.orientation.y=orns[1]
        obj_pose.pose.orientation.z=orns[2]
        obj_pose.pose.orientation.w=orns[3]
        return obj_pose

    def RMHTransform(self,transformobject):

        self.collision_object.setTransform(transformobject)




def Rot(theta):
    return numpy.array([ [math.cos(theta), math.sin(theta),0], [math.sin(theta), math.cos(theta),0],[0,0,1]])

a0=.081
d0=.317
d1=.1925
d2=.4
d3=-.1685
d4=.4
d5=.1363
d6=.11

theta0=0
theta1=0
F=numpy.array([[0,0,0,1]])
R0_1=numpy.matmul(Rot(theta0), numpy.array([[1, 0, 0],[0,  0.0, 1], [0.0,  -1, 0]]) )
T0_1=numpy.array([[a0*math.cos(theta0)], [a0*math.sin(theta0)], [d0]])
H01=numpy.concatenate( (numpy.concatenate((R0_1,T0_1), axis=1), F), axis=0)

R1_2=numpy.matmul(Rot(theta1), numpy.array([[0, 0, 1],[-1,  0.0, 0], [0.0,  -1, 0]]) )
T1_2=numpy.array([[0], [0], [d1]])
H12=numpy.concatenate((numpy.concatenate((R1_2,T1_2), axis=1),F), axis=0)

H0_2=numpy.matmul(H01,H12)
R0_2=H0_2[0:3,0:3]
T0_2=H0_2[0:3,3]

# import pdb;
# pdb.set_trace()
#H01=numpy.concatenate((numpy.concatenate((R0_1,T0_1), axis=1),F), axis=0)


# import pdb;
# pdb.set_trace()


# R = numpy.array([[0, 0, 1],
#               [-1,  0.0, 0],
#               [0.0,  -1, 0]])
# T = numpy.array([0, 0, .1925])











pedestal_link=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/pedestal.STL")
pedestal_pose=pedestal_link.setPoseStamp("world",[.26,.345,-0.91488,-1.5708,0,1.5708]) #1.5708 #<origin rpy="1.5708 0 -1.5708" xyz="0.26 0.345 -0.91488"/>

right_arm_base_link=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/base.STL")
#base_pose=right_arm_base_link.setPoseStamp("world",[-0.0006241,-2.8025e-05,0.065404,0,0,0])  #<origin rpy="0 0 0" xyz="-0.0006241 -2.8025e-05 0.065404"/>
base_pose=right_arm_base_link.setPoseStamp("world",[0,0,0,0,0,0])  #<origin rpy="0 0 0" xyz="-0.0006241 -2.8025e-05 0.065404"/>



right_l0=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_mp3/l0.STL")
#l0_pose=right_l0.setPoseStamp("world",[0.024366,0.010969,0.14363,0,0,0]) #<origin rpy="0 0 0" xyz="0.024366 0.010969 0.14363"/>
#l0_pose=right_l0.setPoseStamp("world",[0,0,0,0,0,0]) #<origin rpy="0 0 0" xyz="0.024366 0.010969 0.14363"/>

transf_l0 = fcl.Transform(R0_1, T0_1)

head=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/head.STL")




right_l1=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_mp3/l1.STL")
transf_l1 = fcl.Transform(R0_2, T0_2)
right_l1.RMHTransform(transf_l1)


# import pdb;
# pdb.set_trace()
right_l2=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/l2.STL")
right_l3=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/l3.STL")
right_l4=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/l4.STL")
right_l5=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/l5.STL")
right_l6=Body("/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_mp1/l6.STL")





#1. build object
#table = Body('/home/rachel/mpalms/catkin_ws/src/config/descriptions/yumi/meshes/coarse/yumi_table.stl')
table=Body('/home/rachel/Desktop/RMH_CODE/cabinet_simple_1_30_2.STL')
#gripper = Body('/home/rachel/mpalms/catkin_ws/src/config/descriptions/yumi/meshes/coarse/palm_coarse.stl')
gripper=Body('/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/base.STL')
#pedestal=Body('/home/rachel/Desktop/RMH_CODE/pybullet_robots-master/data/sawyer_robot/sawyer_description/meshes/sawyer_pv/base.STL')




#PUT ROTATION AND TRANSLATION HERE
#2. set object pose pose
table_pose = PoseStamped()
table_pose.header.frame_id = "world"
table_pose.pose.position.x = 1
table_pose.pose.position.y = 0
table_pose.pose.position.z = 0
table_pose.pose.orientation.x = 0
table_pose.pose.orientation.y = 0
table_pose.pose.orientation.z = 0
table_pose.pose.orientation.w = 1

gripper_pose = PoseStamped()
gripper_pose.header.frame_id = "world"
gripper_pose.pose.position.x = 1
gripper_pose.pose.position.y = 0
gripper_pose.pose.position.z = .60
gripper_pose.pose.orientation.x = 0
gripper_pose.pose.orientation.y = 0
gripper_pose.pose.orientation.z = 0
gripper_pose.pose.orientation.w = 1

table.setCollisionPose(table.collision_object,
                       table_pose)

gripper.setCollisionPose(gripper.collision_object,
                         gripper_pose)

pedestal_link.setCollisionPose(pedestal_link.collision_object,pedestal_pose)
right_arm_base_link.setCollisionPose(right_arm_base_link.collision_object,base_pose)
#right_l0.setCollisionPose(right_l0.collision_object,l0_pose)
#plot mesh
pedestal_link.plot_meshes([ right_l0.mesh, right_l1.mesh]) #right_l0.mesh
#pedestal_link.plot_meshes([ pedestal_link.mesh, right_arm_base_link.mesh, right_l0.mesh, right_l1.mesh])
#check collision
print("collision: ", is_collision(table.collision_object, gripper.collision_object))

# import pdb;
# pdb.set_trace()

