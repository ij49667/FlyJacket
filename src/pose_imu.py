#!/usr/bin/env python

import rospy
import std_msgs
from math import atan2, asin, pi, cos, sin, sqrt, copysign
from numpy import sign, array, matmul, absolute
from numpy.linalg import inv, norm
from sensor_msgs.msg import Imu
import numpy as np
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, PoseStamped
import geometry_msgs.msg
import tf2_ros
from tf.transformations import euler_from_quaternion



global q
global R_elbow_left
global R_neck
lu = 0.30 #upper_arm
lf = 0.23 #forward_arm
l9 = 0.15 #clavicle
l3 = 0.2 
l2 = 0.25
l1 = 1.0

x0 = y0 = z0  = 0.0
x2 = y2 = z2 = y3 = z3 =  0.0
pitch50 = roll50 = yaw50 = 0.0
pitch51 = roll51 = yaw51 = 0.0
pitch52 = roll52 = yaw52 = 0.0
pitch53 = roll53 = yaw53 = 0.0
pitch54 = roll54 = yaw54 = 0.0
pitch55 = roll55 = yaw55 = 0.0
pitch56 = roll56 = yaw56 = 0.0
pitch57 = roll57 = yaw57 = 0.0

q=Quaternion(0,0,0,0)
R_elbow_left =R_back= R_shoulder_right= R_elbow=R_shoulder_left=R_neck= [[1,0,0],[0,1,0],[0,0,1]]


#quaternion rotation matrix  
def quaternion_rotation_matrix(q0,q1,q2,q3):
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

#neck pose
def neck_pose(roll56, pitch56, yaw56):
    x6 = l3*(cos(yaw57)*sin(pitch57)*cos(yaw56)+sin(yaw57)*sin(yaw56))+l2*cos(yaw57)*sin(pitch57) #x
    z6 = l3*(sin(yaw57)*sin(pitch57)*cos(yaw56)-cos(yaw57)*sin(yaw56))+l2*sin(yaw57)*sin(pitch57) #z
    y6 = l3*cos(pitch57)*cos(yaw57)+l2*cos(pitch57) #y
    return x6,y6,z6

#angles roll,pitch,yaw (radians)
def ang(q0,q1,q2,q3):
    
    t0 = +2.0 * (q3 * q0 + q1 * q2)
    t1 = +1.0 - 2.0 * (q0 * q0 + q1 * q1)
    roll = atan2(t0, t1)  
    t2 = +2.0 * (q3 * q1 - q2 * q0)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)
    t3 = +2.0 * (q3 * q2 + q0 * q1)
    t4 = +1.0 - 2.0 * (q1 * q1 + q2 * q2)
    yaw = atan2(t3, t4)
    return roll, pitch, yaw 
#IMU50
def callback50(data):
    global pitch50,roll50,yaw50
    global R_shoulder_right
    
    shoulder_right = Pose()
    shoulder_right.orientation = data.orientation
    shoulder_right.position.x = l9
    shoulder_right.position.y = 0.0
    shoulder_right.position.z = 0.0

    #angles
    (roll50,pitch50,yaw50)=ang(shoulder_right.orientation.x,shoulder_right.orientation.y,shoulder_right.orientation.z, shoulder_right.orientation.w)

    #rotation matrix
    R_shoulder_right = quaternion_rotation_matrix(shoulder_right.orientation.w,shoulder_right.orientation.x,shoulder_right.orientation.y, shoulder_right.orientation.z)
    R_neck_inv = inv(R_neck)
    R_trazeno = matmul(R_neck_inv,R_shoulder_right,)
    #rotation matrix to quaternion
    trace = R_trazeno[0,0] + R_trazeno[1,1]+ R_trazeno[2,2]
    r = sqrt(1+trace)
    q.w = 0.5*r
    q.x = copysign(0.5*sqrt(absolute(1+R_trazeno[0,0] - R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[2,1]-R_trazeno[1,2]))
    q.y = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] + R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[0,2]-R_trazeno[2,0]))
    q.z = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] - R_trazeno[1,1]+ R_trazeno[2,2])),(R_trazeno[1,0]-R_trazeno[0,1]))
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    print q, norm
    #transform
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/neck"
    t.child_frame_id = "/shoulder_right"
    
    t.transform.translation.x = l9 
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
   
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    pub50.publish(shoulder_right)

#IMU51
def callback51(data):
    global roll51, pitch51,yaw51
    global R_wrist_left
    wrist_left = Pose()
    wrist_left.orientation  = data.orientation
    
    (roll51,pitch51,yaw51)=ang(wrist_left.orientation.x, wrist_left.orientation.y,wrist_left.orientation.z, wrist_left.orientation.w)
    
   
    R_wrist_left = quaternion_rotation_matrix(wrist_left.orientation.w,wrist_left.orientation.x, wrist_left.orientation.y,wrist_left.orientation.z)
    R_elbow_left_inv = inv(R_elbow_left)
    R_trazeno = matmul(R_elbow_left_inv,R_wrist_left)
    
    wrist_left.position.x = lf*R_trazeno[1,2] + lu*sin(roll54)
    wrist_left.position.y = lf*R_trazeno[0,2] + lu*cos(pitch54)*cos(roll54)
    wrist_left.position.z = lu*sin(pitch54)*cos(roll54)

    trace = R_trazeno[0,0] + R_trazeno[1,1]+ R_trazeno[2,2]
    r = sqrt(1+(trace))
    q.w = 0.5*r
    q.x = copysign(0.5*sqrt(absolute(1+R_trazeno[0,0] - R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[2,1]-R_trazeno[1,2]))
    q.y = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] + R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[0,2]-R_trazeno[2,0]))
    q.z = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] - R_trazeno[1,1]+ R_trazeno[2,2])),(R_trazeno[1,0]-R_trazeno[0,1]))  
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    #transform
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id ="/elbow_left"
    t.child_frame_id ="/wrist_left"
   
    t.transform.translation.x = lf*R_trazeno[1,2] + lu*sin(roll54)
    t.transform.translation.y = lf*R_trazeno[0,2] + lu*cos(pitch54)*cos(roll54) 
    t.transform.translation.z = lu*sin(pitch54)*cos(roll54) #lf*R_trazeno[2,2] izbrisan
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    br.sendTransform(t)
    pub51.publish(wrist_left)
#IMU52    
def callback52(data):
    global pitch52,roll52,yaw52
    global R_elbow

    elbow_right = Pose()
    elbow_right.orientation  = data.orientation
    elbow_right.position.x = lu*sin(roll50)
    elbow_right.position.y = l9+lu*cos(pitch50)*cos(roll50)
    elbow_right.position.z = lu*sin(pitch50)*cos(roll50) 

    (roll52,pitch52,yaw52) = ang(elbow_right.orientation.x,elbow_right.orientation.y,elbow_right.orientation.z,elbow_right.orientation.w)

    R_elbow = quaternion_rotation_matrix(elbow_right.orientation.w,elbow_right.orientation.x,elbow_right.orientation.y,elbow_right.orientation.z)
    
    R_shoulder_right_inv = inv(R_shoulder_right)
    R_trazeno = matmul(R_shoulder_right_inv,R_elbow)

    #transform
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/shoulder_right"
    t.child_frame_id ="/elbow_right"
   
    trace = R_trazeno[0,0] + R_trazeno[1,1]+ R_trazeno[2,2]
    r = sqrt(1+(trace))
    q.w = 0.5*r
    q.x = copysign(0.5*sqrt(absolute(1+R_trazeno[0,0] - R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[2,1]-R_trazeno[1,2]))
    q.y = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] + R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[0,2]-R_trazeno[2,0]))
    q.z = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] - R_trazeno[1,1]+ R_trazeno[2,2])),(R_trazeno[1,0]-R_trazeno[0,1]))    
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    
    t.transform.translation.y = l9+lu*cos(pitch50)*cos(roll50)
    t.transform.translation.x = lu*sin(roll50)
    t.transform.translation.z = lu*sin(pitch50)*cos(roll50) 
    
        
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    pub52.publish(elbow_right)
#IMU53
def callback53(data):
    global roll53,pitch53,yaw53
    global R_elbow_left

    elbow_left = Pose()
    elbow_left.orientation = data.orientation
    elbow_left.position.x = lu*sin(roll54)
    elbow_left.position.y = l9+lu*cos(pitch54)*cos(roll54)
    elbow_left.position.z = lu*sin(pitch54)*cos(roll54)

    
    (roll53,pitch53,yaw53)=ang(elbow_left.orientation.x,elbow_left.orientation.y,elbow_left.orientation.z, elbow_left.orientation.w) 
    R_elbow_left = quaternion_rotation_matrix(elbow_left.orientation.w,elbow_left.orientation.x,elbow_left.orientation.y,elbow_left.orientation.z)
    R_shoulder_left_inv = inv(R_shoulder_left)
    R_trazeno = matmul(R_shoulder_left_inv, R_elbow_left)
    
    trace = R_trazeno[0,0] + R_trazeno[1,1]+ R_trazeno[2,2]
    r = sqrt(1+trace)
    q.w = 0.5*r
    q.x = copysign(0.5*sqrt(absolute(1+R_trazeno[0,0] - R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[2,1]-R_trazeno[1,2]))
    q.y = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] + R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[0,2]-R_trazeno[2,0]))
    q.z = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] - R_trazeno[1,1]+ R_trazeno[2,2])),(R_trazeno[1,0]-R_trazeno[0,1]))   
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    #transform
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/shoulder_left"
    t.child_frame_id ="/elbow_left"
    
    t.transform.translation.x = lu*sin(roll54)
    t.transform.translation.y = l9+lu*cos(pitch54)*cos(roll54)
    t.transform.translation.z = lu*sin(pitch54)*cos(roll54)
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    pub53.publish(elbow_left)
#IMU54    
def callback54(data):
    global pitch54, roll54, yaw54
    global R_shoulder_left
    shoulder_left = Pose()
    shoulder_left.orientation = data.orientation
    shoulder_left.position.x = -l9
    shoulder_left.position.y = 0.0
    shoulder_left.position.z = 0.0

    
    (roll54,pitch54,yaw54)=ang(shoulder_left.orientation.x,shoulder_left.orientation.y,shoulder_left.orientation.z, shoulder_left.orientation.w) 
    
    R_shoulder_left = quaternion_rotation_matrix(shoulder_left.orientation.w,shoulder_left.orientation.x,shoulder_left.orientation.y,shoulder_left.orientation.z)
    R_neck_inv = inv(R_neck)
    R_trazeno = matmul(R_neck_inv,R_shoulder_left)
    
    trace = R_trazeno[0,0] + R_trazeno[1,1]+ R_trazeno[2,2]
    r = sqrt(1+trace)
    q.w = 0.5*r
    q.x = copysign(0.5*sqrt(absolute(1+R_trazeno[0,0] - R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[2,1]-R_trazeno[1,2]))
    q.y = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] + R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[0,2]-R_trazeno[2,0]))
    q.z = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] - R_trazeno[1,1]+ R_trazeno[2,2])),(R_trazeno[1,0]-R_trazeno[0,1]))    
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/neck"
    t.child_frame_id = "/shoulder_left"
    
    t.transform.translation.x = -l9
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    pub54.publish(shoulder_left)
    
#IMU55 
def callback55(data):
    global accel,roll55,pitch55,yaw55
    global R_wrist
    global marker
    wrist_right = Pose()
    wrist_right.orientation  = data.orientation
     
    (roll55,pitch55,yaw55)=ang(wrist_right.orientation.x,wrist_right.orientation.y,wrist_right.orientation.z, wrist_right.orientation.w)
   
    R_wrist = quaternion_rotation_matrix(wrist_right.orientation.w,wrist_right.orientation.x,wrist_right.orientation.y,wrist_right.orientation.z)
    R_elbow_inv = inv(R_elbow)
    R_trazeno = matmul(R_elbow_inv,R_wrist)
    
    wrist_right.position.x = lf*R_trazeno[1,2] + lu*sin(roll50)
    wrist_right.position.y = l9+lf*R_trazeno[0,2] + lu*cos(pitch50)*cos(roll50)
    wrist_right.position.z = lf*R_trazeno[2,2] + lu*sin(pitch50)*cos(roll50)
    
    trace = R_trazeno[0,0] + R_trazeno[1,1]+ R_trazeno[2,2]
    r = sqrt(1+absolute(trace))
    q.w = 0.5*r
    q.x = copysign(0.5*sqrt(absolute(1+R_trazeno[0,0] - R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[2,1]-R_trazeno[1,2]))
    q.y = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] + R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[0,2]-R_trazeno[2,0]))
    q.z = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] - R_trazeno[1,1]+ R_trazeno[2,2])),(R_trazeno[1,0]-R_trazeno[0,1]))
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
       
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id ="/elbow_right"
    t.child_frame_id ="/wrist_right"
    
    t.transform.translation.x = lf*R_trazeno[1,2] + lu*sin(roll50)
    t.transform.translation.y = lf*R_trazeno[0,2] + lu*cos(pitch50)*cos(roll50) 
    t.transform.translation.z = lu*sin(pitch50)*cos(roll50) #lf*R_trazeno[2,2] izbrisan
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    pub55.publish(wrist_right)
#IMU56
def callback56(data):
    global roll56,pitch56,yaw56
    global R_neck
    
    neck = Pose()
    neck.orientation  = data.orientation
    neck.position.x = 0.0
    neck.position.y = l2+l3
    neck.position.z = 0.0

    
    (roll56,pitch56,yaw56)=ang(neck.orientation.x,neck.orientation.y,neck.orientation.z, neck.orientation.w) 
    (neck.position.x,neck.position.y,neck.position.z) = neck_pose(roll56,pitch56,yaw56)
    R_neck = quaternion_rotation_matrix(neck.orientation.w,neck.orientation.x,neck.orientation.y,neck.orientation.z)
    R_back_inv = inv(R_back)
   
    R_trazeno = matmul(R_back_inv,R_neck)
    
    trace = R_trazeno[0,0] + R_trazeno[1,1]+ R_trazeno[2,2]
    r = sqrt(1+trace)
    q.w = 0.5*r
    q.x = copysign(0.5*sqrt(absolute(1+R_trazeno[0,0] - R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[2,1]-R_trazeno[1,2]))
    q.y = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] + R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[0,2]-R_trazeno[2,0]))
    q.z = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] - R_trazeno[1,1]+ R_trazeno[2,2])),(R_trazeno[1,0]-R_trazeno[0,1]))    
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/back"
    t.child_frame_id = "/neck"
    
    t.transform.translation.x = 0.0
    t.transform.translation.y= l2+l3
    t.transform.translation.z = 0.0
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    br.sendTransform(t)
    pub56.publish(neck)
#IMU57 
def callback57(data):
    global roll57,pitch57,yaw57
    global R_back
    back = Pose()
    back.orientation  = data.orientation
    back.position.x = 0.0
    back.position.y = 0.0
    back.position.z = l1

    
    (roll57,pitch57,yaw57)=ang(back.orientation.x,back.orientation.y,back.orientation.z, back.orientation.w) 

    R_back = quaternion_rotation_matrix(back.orientation.w,back.orientation.x,back.orientation.y,back.orientation.z)
    #R = [[1,0,0],[0,1,0],[0,0,1]]
    #R_inv = inv(R)
    R_trazeno = R_back
    
    trace = R_trazeno[0,0] + R_trazeno[1,1]+ R_trazeno[2,2]
    r = sqrt(1+trace)
    q.w = 0.5*r
    q.x = copysign(0.5*sqrt(absolute(1+R_trazeno[0,0] - R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[2,1]-R_trazeno[1,2]))
    q.y = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] + R_trazeno[1,1]- R_trazeno[2,2])),(R_trazeno[0,2]-R_trazeno[2,0]))
    q.z = copysign(0.5*sqrt(absolute(1-R_trazeno[0,0] - R_trazeno[1,1]+ R_trazeno[2,2])),(R_trazeno[1,0]-R_trazeno[0,1]))
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/base_link_body"
    t.child_frame_id = "/back"
    
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = l1
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    pub57.publish(back)   

if __name__=='__main__':
    rospy.init_node('imu_listener', anonymous=True)
    shoulder_right  = rospy.get_param('~shoulder_right','/imu0')
    wrist_left	=	rospy.get_param('~wrist_left', '/imu1')
    elbow_right = rospy.get_param('~elbow_right','/imu2')
    elbow_left	=	rospy.get_param('~elbow_left' , '/imu3')
    shoulder_left   = rospy.get_param('~shoulder_left','/imu4')
    wrist_right	=	rospy.get_param('~wrist_right', '/imu5')
    neck  = rospy.get_param('~neck','/imu6')
    back = rospy.get_param('~back','/imu7')
    
    #subscribes to topic, type IMU   
    sub50  = rospy.Subscriber(shoulder_right, Imu, callback50)
    sub51  = rospy.Subscriber(wrist_left, Imu, callback51)
    sub52  = rospy.Subscriber(elbow_right, Imu, callback52)
    sub53  = rospy.Subscriber(elbow_left, Imu, callback53)
    sub54  = rospy.Subscriber(shoulder_left, Imu, callback54)
    sub55  = rospy.Subscriber(wrist_right, Imu, callback55)
    sub56  = rospy.Subscriber(neck, Imu, callback56)
    sub57  = rospy.Subscriber(back, Imu, callback57)
    #publishing msgs
    pub50  = rospy.Publisher('shoulder_right_pose', Pose, queue_size=10)
    pub51  = rospy.Publisher('wrist_left_pose', Pose, queue_size=10)
    pub52  = rospy.Publisher('elbow_right_pose', Pose, queue_size=10)
    pub53  = rospy.Publisher('elbow_left_pose', Pose, queue_size=10)
    pub54  = rospy.Publisher('shoulder_left_pose', Pose, queue_size=10)
    pub55  = rospy.Publisher('wrist_right_pose', Pose, queue_size=10)
    pub56  = rospy.Publisher('neck_pose', Pose, queue_size=10)
    pub57  = rospy.Publisher('back_pose', Pose, queue_size=10)

    rospy.spin()
