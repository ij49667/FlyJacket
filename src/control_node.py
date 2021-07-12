#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Vector3, PoseStamped
from std_msgs.msg import Float64
from math import atan2,asin, pi, cos, sin,sqrt
from sensor_msgs.msg import JointState

global current_pose
global pose_ref, pose_ref_left, pose_stamped, pose_ref_back, pose_ref_neck, pose_ref_shoulder_left, pose_ref_elbow_left
global joint1,joint2,joint3,joint4,joint5
global start_time
global joint_state 

pose_stamped = PoseStamped()
current_pose = Pose()
pose_ref = pose_ref_left = pose_ref_neck = pose_ref_back = pose_ref_elbow_left = pose_ref_shoulder_left = Pose()
joint1 = joint2 = joint3 = joint4 = joint5 =  Float64()
joint_state = JointState()

def callback1(data):
    global pose_ref
    pose_ref = data
def callback2(data):
    #data from uav
    global pose_stamped
    pose_stamped = data
def callback3(data):
    global pose_ref_left
    pose_ref_left = data   
def callback4(data):
    global pose_ref_neck
    pose_ref_neck = data
def callback5(data):
    global pose_ref_back
    pose_ref_back = data
def callback6(data):
    global pose_ref_elbow_left
    pose_ref_elbow_left = data
def callback7(data):
    global pose_ref_shoulder_left
    pose_ref_shoulder_left = data
def callback8(data):
    global joint_state
    joint_state = data
   

#publisher
pub_pose = rospy.Publisher('/uav/pose_ref',Pose,queue_size=10)
pub1 = rospy.Publisher('/uav/joint1_position_controller/command',Float64,queue_size=10)
pub2 = rospy.Publisher('/uav/joint2_position_controller/command',Float64,queue_size=10)
pub3 = rospy.Publisher('/uav/joint3_position_controller/command',Float64,queue_size=10)
pub4 = rospy.Publisher('/uav/joint4_position_controller/command',Float64,queue_size=10)
pub5 = rospy.Publisher('/uav/joint5_position_controller/command',Float64,queue_size=10)
#susbcriber
sub1 = rospy.Subscriber('wrist_right_pose',Pose,callback1)
sub2 = rospy.Subscriber('/uav/pose',PoseStamped,callback2)
sub3 = rospy.Subscriber('wrist_left_pose',Pose,callback3)
sub4 = rospy.Subscriber('/neck_pose',Pose,callback4)
sub5 = rospy.Subscriber('/back_pose',Pose,callback5)
sub6 = rospy.Subscriber('elbow_left_pose',Pose,callback6)
sub7 = rospy.Subscriber('shoulder_left_pose',Pose,callback7)
sub8 = rospy.Subscriber('/uav/joint_states',JointState,callback8)
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
def up():
    current_pose.position.x = pose_stamped.pose.position.x 
    current_pose.position.y = pose_stamped.pose.position.y
    if(pose_ref.position.z < 0.0):
                current_pose.position.z = pose_stamped.pose.position.z  - pose_ref.position.z
    else:
                current_pose.position.z = pose_stamped.pose.position.z  + pose_ref.position.z
    current_pose.orientation.x = pose_ref.orientation.x
    current_pose.orientation.y = pose_ref.orientation.y
    current_pose.orientation.z = pose_ref.orientation.z
    current_pose.orientation.w = pose_ref.orientation.w
    
def down():
    current_pose.position.x = pose_stamped.pose.position.x
    current_pose.position.y = pose_stamped.pose.position.y
    if(pose_ref.position.z < 0.0):
                current_pose.position.z = pose_stamped.pose.position.z  + pose_ref.position.z
    else:
                current_pose.position.z = pose_stamped.pose.position.z  - pose_ref.position.z
    current_pose.orientation.x = pose_ref.orientation.x
    current_pose.orientation.y = pose_ref.orientation.y
    current_pose.orientation.z = pose_ref.orientation.z
    current_pose.orientation.w = pose_ref.orientation.w
    
def right():
    if(pose_ref.position.x < 0.0):
                      current_pose.position.x = pose_stamped.pose.position.x - pose_ref.position.x
    else:
                      current_pose.position.x = pose_stamped.pose.position.x + pose_ref.position.x
    current_pose.position.y = pose_stamped.pose.position.y 
    current_pose.position.z = pose_stamped.pose.position.z 
    current_pose.orientation.x = pose_ref.orientation.x
    current_pose.orientation.y = pose_ref.orientation.y
    current_pose.orientation.z = pose_ref.orientation.z
    current_pose.orientation.w = pose_ref.orientation.w

def left():
    if(pose_ref.position.x < 0.0):
                      current_pose.position.x = pose_stamped.pose.position.x + pose_ref.position.x
    else:
                      current_pose.position.x = pose_stamped.pose.position.x - pose_ref.position.x
    current_pose.position.y = pose_stamped.pose.position.y 
    current_pose.position.z = pose_stamped.pose.position.z 
    current_pose.orientation.x = pose_ref.orientation.x
    current_pose.orientation.y = pose_ref.orientation.y
    current_pose.orientation.z = pose_ref.orientation.z
    current_pose.orientation.w = pose_ref.orientation.w
    
def neutral():
    current_pose.position.x = pose_stamped.pose.position.x 
    current_pose.position.y = pose_stamped.pose.position.y 
    current_pose.orientation.z = -(pose_ref.position.z + 2*pose_stamped.pose.position.z) #dead zone
    current_pose.orientation.x = pose_stamped.pose.orientation.x 
    current_pose.orientation.y = pose_stamped.pose.orientation.y
    current_pose.orientation.z = pose_stamped.pose.orientation.z
    current_pose.orientation.w = pose_stamped.pose.orientation.w

def forward():
    current_pose.position.x = pose_stamped.pose.position.x 
    current_pose.position.y = pose_stamped.pose.position.y + pose_ref_neck.position.y
    current_pose.position.z = pose_stamped.pose.position.z
    current_pose.orientation.x = pose_ref_neck.orientation.x
    current_pose.orientation.y = pose_ref_neck.orientation.y
    current_pose.orientation.z = pose_ref_neck.orientation.z
    current_pose.orientation.w = pose_ref_neck.orientation.w
def back():
    current_pose.position.x = pose_stamped.pose.position.x 
    current_pose.position.y = pose_stamped.pose.position.y - pose_ref_neck.position.y
    current_pose.position.z = pose_stamped.pose.position.z
    current_pose.orientation.x = pose_ref_neck.orientation.x
    current_pose.orientation.y = pose_ref_neck.orientation.y
    current_pose.orientation.z = pose_ref_neck.orientation.z
    current_pose.orientation.w = pose_ref_neck.orientation.w

def move_robot_arm():

       joint1 = atan2(-pose_ref_elbow_left.position.z, pose_ref_elbow_left.position.x)
       joint2 = atan2(pose_ref_elbow_left.position.y, sqrt(pose_ref_elbow_left.position.x**2 + pose_ref_elbow_left.position.z**2))
       joint3 = atan2(sqrt(((cos(y_elbow_left)*sin(r_elbow_left))**2) + ((sin(y_elbow_left)*sin(r_elbow_left))**2)),cos(r_elbow_left))
       joint4 = atan2(cos(y_elbow_left)*sin(r_elbow_left),sin(y_elbow_left)*sin(r_elbow_left)) 
       joint5 = 0.0

       pub1.publish(joint1)
       pub2.publish(joint2)
       pub3.publish(joint3)
       pub4.publish(joint4)
       pub5.publish(joint5)



       
def move_uav():
       global r_left, r_elbow_left, y_elbow_left
       r_left = r_elbow_left= y_elbow_left=0.0
       quat_right = pose_ref.orientation
       quat_left = pose_ref_left.orientation
       quat_back = pose_ref_back.orientation
       quat_elbow_left = pose_ref_elbow_left.orientation
       (r_right,p_right,y_right)= ang(quat_right.x,quat_right.y,quat_right.z,quat_right.w)
       (r_left,p_left,y_left)= ang(quat_left.x,quat_left.y,quat_left.z,quat_left.w)
       (r_back,p_back,y_back)= ang(quat_back.x,quat_back.y,quat_back.z,quat_back.w)
       (r_elbow_left,p_elbow_left,y_elbow_left)= ang(quat_elbow_left.x,quat_elbow_left.y,quat_elbow_left.z,quat_elbow_left.w)
       print r_right*180/pi, p_right*180/pi,y_right*180/pi, 'desna'
       print r_left*180/pi,p_left*180/pi,y_left*180/pi
       #print r_back*180/pi, p_back*180/pi,y_back*180/pi
       if (r_right>=-pi/18 and r_right<=pi/6):#roll veci od -5 i manji od 30
              neutral()
              move_robot_arm()
              if(p_right>=pi/4 and p_right<=pi/3):# pitch veci od -5 i manji od 90
                    right()
                    #move_robot_arm()
              elif(p_right>-pi/3 and p_right<-pi/4):
                    left()
                    #move_robot_arm()
              elif(r_back>=pi/2 and r_back<=2*pi/3):
                    back()
                    #move_robot_arm()
              elif(r_back>=pi/6 and r_back<pi/3):
                    forward()
                    #move_robot_arm()
              else:
                    neutral()
                    move_robot_arm()
       elif(r_right>pi/6 and r_right<=pi/2):
              up()
              #move_robot_arm()
       elif(r_right>=-pi/2 and r_right<-pi/36):
              down()
              #move_robot_arm()
       else:
              neutral()
              move_robot_arm()
       pub_pose.publish(current_pose)
    
def control():
       rospy.init_node('controller', anonymous=True)
       rate = rospy.Rate(1) #hz
       global start_time
       start_time = rospy.get_rostime()
       while not rospy.is_shutdown():
          move_uav()
                   
if __name__ == '__main__':
     try:
         control()
     except rospy.ROSInterruptException:
         pass
