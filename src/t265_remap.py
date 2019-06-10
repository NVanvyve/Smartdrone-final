#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
remaps the information from the  realsense camera to the vision topic
The name of the node is  remapper
Position is published as a PoseWithCovarianceStamped on  /mavros/vision_pose/pose_cov

"""

from __future__ import division

import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mavros.utils import *
import math
import tf

#OUT_TOPIC = '/mavros/vision_pose/pose_cov'
OUT_TOPIC = '/cam/pose'
IN_TOPIC = '/camera/odom/sample'
NODE_NAME = "remapper"

def callback(data):
    global msg, lock, home, diff
    #msg = PoseWithCovarianceStamped()
    if(lock.acquire(False)):
    	msg.header = data.header
    	msg.header.stamp = rospy.Time.now()
	Q = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w) 
	euler = tf.transformations.euler_from_quaternion(Q)
	yaw = euler[2]	 
	#rospy.loginfo(yaw*180/math.pi)
	#rospy.loginfo(diff*math.cos(yaw))
	msg.pose = data.pose 
    	msg.pose.pose.position.x  = (data.pose.pose.position.x+home[0]+diff)-(diff*math.cos(yaw))
	msg.pose.pose.position.y =  (data.pose.pose.position.y+home[1])-(diff*math.sin(yaw))
	lock.release()
    #pub.publish(msg)
	
def init():
    global pub, msg, lock, home, diff  
    rospy.init_node(NODE_NAME)
    diff = 0.15
   # home = [4.379,-0.010,0.0008]
    home =[3.03,4,0]
    msg = PoseWithCovarianceStamped()
    lock = threading.Lock()
    time.sleep(1)
    rate = rospy.Rate(20)
    pub = rospy.Publisher(OUT_TOPIC, PoseWithCovarianceStamped, queue_size=0)
    sub = rospy.Subscriber(IN_TOPIC, Odometry, callback)
    while not rospy.is_shutdown():
	lock.acquire()
	pub.publish(msg)
	lock.release()
	rate.sleep()

if __name__ == '__main__':
    rospy.loginfo("remapper starting")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
