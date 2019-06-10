#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Name = Position publisher
Node opens a connections with groundstation, gets position of total station and publishes them.
If that connection is lost, falls back on the UWB position.
"""
from __future__ import division

import time
import socket
import sys
import datetime
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros.utils import *

TARGET_TOPIC = '/mavros/vision_pose/pose'
DELTA_TOPIC = '/delta/total'
NODE_NAME = "position_publisher"
IP_GROUNDSTATION = "192.168.0.150"
PORT_GROUNDSTATION = 15000

def init():
    global msg, msg_lock, publishing_thread_started, publish_thread, tt, deltapub
    deltapub = rospy.Publisher(DELTA_TOPIC,String,queue_size=20)
    publishing_thread_started = False
    msg = [PoseStamped(),0]
    tt = "null"
    msg_lock = threading.Lock()
    rospy.init_node(NODE_NAME)
    # create socket to connect to groundstation
    rospy.loginfo("position pusblisher started")
    time.sleep(1)
    # Threads that will read the input from the UDP connection.
    receiver_thread = threading.Thread(target = read_positions)
    publish_thread = threading.Thread(target = publish_message)
    receiver_thread.start()


def publish_message():
    rate = rospy.Rate(20)
    pub = rospy.Publisher(TARGET_TOPIC, PoseStamped, queue_size=0)
    rospy.loginfo("Publish_message started")
    try:
        while not rospy.is_shutdown():
            msg_lock.acquire()
            try:
		global deltapub
                #rospy.loginfo(str(msg) + "\n")
		#rospy.loginfo("time ros " + str(rospy.get_time()) + " from " + tt + " now " + str(datetime.datetime.now().strftime(' %d-%m-%Y %H:%M:%S ')))  
		msg[0].header.stamp = rospy.Time.now()
                pub.publish(msg[0])
		deltapub.publish("out " + str(msg[1]) + ";" +  str(datetime.datetime.now().strftime(' %d-%m-%Y %H:%M:%S ')))
            finally:
                msg_lock.release()
            rate.sleep()
    finally:
        rospy.loginfo("ROS closed: ending position publishing thread")

#TODO : msg is either total or UWB
def read_positions():
	idx = 0
	rospy.loginfo("reader started")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((IP_GROUNDSTATION,PORT_GROUNDSTATION))
        try:
            while not rospy.is_shutdown():
                data, address = s.recvfrom(4096)
                #rospy.loginfo(data)
                if data:
                    values = data.split(";")
                    if msg_lock.acquire(False):
                        try:
			    global tt
			    global deltapub 
		            deltapub.publish("in " + str(idx) +";" + values[3])
			    msg[1] = idx 
			    #rospy.loginfo(tt)
			    idx = idx +1
                            msg[0].header.stamp = rospy.Time.now()
                            msg[0].pose.position.x = float(values[0])
                            msg[0].pose.position.y = float(values[1])
                            msg[0].pose.position.z = float(values[2])
                            global publishing_thread_started
                            if not publishing_thread_started:
                                publishing_thread_started = True
                                publish_thread.start()
                        finally:
                            msg_lock.release()
        finally:
            rospy.loginfo("ROS closed: ending position reading thread")


if __name__ == '__main__':
    rospy.loginfo("Position Publisher node starting")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("failed to startup")
        pass

