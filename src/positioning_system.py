#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

Node that retrieves UWB position from a Decawave device and a Total position frome a TCRP1203 theodolite.
Only one type of topic is published at once. The decision process selects the most precise available position.

The name of the node is positioning_system
The positions are published on the topic /mavros/vision_pose/pose

"""

from __future__ import division

import time
import datetime
import socket
import sys


from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros.utils import *

import position_estimation
from TagHandler import TagHandler
from serial_range_report import read_config_file

TARGET_TOPIC = '/mavros/vision_pose/pose'
DELTA_TOPIC = '/delta/uwb'
NODE_NAME = "positioning_system"
CONFIG_NAME = "uwb_position_publisher"
TOTAL = 0
UWB = 1
IP_GROUNDSTATION = "192.168.0.150"
PORT_GROUNDSTATION = 15000


def init():
    global uwb_msg, total_msg, publishing_thread_started, uwb_msg_lock, total_msg_lock, msg_lock, publisher, publish_lock, file_path, publish_thread
    rospy.init_node(NODE_NAME)
    uwb_msg = [PoseStamped(),0]
    total_msg = [PoseStamped(),0]
    publishing_thread_started = False
    publisher = TOTAL
    file_path = rospy.get_param("/" + NODE_NAME + "/config_file_path")
    time.sleep(1)
    uwb_msg_lock = threading.Lock()
    total_msg_lock = threading.Lock()
    msg_lock = threading.Lock() # lock for publisher variable
    publish_lock = threading.Lock()
    publish_thread = threading.Thread(target=publish_message)
    uwb_message_thread = threading.Thread(target=read_uwb_positions)
    total_message_thread = threading.Thread(target=read_total_positions)
    uwb_message_thread.start()
    total_message_thread.start()


def publish_message():
    rospy.loginfo("Publish_message started")
    rate = rospy.Rate(20)
    pub = rospy.Publisher(TARGET_TOPIC, PoseStamped,queue_size=20)
    uwb_pub = rospy.Publisher("/uwb/pose",PoseStamped,queue_size=20)
    total_pub = rospy.Publisher("total/pose",PoseStamped,queue_size=20)
    deltapub = rospy.Publisher(DELTA_TOPIC,String,queue_size=20)
    try:
        global publishing_thread_started
        while not publishing_thread_started and not rospy.is_shutdown():
            continue
        while not rospy.is_shutdown():
            msg_lock.acquire()
            try:
                global publisher
		total_pub.publish(total_msg[0])
		uwb_pub.publish(uwb_msg[0])
                if publisher == TOTAL and total_msg_lock.acquire():
                    #TODO test with origin timestamp, see if timeout
                    total_msg[0].header.stamp = rospy.Time.now()
                    pub.publish(total_msg[0])
                    #deltapub.publish(str(rospy.Time.now()-total_msg[1]))
                    total_msg_lock.release()
                elif publisher == UWB and uwb_msg_lock.acquire() :
                    #TODO test with origin timestamp, see if timeout
                    uwb_msg[0].header.stamp = rospy.Time.now()
                    pub.publish(uwb_msg[0])
                    #deltapub.publish(str(rospy.Time.now()-uwb_msg[1]))
                    uwb_msg_lock.release()
            finally:
                msg_lock.release()
                rate.sleep()
    finally:
        rospy.loginfo("ROS closed: ending position publishing thread")

def read_uwb_positions():
    rospy.loginfo("UWB reader started")
    init = True
    global file_path
    port, anchor_positions = read_config_file(file_path)
    tag_handler = TagHandler(port)
    tag_handler.info_logger = rospy.loginfo
    tag_handler.error_logger = rospy.logerr
    tag_handler.open()
    try:
        while not rospy.is_shutdown():
            report = tag_handler.read_report()
            if report is None or report.mid != "mc":
                continue
            try:
                best_solution = position_estimation.compute(report, anchor_positions)
                values = best_solution
                # Lock
                if uwb_msg_lock.acquire(False):
                    try:
                        uwb_msg[1] = rospy.Time.now()
                        uwb_msg[0].header.stamp = rospy.Time.now()
                        uwb_msg[0].pose.position.x = float(values[0])
                        uwb_msg[0].pose.position.y = float(values[1])
                        uwb_msg[0].pose.position.z = float(values[2])-0.48
                        global publishing_thread_started, publish_thread
                        if init and not publishing_thread_started:
			    publish_thread.start()
                            publishing_thread_started = True
                            init = False
                    finally:
                        uwb_msg_lock.release()
            except ValueError as e:
                rospy.loginfo("Could not find solution with data! Error:" + str(e.args))
    finally:
        rospy.loginfo("ROS closed: ending UWB position reading thread")
        tag_handler.close()

def read_total_positions():
    rospy.loginfo("TOTAL reader started")
    init = True
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((IP_GROUNDSTATION,PORT_GROUNDSTATION))
    s.settimeout(1) #1 sec of nonactivity
    try:
        while not rospy.is_shutdown():
            try:
                data, address = s.recvfrom(4096)
                if data:
                    global publisher
                    if publisher != TOTAL:
                        msg_lock.acquire()
                        publisher = TOTAL
                        rospy.loginfo("switched to total!")
                        msg_lock.release()
                    publisher = TOTAL
                    values = data.split(";")
                    if total_msg_lock.acquire(False):
                        try:
                            diff = 0 #datetime.datetime.now().total_seconds()-values[3]   (s)
                            total_msg[1] = data #-diff*10**9
                            total_msg[0].header.stamp = rospy.Time.now()
                            total_msg[0].pose.position.x = float(values[0])
                            total_msg[0].pose.position.y = float(values[1])
                            total_msg[0].pose.position.z = float(values[2])
                            global publishing_thread_started, publish_thread
                            if init and not publishing_thread_started:
				publish_thread.start()
                                publishing_thread_started = True
                                init = False
                        finally:
                            total_msg_lock.release()
            except socket.timeout:
		if publisher != UWB:
                    msg_lock.acquire()
                    publisher = UWB
                    msg_lock.release()
                    rospy.loginfo("switched to uwb!")
    finally:
        rospy.loginfo("ROS closed: ending Total position reading thread")
        s.close()


if __name__ == '__main__':
    rospy.loginfo("starting positioning system")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
