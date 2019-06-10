#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

Node that retrieves position from a Decawave EVB1000 device and publish them.

The name of the node is uwb_position_publisher
Position is published as a PoseStamped on the topic /uwb/pose

"""

from __future__ import division

import time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros.utils import *

import position_estimation
from TagHandler import TagHandler
from serial_range_report import read_config_file

# See https://github.com/PX4/Devguide/blob/master/en/ros/external_position_estimation.md
# Depending on the active mode, either one of these topic is read by MavROS for external position estimation
# TARGET_TOPIC = '/mavros/mocap/pose'
TARGET_TOPIC = '/mavros/vision_pose/pose'
TARGET_TOPIC = '/uwb/pose'
DELTA_TOPIC = '/delta/uwb'
NODE_NAME = "uwb_position_publisher"


def init():
    global msg, publishing_thread_started, deltapub
    deltapub = rospy.Publisher(DELTA_TOPIC,String,queue_size=20)
    msg = [PoseStamped(),0]
    publishing_thread_started = False
    rospy.init_node(NODE_NAME)
    file_path = rospy.get_param("/" + NODE_NAME + "/config_file_path")

    rospy.loginfo("UWB position publisher started")
    time.sleep(1)

    msg_lock = threading.Lock()

    def publish_message():
        rate = rospy.Rate(20)
        pub = rospy.Publisher(TARGET_TOPIC, PoseStamped, queue_size=20)
        rospy.loginfo("Publish_message started")
        try:
            while not rospy.is_shutdown():
                msg_lock.acquire()
                try:
		    global deltapub
		    msg[0].header.stamp = rospy.Time.now()
                    pub.publish(msg[0])
		    #rospy.loginfo("delta" + str(rospy.Time.now()-msg.header.stamp))
                    deltapub.publish("out " + str(msg[1]) + ";" + str(rospy.Time.now()))
		    #rospy.loginfo(str(msg) + "\n")
                finally:
                    msg_lock.release()
                rate.sleep()
        finally:
            rospy.loginfo("ROS closed: ending UWB position publishing thread")

    def read_positions():
	idx = 0 
        port, anchor_positions = read_config_file(file_path)

        tag_handler = TagHandler(port)
        tag_handler.info_logger = rospy.loginfo
        tag_handler.error_logger = rospy.logerr
        tag_handler.open()
        try:
            while not rospy.is_shutdown():
                report = tag_handler.read_report()
		global deltapub 
		deltapub.publish("in " + str(idx) +";" + str(rospy.Time.now()))
		idx = idx +1
                if report is None or report.mid != "mc":
                    continue

                #rospy.loginfo(report)
                try:
                    best_solution = position_estimation.compute(report, anchor_positions)
                    #rospy.loginfo(best_solution)
                    # TODO update message instead of recreating one ?

                    values = best_solution

                    # Lock
                    if msg_lock.acquire(False):
                        try:
			    msg[1] = idx
                            msg[0].header.stamp = rospy.Time.now()
                            msg[0].pose.position.x = float(values[0])
                            msg[0].pose.position.y = float(values[1])
                            msg[0].pose.position.z = float(values[2])-0.43
                            global publishing_thread_started
                            if not publishing_thread_started:
                                publishing_thread_started = True
                                publish_thread.start()
                        finally:
                            msg_lock.release()

                except ValueError as e:
                    rospy.loginfo("Could not find solution with data! Error:" + str(e.args))
        finally:
            rospy.loginfo("ROS closed: ending UWB position reading thread")
            tag_handler.close()

    publish_thread = threading.Thread(target=publish_message)
    message_thread = threading.Thread(target=read_positions)

    message_thread.start()


if __name__ == '__main__':
    rospy.loginfo("UWB position publisher node ready")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
