#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Node that sends a mission of setpoints to UAV.

The name of the node is guidance_system
The setpoints are published on the topic /mavros/setpoint_position/local
"""
from __future__ import division

import time
import socket
import sys
import datetime
from threading import Thread
import wiringpi as wpi
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String
from mavros.utils import *



TARGET_TOPIC = '/mavros/setpoint_position/local'
NODE_NAME = "guidance_system"

class _GetchUnix:
    """Fetch and character using the termios module."""
    def __init__(self):
        import tty, sys
        from select import select

    def __call__(self):
        import sys, tty, termios
        from select import select
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            [i, o, e] = select([sys.stdin.fileno()], [], [], 1)
            if i:
                ch = sys.stdin.read(1)
            else:
                ch = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

getch = _GetchUnix()

def State_Callback(data):
    global state
    state = data

def Pose_Callback(data):
    global pose, init, home
    pose = data
    if init :
        home.x = pose.pose.position.x
        home.y = pose.pose.position.y
        home.z = pose.pose.position.z
        init = False

def sendSetpoint(mission):
    global setpoint, run, pose, home
    local_setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=0)
    rate = rospy.Rate(20)
    idx = -1
    #wait for home position to be initialised
    while init:
        sleep(1)
    setpoint = home
    #mission.append([home.x,home.y,home.z])
    set_mode_client(custom_mode = "OFFBOARD")
    arming_client(True)
    while run and idx < len(mission):
        if(reached(setpoint, pose)):
            rospy.loginfo("position reached!\n")
            idx= idx + 1
            if(idx == len(mission)):
                arming_client(False)
                run = False
            elif(mission[idx] == 'A'):
                wpi.digitalWrite(0,1)
            elif(mission[idx] == 'D'):
                wpi.digitalWrite(0,0)
            else:
                setpoint.x, setpoint.y, setpoint.z = mission[idx][0], mission[idx][1], mission[idx][2]
        else :
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = float(setpoint.x)
            msg.pose.position.y = float(setpoint.y)
            msg.pose.position.z = float(setpoint.z)
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 0
            local_setpoint_pub.publish(msg)
            rate.sleep()
    time.sleep(1)
    rospy.loginfo("exiting")
    exit()

def reached(setpoint,position):
    #rospy.loginfo(setpoint)
    #rospy.loginfo(position)
    if(position.pose.position.x < (setpoint.x + 0.1) and position.pose.position.x > (setpoint.x - 0.1) and position.pose.position.y < (setpoint.y + 0.1) and position.pose.position.y > (setpoint.y - 0.1) and position.pose.position.z < (setpoint.z + 0.1) and position.pose.position.z > (setpoint.z -0.1) ):
        rospy.loginfo(pose)
	    return True
    else:
        return False

#security measures
def InterfaceKeyboard():
    global pose, setpoint, home
    global arming_client, set_mode_client
    #Securities
    what = getch()
    if what == "d":
        #Disarm
        arming_client(False)
        rospy.loginfo("Disarming")
    if what == "a":
        #Arm
        arming_client(True)
        rospy.loginfo("Arming")
    if what == "o":
        #setting mode to offboard
        set_mode_client(custom_mode = "OFFBOARD")
    if what == "w":
        #activate electromagnet
        wpi.digitalWrite(0,1)
        rospy.loginfo("Electromagnet activated")
    if what == "x":
        #deactivate electromagnet
	    wpi.digitalWrite(0,0)
        rospy.loginfo("Electromagnet deactivated")
    if what == "l":
        #LAND
        set_mode_client(custom_mode = "AUTO.LAND")
        rospy.loginfo("Land")
    if what == "o":
        #LOITER
	    set_mode_client(custom_mode = "AUTO.LOITER")
        rospy.loginfo("Loiter")
    if what == "h":
        #Go Home
	    setpoint  = home
        run = False
        rospy.loginfo("going home")
    if what == "m":
        #LOITER
	    set_mode_client(custom_mode = "MANUAL")
        rospy.loginfo("Manual mode activated")
    if what == "e":
        run = False
        time.sleep(1)
        exit()

def init(mission):
    global state, setpoint, run, pose, home, init
    global arming_client, set_mode_client
    #Global variable initialisation
    state, pose, setpoint, home = State(), PoseStamped(), Point(), Point()
    run = True
    init = True
    # Node initiation
    rospy.init_node(NODE_NAME)
    rospy.loginfo('node initializeed')
    time.sleep(1)
    # setting up the pins for electromagnet
    wpi.wiringPiSetup()
    wpi.pinMode(0,1)
    # Publishers, subscribers and services
    pose_sub        = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, Pose_Callback)
    state_sub       = rospy.Subscriber('/mavros/state', State, State_Callback)
    rospy.wait_for_service('mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    # Thread to send setpoints - mission execution
    tSetPoints = Thread(target=sendSetpoint,args=(mission,)).start()
    # Monitor security
    while not rospy.is_shutdown():
        InterfaceKeyboard()

if __name__ == '__main__':
    rospy.loginfo("Guidance system starting")
    #complete your mission here: FORMAT [[X,Y,Z],'A','D']
    mission = [[3,4,1],[3,4,0]]
    try:
        init(mission)
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
