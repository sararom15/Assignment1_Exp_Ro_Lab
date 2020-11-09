#!/usr/bin/env python

## @file Display.py 
# @brief This node displays the target Position 
# 
# Details: It subsscribe to a newtargetposition topic and publishes what the robot is doing 
#

import rospy
import time
import math
import random
from geometry_msgs.msg import Point #2D coordinate
from std_msgs.msg import String #commands 


NewTarget = Point() 

## odomCallback : callback for new Target Position 
def odomCallback(data): 
    NewTarget.x = data.x
    NewTarget.y = data.y
    NewTarget.z = data.z 

    

## initialization Position 
old_Position = Point() 
old_Position.x = 0
old_Position.y = 0 
old_Position.z = 0 

## ReadPosition function 
def ReadPosition():
    rospy.loginfo('I am waiting the new target')
    ## subscribe to newTargetPosition
    rospy.Subscriber("newTargetPosition", Point, odomCallback) 
    
    ## wait for a message 
    NewTarget = rospy.wait_for_message("newTargetPosition", Point) 

    ## Main Loop 
    while not ((old_Position.x == NewTarget.x) and (old_Position.y == NewTarget.y)): 
        ## Get the state parameter 
        state = rospy.get_param('state') 
        ## Normal state 
        if state == 1: #we are in Normal state 
    
            rospy.loginfo('we are in Normal state')
            
            TimetoGetPosition=rospy.get_param('TimetoGetPosition')
            
            rospy.loginfo('The target position is achieved')
            time.sleep(TimetoGetPosition)
            
        ## Sleep state 
        if state == 2: #we are in Sleep state 

            rospy.loginfo('we are in Sleep state')
            TimetoGetPosition=rospy.get_param('TimetoGetPosition')
            time.sleep(TimetoGetPosition)
            rospy.loginfo('The home position is achieved and I am sleeping')
            Timeforsleeping = rospy.get_param('TimeforSleeping') 
            time.sleep(Timeforsleeping)
            
        ## Play state 
        if state ==3: #we are in Play state 
            rospy.loginfo('we are in Play state')
            TimetoGetPosition = rospy.get_param('TimetoGetPosition')
            time.sleep(TimetoGetPosition)
            rospy.loginfo('The person location is achieved and I am waiting for a Pointing Gesture') 
            WaitingForANewPointingGesture = rospy.get_param('WaitForANewPointingGesture')
            time.sleep(WaitingForANewPointingGesture)
            time.sleep(TimetoGetPosition)
            rospy.loginfo('The pointed location is achieved')
            time.sleep(TimetoGetPosition)
            rospy.loginfo('I came back and I am waiting for a new Pointing Gesture')
            time.sleep(WaitingForANewPointingGesture)

    #old.Position = NewTarget

## Main function 
if __name__ == "__main__":
    ## init the ros node 
    rospy.init_node('display')
    ReadPosition() 




