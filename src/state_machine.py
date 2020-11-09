#!/usr/bin/env python

## @file state_machine.py 
# @brief This node implements a state machine 
# 
# Details: It is the main node: it receives commands from the commander node, implements the state machine and sends the target positions to display node. 
# 

import rospy
import time
import math
import random
import smach 
import smach_ros
from geometry_msgs.msg import Point #2D coordinate
from std_msgs.msg import String #commands 


pose=Point()

## GenerateRandomPosition function
def GenerateRandomPosition(): 
    ## Random position in a 11x11 grid
    pose.x = random.randrange(1,11,1)
    pose.y = random.randrange(1,11,1)
    pose.z = 0
    return pose 

userdata = String() 

## commcallback : callback for the command 
def commcallback(data): 
    userdata = data.data

PersonPosition = Point() 

## PersonPositioncallback : callback for the Person Position 
def PersonPositioncallback(pose): 
    PersonPosition.x = pose.x 
    PersonPosition.y = pose.y
    PersonPosition.z = 0

PointingGesture = Point() 

## PointingGesturecallback: callback for the Pointed Location 
def PointingGesturecallback(pose2): 
    PointingGesture.x = pose2.x 
    PointingGesture.y = pose2.y 
    PointingGesture.z = 0


## Normal state definition    
class Normal(smach.State):

    ## inizialization
    def __init__(self):
        ## 2 outcomes defined 
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.RandomPose = Point() 
        
    ## execution 
    def execute(self, userdata):
        ## set parameter service state = 1 
        rospy.set_param('state',1)
        
        ## Main Loop 
        while True: 
            ## subscribe to the command topic 
            rospy.Subscriber("command",String, commcallback) 
            usercommand = rospy.wait_for_message("command", String) 
            
            targ = rospy.Publisher("/newTargetPosition", Point, queue_size=10)
            ## generate random target position 
            self.RandomPose = GenerateRandomPosition() 
            rospy.loginfo('x target is %s', self.RandomPose.x)
            rospy.loginfo('y target is %s', self.RandomPose.y)
            ## publish the target random position in a topic for displaying 
            targ.publish(self.RandomPose) 
            rospy.loginfo('command received is %s',usercommand.data)
            TimetoGetPosition = rospy.get_param("/TimetoGetPosition")
            ## sleep for a while 
            time.sleep(TimetoGetPosition) 
            
            ## choose the outcome
            if usercommand.data == "play" : 
                return 'outcome2'
            if usercommand.data == "sleep" : 
                return 'outcome1'




## Sleep State definition 
class Sleep(smach.State):
    ## Initialization
    def __init__(self):
        ## 1 outcome defined : Normal 
        smach.State.__init__(self, outcomes=['outcome1'])
        self.home = Point() 
    
    ## Execution 
    def execute(self, userdata):
        ## set parameter service state = 2
        rospy.set_param('state',2)
        ## Define the home position 
        self.home.x = 1
        self.home.y = 1
        self.home.z = 0 
        rospy.loginfo('x target = x home %s', self.home.x)
        rospy.loginfo('y target = y home %s', self.home.y)
        targ = rospy.Publisher("/newTargetPosition", Point, queue_size=10)
        ## publish the home position 
        targ.publish(self.home)
        TimeforSleeping = rospy.get_param("/TimeforSleeping") 
        ## sleep for a while 
        time.sleep(14) 
         
        return 'outcome1'

## Play state definition 
class Play(smach.State): 
    ## initialization 
    def __init__(self): 
        ## 1 outcome defined : Normal 
        smach.State.__init__(self, outcomes=['outcome2'])
        self.location = Point() 
        self.PointingGesture = Point() 

    ## Execution
    def execute(self, userdata): 
        ## set parameter service state = 3
        rospy.set_param('state', 3) 
        ## Subscribe to PersonPosition topic 
        rospy.Subscriber("PersonPosition", Point, PersonPositioncallback)
        position = rospy.wait_for_message("PersonPosition", Point) 

        rospy.loginfo('Location Person x = %s', position.x)
        rospy.loginfo('Location Person y = %s', position.y)
        pos = rospy.Publisher("/newTargetPosition", Point, queue_size = 10)
        ## publish person position for displaying 
        pos.publish(PersonPosition) 
        TimetoGetPosition = rospy.get_param("/TimetoGetPosition") 
        ## wait for a while 
        time.sleep(TimetoGetPosition)
        WaitingForANewPointingGesture = rospy.get_param('WaitForANewPointingGesture')
        ## wait for a new pointing gesture 
        time.sleep(WaitingForANewPointingGesture)
        ## Subscribe to Pointing Gesture topic 
        rospy.Subscriber("PointingGesture", Point, PointingGesturecallback)
        point = rospy.wait_for_message("PointingGesture", Point) 
        rospy.loginfo('PointingGesture x = %s', point.x)
        rospy.loginfo('PointingGesture y = %s', point.y)
        ## wait for getting desired location and coming back to person position 
        time.sleep(TimetoGetPosition) 
        time.sleep(TimetoGetPosition)
        ## wait for a new pointing gesture 
        time.sleep(WaitingForANewPointingGesture)


        #return to a Normal state 
        return 'outcome2'

## Main Function definition 
def main():
    ## init the ros node 
    rospy.init_node('state_machine')
  


    ## Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    ## Open the container
    with sm:
        ## Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'outcome1':'SLEEP',
                                            'outcome2':'PLAY'})
        
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'outcome1':'NORMAL'})
        
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'outcome2':'NORMAL'})

    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()


