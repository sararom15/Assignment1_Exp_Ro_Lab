#!/usr/bin/env python

import rospy
import time
import math
import random
import smach 
import smach_ros
from geometry_msgs.msg import Point #2D coordinate
from std_msgs.msg import String #commands 
from std_msgs.msg import Float32
from std_msgs.msg import Bool




pose=Point()

#function to generate random position 
def GenerateRandomPosition(): 
    pose.x = random.randrange(1,11,1)
    pose.y = random.randrange(1,11,1)
    pose.z = 0
    return pose 

#function to choose randomly what the robot should do 
def UserAction(): 
    return random.choice(['GotoSleep','Gotoplay','Normal'])


#Normal State     
class Normal(smach.State):  
    #inizialization Normal state
    def __init__(self):
        #the outcome are 2: 'outcome1' to go to sleep state and 'outcome2' to go in Play state 
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.RandomPose = Point() 
        
    #define the execution of Normal state
    def execute(self, userdata):
        #setting the state = 1 for Normal State
        rospy.set_param('state',1)
        

        while True: 
            
            targ = rospy.Publisher("/newTargetPosition", Point, queue_size=10)
            #generate random target position 
            self.RandomPose = GenerateRandomPosition() 
            rospy.loginfo('x target is %s', self.RandomPose.x)
            rospy.loginfo('y target is %s', self.RandomPose.y)
            #publishes the target random position in a topic for displaying 
            targ.publish(self.RandomPose) 
            TimetoGetPosition = rospy.get_param("/TimetoGetPosition")
            #sleep for a while 
            time.sleep(TimetoGetPosition) 
            #choose what will be the outcome 
            comm = UserAction() 

            if comm == 'GotoSleep':
                return 'outcome1'
            elif comm == 'Gotoplay': 
                return 'outcome2'


#Sleep State
class Sleep(smach.State):
    #Initialization Sleep State 
    def __init__(self):
        #After sleep state, the robot can only come back to the Normal state: 'outcome1'
        smach.State.__init__(self, outcomes=['outcome1'])
        self.home = Point() 
    
    #define the execution if Sleep State
    def execute(self, userdata):
        rospy.set_param('state',2)
        #the home position is already defined 
        self.home.x = 0
        self.home.y = 0
        self.home.z = 0 
        rospy.loginfo('x target = x home %s', self.home.x)
        rospy.loginfo('y target = y home %s', self.home.y)
        targ = rospy.Publisher("/newTargetPosition", Point, queue_size=10)
        #publishes the home position 
        targ.publish(self.home)
        TimeforSleeping = rospy.get_param("/TimeforSleeping") 
        #waiting for a while 
        time.sleep(14) 
         
        return 'outcome1'

#Play state
class Play(smach.State): 
    #initialization Play state
    def __init__(self): 
        #After play state, the robo can only come back to the Normal state: 'outcome2' 
        smach.State.__init__(self, outcomes=['outcome2'])
        self.location = Point() 
        self.PointingGesture = Point() 
    #define the execution of the Play state
    def execute(self, userdata): 
        rospy.set_param('state', 3) 
        #generate random person position 
        self.location = GenerateRandomPosition() 
        rospy.loginfo('Location Person x = %s', self.location.x)
        rospy.loginfo('Location Person y = %s', self.location.y)
        location = rospy.Publisher("/newTargetPosition", Point, queue_size = 10)
        #publishes person position
        location.publish(self.location) 
        TimetoGetPosition = rospy.get_param("/TimetoGetPosition") 
        #wait for a while 
        time.sleep(TimetoGetPosition)
        WaitingForANewPointingGesture = rospy.get_param('WaitForANewPointingGesture')
        #wait for a new pointing gesture 
        time.sleep(WaitingForANewPointingGesture)
        #generate random Pointing Gesture location 
        self.PointingGesture = GenerateRandomPosition() 
        rospy.loginfo('PointingGesture x = %s', self.PointingGesture.x)
        rospy.loginfo('PointingGesture y = %s', self.PointingGesture.y)
        #wait for getting desired location 
        time.sleep(TimetoGetPosition) 
        #wait for coming back to person location 
        time.sleep(TimetoGetPosition)
        #wait for a new pointing gesture 
        time.sleep(WaitingForANewPointingGesture)


        #return to a Normal state 
        return 'outcome2'

def main():
    rospy.init_node('state_machine')
  


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'outcome1':'SLEEP',
                                            'outcome2':'PLAY'})
        
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'outcome1':'NORMAL'})
        
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'outcome2':'NORMAL'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()


