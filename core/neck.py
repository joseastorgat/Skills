#!/usr/bin/env python
# -*- coding: utf-8 -*-
import qi
import rospy
from uchile_skills.robot_skill import RobotSkill

class NeckSkill(RobotSkill):
    """
    The LookSkill

    TODO
    """
    _type = "neck"

    def __init__(self):
        """
        Base class for sound play.
        """
        super(NeckSkill, self).__init__()

        self.session = None
        self.track_srv = None
        self.motion_srv = None
        
        self.vel = 0.3
        self.max_vel = 1.0
        
        self.UseWholeBody = False
        self.async = False
        
        self.frame = 2 #Frame Robot
        # self.frame = 0 #FrameTorso

        self.x = 0.001
        self.y = 0.
        self.z = 0.

        self.max_x = 10.
        self.min_x = 0.001

        self.max_y= 10.
        self.min_y = -10.

        self.max_z = 10.
        self.min_z = -10.


    def check(self, timeout = 1.0):
        return True

    def setup(self):

        
        #Get the Naoqi service AlTextToSpeech
        self.track_srv = self.robot.session.service("ALTracker")
        self.motion_srv = self.robot.session.service("ALMotion")

        self.track_srv.setMaximumVelocity(self.max_vel)
        #print("Setting max velocity = " + str(self.max_vel)) + "rads/sec"
        
        return True

    def shutdown(self):
        rospy.loginfo("{skill: %s}: shutdown()." % self._type)
        return True
    def start(self):
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True
    def pause(self):
        rospy.loginfo("{skill: %s}: pause()." % self._type)

        return True



    def look_at(self,x = 1.0, y= 0., z = 0. ):
        """
        Look at the target with head.
        If tracker is active, it’s stopped and restarted at the last location of active target after lookAt. This is a blocking call.
        """     
        if self.isTargetValid(x,y,z):
            try:
                
                self.x = x
                self.y = y
                self.z = z
                self.track_srv.lookAt([self.x, self.y, self.z], self.frame, self.vel, self.UseWholeBody,_async = self.async)
                rospy.loginfo("look at : {0} {1} {2}".format(self.x,self.y,self.z))
                return True                
            except RuntimeError:
                rospy.loginfo("Error")
                return False
        else:
            rospy.loginfo("target is not valid")
            return False

    def look_left(self):

        return self.look_at(self.min_x,10.0,0.6)

    def look_right(self):
        return self.look_at(self.min_x,-10, 0.6)

    def look_at_ground(self):
        return self.look_at(x=0.5,y=0.0,z=0.0) 

    def look_person(self):
        return self.look_at(x=1.0,y=0.0,z=1.4)

    def look_front(self):
        return self.look_at(x=1.0,y=0.0,z=1.0)

    def home(self):
        return self.look_at(x=1.0,y=0.0,z=1.0)

    #def home(self):
    #    pass

    def nope(self):

        vel = self.vel
        async = self.async
        self.async = False

        self.home()

        self.set_vel(0.3)
        self.look_at(x = 10.0 , y = 7.0, z = 1.9)
        #self.wait_for_motion_done()
        self.look_at(x = 10.0 , y = -7.0, z = 1.9)
        #self.wait_for_motion_done()
        self.look_at(x = 10.0 , y = 7.0, z = 1.9)
        #self.wait_for_motion_done()
        self.look_front()

        self.set_vel(vel)
        self.async = async

        return True

    def nod(self):
        
        vel = self.vel
        async = self.async
        self.async = False

        self.home()

        self.set_vel(0.1)
        self.look_at(x = 5.0 , y = 0.0, z = 3.0)
        #self.wait_for_motion_done()
        self.look_at(x = 5.0 , y = 0.0, z = 1.0)
        #self.wait_for_motion_done()
        self.look_at(x = 5.0 , y = 0.0, z = 3.0)
        #self.wait_for_motion_done()
        self.look_front()

        self.set_vel(vel)

        self.set_vel(vel)
        self.async = async

        return True
    
    def wait_for_motion_done(self, timeout=0.0):
        pass
        #return self.motion_srv.waitUntilMoveIsFinished()



    def set_vel(self,vel):

        """
        Set the velocity of the head. 
        Recieve a fraction of the maximum velocity
        """
        if vel>0:
            
            if vel > self.max_vel:
                vel = self.max_vel

            print("velocity set at " + str(vel))
            self.vel = vel
        else:
            print("Invalid Vel")

    def get_vel(self):

        print( "Maximum vel =" + str(self.max_vel) + "rads/sec \n Actual vel is " + str(self.vel*self.max_vel)+ "rads/sec")
        return self.vel

    def _set_whole_body(self,bool=False):
        self.UseWholeBody = bool
        pass


    def isTargetValid(self,x,y,z):
        x_valid = x>=self.min_x and x<=self.max_x
        y_valid = y>=self.min_y and y<=self.max_y
        z_valid = z>=self.min_z and z<=self.max_z        
        is_valid = (x_valid and y_valid and z_valid) 
        return  is_valid