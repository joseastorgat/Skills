#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill
import numpy as np
import math

class BaseSkill(RobotSkill):
    """
    """
    _type = "base"
    def __init__(self):
        """
        Base Movement Skill
        """
        super(BaseSkill, self).__init__()
        self._description = "Maqui Base skill"

        self.base = None
        self.async = False
        """
    
        Defaults Values
        self.maxvelXY= 0.35 
        self.maxveltheta = 1.0 
        self.maxaccXY = 0.3
        self.maxaccTheta = 0.75
        maxJerkXY = 1.0
        maxJerkTheta = 2.0
        """
    
    def setup(self):

        self.base = self.robot.session.service("ALMotion")
        leftArmEnable  = True
        rightArmEnable = True
        self.base.setMoveArmsEnabled(leftArmEnable, rightArmEnable)  
        
        return True

    def check(self):

        
        return True

    def start(self):
        return True
    def shutdown(self):
        self.stop()
        return True


    
    def _move(self, x=0.0, y=0.0, thetha=0.0, execution_time=None):
        """
        This methods moves the robot 
        """

        try:
            self.base.moveInit()
        except Exception,e:
            rospy.logerr("Couldn't init")
            return False

        if execution_time is not None:
            try:
                success  = self.base.moveTo(x,y,thetha,execution_time, _async = self.async) 
            except Exception,e:
                rospy.logerr("Couldn't reach the goal, Exception %s",e)
                self.stop()
                return False
        else:
            try:
                success = self.base.moveTo(x,y,thetha, _async = self.async)
            except Exception,e:
                rospy.logerr("Couldn't reach goal: %s",e)
                self.stop()
                return False
        return success

    def move(self, x=0.0, y=0.0, thetha=0.0, execution_time=None,):      
        rospy.loginfo("[skill: \"{0}\"]: move(). Moving by x: \"{1}\" y: \"{2}\" rotation: \"{3}\" in execution_time: \"{4}\"".format(BaseSkill._type,x,y,thetha,execution_time))
        return self._move(x,y,thetha,execution_time)

    def move_forward(self, distance = 0.0, execution_time = None):
        """
        """
        rospy.loginfo("[skill: \"{0}\"]: move_forward()  . Moving forward by \"{1}\" meters.".format(BaseSkill._type,distance))
        return self._move(distance,0,0,execution_time)

    def move_backward(self, distance = 0.0, execution_time = None):
        """
        """
        rospy.loginfo("[skill: \"{0}\"]: move_backward()  . Moving backward by \"{1}\" meters.".format(BaseSkill._type,distance))
        return self._move(-distance,0,0,execution_time)

    def move_right(self, distance = 0.0, execution_time = None):
        """
        """
        rospy.loginfo("[skill: \"{0}\"]: move_right()  . Moving right by \"{1}\" meters.".format(BaseSkill._type,distance))
        return self._move(0,-distance,0,execution_time)
    
    def move_left(self, distance = 0.0, execution_time = None):
        """
        """
        rospy.loginfo("[skill: \"{0}\"]: move_left()  . Moving left by \"{1}\" meters.".format(BaseSkill._type,distance))
        return self._move(0,distance,0,execution_time)

    def rotate_rad(self,angle=0.0,execution_time=None,timeout = None):
        """
        """
        return self._move(0,0,angle,execution_time)
   
    def rotate(self,angle=0.0,execution_time=None):
        """
        """
        angle_rad = DegToRad(angle)
        rospy.loginfo("[skill: \"{0}\"]: rotate()  . Rotating by \"{1}\" degrees.".format(BaseSkill._type,angle))
        return self.rotate_rad(angle_rad,execution_time)
   
    def rotate_right(self,angle=0.,execution_time=None):
        """
        """
        angle_rad = DegToRad(angle)
        rospy.loginfo("[skill: \"{0}\"]: rotate_right()  . Rotating right by \"{1}\" degrees.".format(BaseSkill._type,angle))
        return self.rotate_rad(-angle_rad,execution_time)

    def rotate_left(self,angle=0.,execution_time=None):
        """
        """ 
        angle_rad = DegToRad(angle)
        rospy.loginfo("[skill: \"{0}\"]: rotate_left()  . Rotating left by \"{1}\" degrees.".format(BaseSkill._type,angle))
        return self.rotate_rad(angle_rad,execution_time)

    def stop(self):
        return self.base.stopMove()


    def wait_until_done(self):
        return  self.base.waitUntilMoveIsFinished()

    def is_moving(self):
        return self.base.moveIsActive()


    def move_no_omni(self,x= 0. ,y= 0.,thetha = 0., execution_time = None):

        distance,angle = cart2pol(x,-y)
        
        angle = normalize_angle(angle)
        print angle
        self.rotate_rad(-angle)
        print distance
        self.wait_until_done()

        self.move( x = distance)
        self.wait_until_done()
        success = self.rotate_rad(angle+thetha)
        self.wait_until_done()
        return True

def DegToRad(angle):
    return angle*((math.pi)/180)

def RadToDeg(angle):
    return angle*(180/(math.pi))
def cart2pol(x, y):
    rho = math.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)



"""
functions from bender base skills
"""

def normalize_angle_positive(angle):
    """ Normalizes the angle to be 0 to 2*pi
        It takes and returns radians. """
    return math.fmod(math.fmod(angle, 2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi)

def normalize_angle(angle):
    """ Normalizes the angle to be -pi to +pi
        It takes and returns radians."""
    a = normalize_angle_positive(angle)
    if a > math.pi:
        a -= 2.0 * math.pi
    return a


"""
getRobotPosition
getNextRobotPosition
"""