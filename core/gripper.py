#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from uchile_skills.robot_skill import RobotSkill

class GripperSkill(RobotSkill):
    """
    Base class for gripper control using naoqi ALMotion service
    """

    _type = "gripper"

    L_GRIPPER = "LHand"
    """str: Left gripper name"""

    R_GRIPPER = "RHand"

    OPEN_POSITION = 1.0
    
    CLOSE_POSITION = 0.0

    DEFAULT_POSITION = 0.5

    MAX_FRACTION_SPEED = 0.5
    """str: Right gripper name"""
    
    def __init__(self,gripper):
        super(GripperSkill,self).__init__()
        self.motion_srv = None
        self.hand = gripper
        
    def setup(self):
        self.motion_srv = self.robot.session.service("ALMotion")
        return True
    
    def check(self):
        return True
    
    def start(self):
        return True

    def shutdown(self):
        self._default()
        return True
    
    def close(self, timeout = 0.0):
        #Closes the hand, then cuts motor current to conserve energy. This is a blocking call.
        self.motion_srv.angleInterpolationWithSpeed(self.hand,GripperSkill.CLOSE_POSITION,GripperSkill.MAX_FRACTION_SPEED)
        return True

    def open(self, timeout = 0.0):
        
        self.motion_srv.angleInterpolationWithSpeed(self.hand,GripperSkill.OPEN_POSITION,GripperSkill.MAX_FRACTION_SPEED)
        return True

    def _default(self):

        self.motion_srv.angleInterpolationWithSpeed(self.hand,GripperSkill.DEFAULT_POSITION,GripperSkill.MAX_FRACTION_SPEED)
   
    def wait_for_motion_done(self):
        return True

    def get_result(self):
        return True


class LeftGripperSkill(GripperSkill):
    """Left gripper control using gripper command action"""
    _type = "l_gripper"
    def __init__(self):
        super(LeftGripperSkill, self).__init__(GripperSkill.L_GRIPPER)

class RightGripperSkill(GripperSkill):
    """Right gripper control using gripper command action"""
    _type = "r_gripper"
    def __init__(self):
        super(RightGripperSkill, self).__init__(GripperSkill.R_GRIPPER)
