#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class BasicMotionSkill(RobotSkill):
    """
    """
    _type = "basic_motion"
    def __init__(self):
        """
        Base Movement Skill
        """
        super(BasicMotionSkill, self).__init__()
        self._description = "skill to manage basic moves of maqui"

        self._chains = ['Body','Legs', 'Arms', 'LArm', 'RArm','Head']

    def setup(self):

        self.memory = self.robot.session.service("ALMemory")
        self.motion = self.robot.session.service("ALMotion")
        self.posture = self.robot.session.service("ALRobotPosture")

        self.set_idle(bool = False)
        self.set_breathing(bool = False)

        return True

    def check(self):
        return True
    
    def start(self):
        return True

    def pause(self):
        return None
    
    def shutdown(self):
        return True

    def wake_up(self):
        
        """
        Turns the Stiffness of its motors on,
        Goes to a standing posture,
        Resumes its life
        """

        try:
            self.motion.wakeUp()
        except Exception as e:
            rospy.logerr(e)
        
        return

    def rest(self):
        """
        Goes to its resting posture,
        Turns the Stiffness of its motors off.

        """

        try:
            self.motion.rest()
        except Exception as e:
            rospy.logerr(e)
        return

    def robot_is_awake(self):
        try:
            result = self.motion.robotIsWakeUp()
        except Exception as e:
            rospy.logerr(e)
            return None
        return result

    def set_posture(self,posture = "Stand",speed = 0.8):
        
        if not posture in ["Stand", "StandZero","Crouch"]:
            return False
        try:
            self.posture.goToPosture(posture, speed) # block method
        except Exception as e:
            rospy.logerr(e)
            return False
        return True
    
    def stop_posture(self):

        try:
            self.posture.stopMove()
        except Exception as e:
            rospy.logerr(e)


    def set_breathing(self, chains = ["Body"], bool = True):

        for chain in chains:
            if chain in self._chains:
                try:
                    self.motion.setBreathEnabled(chain, bool)
                except Exception as e:
                    rospy.logerr(e)
            else:
                rospy.logerr("Error setting Breathing: "+ chain + " is not a robot chain" )

    def set_idle(self, chains = ['Body','Legs', 'Arms', 'LArm', 'RArm','Head'] , bool = True):
        for chain in chains:
            if chain in self._chains:
            
                try:
                    self.motion.setIdlePostureEnabled(chain, bool)
                    return True
                except Exception as e:
                    rospy.logerr(e)
                    return False
            else:
                rospy.logerr("Error setting Breathing: "+ chain + " is not a robot chain" )        
                return False

"""
Continue with Autonomous Life
"""