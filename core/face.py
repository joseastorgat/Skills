#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill


class FaceSkill(RobotSkill):
    """
    """
    _type = "face"
    def __init__(self):
        """
        Base Movement Skill
        """
        super(FaceSkill, self).__init__()
        self._description = "Face skill"


    def setup(self):

        self.eyes_blink = self.robot.session.service("ALAutonomousBlinking")
        self.leds = self.robot.session.service("ALLeds")
        return True

    def check(self):
        return True
    
    def start(self):
        return True
    
    def shutdown(self):
        self.stop()
        return True

    def set_autonomous_blink(self,bool):
        try:
            self.eyes_blink.setEnabled(bool)
        except Exception as e:
            rospy.logerr("")
        return
        
    # def set_emotion(self):
    #     pass

    # def set_brightness(self):
    #     pass

    # def set_rainbow(self):
    #     pass


    # def set_eyes(self, eye = "both" , duration = 0.0, color = None):
    #     pass

    # def set_ears(self, ear = "both" , duration = 0.0, color = None):
    #     pass

    # def set_shoulder(self, shoulder = "both" , duration = 0.0, color = None):
    #     pass

    # def set_leds_off(self, leds = "all"):
    #     return

    def random_eyes(self, duration = 10.0):
        
        try:
            self.leds.randomEyes()
        except Exception as e:
            rospy.logerr()

    def rotate_eyes(self, color, duration = 10.0, rotation_time = 0.5 ):
        """
        color in rgb hexadecimal
        """
        try:
            self.leds.rotateEyes(color,rotation_time,duration)
        
        except Exception as e:
            rospy.logerr()


    # def _reset(self):
    #     pass

    # def _fade(self):
    #     return

    # def _fade_color(self):
    #     return
    
    # def _fade_rgb(self):
    #     return

    # def _intesity(self):
    #     pass

    # def _on(self):
    #     return
    
    # def _off(self):
        # return 
