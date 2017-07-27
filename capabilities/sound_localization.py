#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class SoundLocalizationSkill(RobotSkill):
    """
    """
    _type = "sound_localization"
    def __init__(self):
        """
        
        """
        super(SoundLocalizationSkill, self).__init__()
        self._description = "sound localization using naoqi apis"


    def setup(self):
        self.sound_localization = self.robot.session.service("ALSoundLocalization")
        self.memory = self.robot.session.service("ALMemory")
        self.start()
        return True

    def check(self):
        return True
    
    def start(self):
        try:
            self.sound_localization.subscribe("SoundLocalizationSkill")
        except Exception as e:
            raise e
        return True

    def pause(self):
        return None
    
    def shutdown(self):

        try:
            self.sound_localization.unsubscribe("SoundLocalizationSkill")
        except Exception as e:
            raise e
        return True

    def set_sensitivity(self,sensitivity):

        try:
            self.sound_localization.setParameter("Sensitivity",sensitivity)
        except Exception as e:
            raise e

    def _get_sound_data(self):

        try:
            return self.memory.getData("ALSoundLocalization/SoundLocated")
        
        except Exception as e:
            rospy.logerr("Naoqi ALMemory Api Error: Couldn't extract data from ALSoundLocalization/SoundLocated")
            return None

    def _localize_sound(self,timeout=20.0):
        
        
        begin = rospy.get_time()
        last_sound = self._get_sound_data()
        rospy.sleep(0.1)
        
        while(not rospy.is_shutdown() and (rospy.get_time()-begin) < timeout):
            sound_located = self._get_sound_data()
            if sound_located == last_sound:
                pass
            else:
                last_sound = sound_located
                return sound_located
        rospy.loginfo("Timeout: No sound located ")
        return []

    def get_angle(self,timeout = 20.0):

        sound = self._localize_sound(timeout)
        
        azimuth = sound[1][0]
        elevation = sound[1][1]
        confidence = sound[1][3]
        
        rospy.loginfo("Sound Located, azimuth : {0} , elevation: {1}, confidence: {2}".format(azimuth,elevation,confidence))
        return azimuth

