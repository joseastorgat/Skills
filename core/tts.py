#!/usr/bin/env python
# -*- coding: utf-8 -*-
import qi
import sys
import rospy
from uchile_skills.robot_skill import RobotSkill

class TTSSkill(RobotSkill):
    """
    The TTSSkill

    TODO
    """
    _type = "tts"
    def __init__(self):
        super(TTSSkill, self).__init__()

        self.session = None
        
        self.tts = None
        self.gestures = None

        self._language = None
        self._languages = None
        self._voices = None

        self.is_talking = False 
        self.is_gestures_active = False

    def check(self, timeout = 1.0):
        return True


    def setup(self):
        
        #Get the Naoqi service AlTextToSpeech
        
        self.tts = self.robot.session.service("ALTextToSpeech")
        self.memory = self.robot.session.service("ALMemory")
        
        self.animated_tts = self.robot.session.service("ALAnimatedSpeech")
        self.gestures = self.robot.session.service("ALSpeakingMovement")

        self._language = self.tts.getLanguage()
        self._languages = self.tts.getAvailableLanguages()
        self._voices = self.tts.getAvailableVoices()
        self.set_gestures_mode(0)

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


    def say(self, text="bender yes", block = False):
        try:
            self.tts.say(text,_async = not block)
        except Exception as e:
            rospy.logerr("[skill: {0}]: {1}".format(TTSSkill._type,e))
            return False
        return True

    def say_with_gestures(self, text="bender yes"):
                
        self.animated_tts.say(text)
        self.wait_until_done()
        return True


    def set_language(self, language="English"):
        
        try :
            self.tts.setLanguage(language)
            rospy.loginfo( "[skill: {0}]: TTS language set as {1}".format(TTSSkill._type,language))
        
        except RuntimeError:
            rospy.logerr("You need to install "+ language +" language. \n Available Languages are:" + str(self._languages) + str(self._voices))
            return False
        return True

    def set_speed(self,speed=100):
        try:
            self.tts.setParameter("speed", speed)
            rospy.loginfo( "[skill: {0}]: TTS speed set at {1} %".format(TTSSkill._type,speed))
            return True
        except RuntimeError:
            rospy.logerr("[skill: {0}]:".format(TTSSkill._type))
            return False
    
    def set_volume(self,volume=1.0):
        try:
            self.tts.setVolume(volume)
            rospy.loginfo("[skill: {0}]: TTS volume set at {1} %".format(TTSSkill._type,volume*100))
            return True
        except RuntimeError:
            rospy.logerr("[skill {0}]: Couldn't set TTS volume. Exception : {1}".format(TTSSkill._type, RuntimeError))
            return False

    def stop(self):
        
        try:
            self.tts.stopAll()
        except RuntimeError:
            rospy.logerr("couldn't stop TTSSkill")
            return False
        return True


    def wait_until_done(self,timeout=10.0):
        begin = rospy.get_time()
        
        self.is_talking = self.memory.getData("ALTextToSpeech/TextStarted")
        
        while(not rospy.is_shutdown() and self.is_talking  and (rospy.get_time()-begin) < timeout):
            
            if self.memory.getData("ALTextToSpeech/TextDone"):
                self.is_talking = False
                break
            pass
        return

    def set_gestures_mode(self,mode = 0):

        active = False if mode == 0 else True

        try:
            self.gestures.setEnabled(active)

        except Exception as e:
            rospy.logerr(" [skill: {0}]: Error setting up gestures. Exception :  {1}".format(TTSSkill._type,e))
            return False
                
        if not active:
            self.is_gestures_active = False
            return True
        else:
           
            _mode = "random" if mode == 1 else "contextual"
            self.is_gestures_active = True
           
            try:
                self.gestures.setMode(_mode)
                return True
            except Exception as e:
                rospy.logerr(" [skill: {0}]: Error setting up gestures mode. Exception :  {1}".format(TTSSkill._type,e))
                return False

#rospy.loginfo( "[skill: {0}]:".format(TTSSkill._type))
