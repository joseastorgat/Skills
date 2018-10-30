#!/usr/bin/env python
# -*- coding: utf-8 -*-
import qi
import sys
import rospy
from uchile_skills.robot_skill import RobotSkill

class AuditionSkill(RobotSkill):
    """
    Base class for speech recognition 
    """

    _type = "audition"


    def __init__(self):
        """
        Base class for speech recognition using speech

        Args:
            None
        Raises:
            None
        """
        super(AuditionSkill, self).__init__()
        self._description = "Speech recognition using naoqi APIS"
        self.audition_srv = None
        self.memory  = None

        self._is_subscribed = False
        self._subname = "AuditionSkill"
        self._subscriber_id = None

        self._is_running = False

        self.available_languages = ['English','Spanish']
        self.vocabulary = None

    def check(self, timeout=1.0):
        return True

    def setup(self):
        
        self.audition_srv = self.robot.session("ALSpeechRecognition")
        self.memory  = self.robot.session("AlMemory")

        self.start()

        self.pause()

        self.set_recognizer_language("English")

        return True

    def shutdown(self):

        try: 
            self.audition_srv.unsubscribe(self._subname)
        except Exception as e:
            logerr()
            return False
        return True


    def start(self):
        
        try:
            self._subscriber_id = self.audition_srv.subscribe(self._subname)
            rospy.loginfo("Subscribed to Audition Service with id: %s".format(self._subscriber_id) )
            self._is_subscribed = True        
        except Exception as e:
            rospy.logerr("Couldn't subscribe to Audition Service, Exception: %s".format(e))


        #self.logdebug("Start audition skill")
        return True

    def pause(self):
        try:
            self.audition_srv.pause(True)
            self._is_running = False

        except Exception as e:
            return False
        self.logdebug("Pause audition skill")
        return True

    def unpause(self):

        try: 
            self.audition_srv.pause(False)
            self._is_running = True
        except Exception as e:
            return False
        self.logdebug("Unpause audition skill")            
        return True

    def stop(self):
        self.logdebug("Stopping audition skill")

    # Speech recognition related methods


    def recognize_with_grammar(self, dictionary="Stage1/Stage2gpsr",timeout=16.0):

        if not dictionary == self.vocabulary:
            self._set_vocabulary(dictionary)




    def recognize(self, dictionary="Stage1/Stage2gpsr"):

    def done_cb(self, state, result):

      
    def send_goal(self, dictionary="Stage1/Stage2gpsr"):

    def wait_for_result(self,timeout = 3.0):

    def get_result(self):

    def set_recognizer_language(self,language):

        if not language in self.available_languages:
            rospy.logerr("Language not available : " + language)
            return False

        try:
            self.audition_srv.setLanguage(language)
            pass
        except Exception as e:
            rospy.logerr("Failed to set recognizer language as" + language)
            return False

        rospy.loginfo("Setting recognizer language as"+language)
        return True

    def set_audio_expresion(self,bool):
        try:
            self.audition_srv.setAudioExpresion(bool)
        except Exception as e:
            rospy.logerr("")
            return False

    def _get_words(self):




    
    def _set_vocabulary(self,vocabulary):

        voc = "/home/naoqi/nao/grammar/"+ vocabulary

        try:
            self.audition_srv.setVocabulary(voc, False)
            pass
        except Exception as e:
            return False
        self.vocabulary = vocabulary
        return True

