#!/usr/bin/env python
#-*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class EmotionRecognitionSkill(RobotSkill):
    """
    """
    _type = "emotion_recognition"
    def __init__(self):
        """
        
        """
        super(EmotionRecognitionSkill, self).__init__()
        self._description = ""


    def setup(self):

        self.memory = self.robot.session.service("ALMemory")
    
        self.face_characteristic = self.robot.session.service("ALFaceCharacteristics")

        self.people_perception = self.robot.session.service("ALPeoplePerception")
        return True

    def check(self):
        return True
    
    def start(self):
        return True

    def pause(self):
        return None
    
    def shutdown(self):
        return True


    def get_emotion(self,id,features = ["emotion","smile"]):
        self.timeout = 1.0
            
        ids = self._get_ids()

        if len(ids) == 0 or id not in ids:
            self.logerr("No person detected")
            return []

        a = False
        begin = rospy.get_time()
        while not a:
            a = self.face_characteristic.analyzeFaceCharacteristics(id)
            rospy.sleep(0.1)
            if rospy.get_time()-begin > self.timeout:
                rospy.logerr("couldn't analyze your face")
                break
            
        person_emotions = []
        emotions = []
        smile = []
        if "emotion" in features:
            try:
                emotions = self.memory.getData("PeoplePerception/Person/"+str(id)+"/ExpressionProperties")
            except Exception as e:
                rospy.logerr(e)
            person_emotions.append(emotions)

        if "smile" in features:
            try:
                smile= self.memory.getData("PeoplePerception/Person/"+str(id)+"/SmileProperties")
            except Exception as e:
                rospy.logerr(e)
            person_emotions.append(smile)
        
        return  person_emotions
    

    def _get_ids(self):    
        return self.memory.getData("PeoplePerception/PeopleList")