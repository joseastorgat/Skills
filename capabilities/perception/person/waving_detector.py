#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class WavingDetectorSkill(RobotSkill):
    """
    """
    _type = "wave_detection"
    def __init__(self):
        """
        """
        super(WavingDetectorSkill, self).__init__()
        self._description = "Waving detection control"
        self.last_detection_stamp = [0,0]

    def setup(self):
        self.memory = self.robot.session.service("ALMemory")

        self.waving = self.robot.session.service("ALWavingDetection")
        self.waving_subscriber = self.waving.subscribe("WavingDetectorSkill")

        self.pp = self.robot.session.service("ALPeoplePerception")
        self.pp_subscriber = self.pp.subscribe("WavingDetectionSkill")

        # self.subscriber = self.memory.subscriber("WavingDetection/Waving") #subscribe event memory
        # self.subscriber.signal.connect(self.on_wave) #connect callback

        return True

    def check(self):
        return True
    
    def start(self):
        return True

    def pause(self):
        return None
    
    def shutdown(self):
        return True



    def get_detections(self):
        value = self._get_detection()
        print(value)
        if value is None or value ==[]:
            return []
        else:
            if self.last_detection_stamp == value[0]:
                return []
            else:
                self.last_detection_stamp = value[0]
                print(value)
                confidence = value[1][0][1]
                pose = value[1][0][2]
                return pose


    def _get_detection(self):
        try:
            value = self.memory.getData("WavingDetection/Waving")
            print(value)
        except Exception as e:
            rospy.logerr("Couldn't get information from WavingDetection/Waving")
            value = []
        return value        

    def get_person_waving(self):

        value = self._get_detection()
        if value == []:
            return []
        else:
            id = value[1][3]
            print (str(id))
            return id

    def _get_people_list(self):

        try:
            ids = self.memory.getData("PeoplePerception/PeopleList")
            return ids
        except Exception as e:
            rospy.logerr(e)
            return None
        
    def _get_person_position(self,id):
        """
    
        """
        position = [0,0]

        try: 
            position =  self.memory.getData("PeoplePerception/Person/"+str(id)+"/PositionInRobotFrame")
            return position
        except Exception as e:
            rospy.logerr("Error getting position from person : " + str(id))
            return None