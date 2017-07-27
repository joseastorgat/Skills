#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class PeoplePerceptionSkill(RobotSkill):
    """
    """
    _type = "person_detector"
    def __init__(self):
        """
        People Perception Skill
        """
        super(PeoplePerceptionSkill, self).__init__()
        self._description = "Maqui PeoplePerception Detection skill"

        self.memory = None
        self.people_perception = None

        self.ids = []
        self.people_detected = False

    def setup(self):

        self.people_perception = self.robot.session.service("ALPeoplePerception")
        self.memory = self.robot.session.service("ALMemory")
        
        self.people_subscriber = self.people_perception.subscribe("PeoplePerceptionSkill") # subscribe people_perception service

        self.subscriber = self.memory.subscriber("PeoplePerception/PeopleList") #subscribe event memory
        # self.subscriber.signal.connect(self.people_detection_callback) #connect callback
        return True

    def check(self):
        return True
    
    def start(self):
        return True
    
    def pause(self):
        return True
    
    def shutdown(self):
        self.pause()
        return True

    # def person_detection(self):

    #     poses = []
    #     print(self.ids)
    #     if self.ids == []:
    #         rospy.loginfo("No detections")

    #     for id in self.ids:
    #         pose = self.get_person_position(id)
    #         poses.append(pose)
    #     return poses


    # def is_people_detected(self):
    #     return self.people_detected

    # def people_detection_callback(self,value):
    #     """
    #     Callback for event /PeoplePerception/PeopleList
    #     """

    #     if value == []:  
    #         self.people_detected = False
    #         self.ids  = []
    #     else:  
    #         self.people_detected = True
    #         self.ids = value  

    """
    Extra Methods for Maqui
    """
    def is_people_detected(self):

        self.ids = self.memory.getData("PeoplePerception/PeopleList")
        if self.ids == []:
            return False
        else:
            return True

    def person_detection(self):

        self.is_people_detected()
    
        poses = []

        if self.ids == []:
            rospy.loginfo("No detections")

        for id in self.ids:
            pose = self.get_person_position(id)
            poses.append(pose)
        
        return poses

    def get_crowd(self):
        return  self.ids

    def get_visible_crowd(self):
        """ """

        try:
            visible_crowd = self.memory.getData("PeoplePerception/VisiblePeopleList")
        except Exception as e:
            visible_crowd = []
            rospy.logerr("")
        return visible_crowd

    def get_non_visible_crowd(self):

        try:
            non_visible_crowd = self.memory.getData("PeoplePerception/NonVisiblePeopleList")
        except Exception as e:
            non_visible_crowd = []
            rospy.logerr("")
        return non_visible_crowd

    def get_person_features(self, id,features = ["height","shirt_color"]):
        """
        This method return the specified features from a person id
        """
        person_features = []

        if not id in self.ids:
            rospy.logerr("Person is not in the Crowd")
            return [None,None]

        if "height" in features:
            height = self.memory.getData("PeoplePerception/Person/"+str(id)+"/RealHeight")

            person_features.append(height)

        if "shirt_color" in features:
            shirt_color = self.memory.getData("PeoplePerception/Person/"+str(id)+"/ShirtColor")
            person_features.append(shirt_color)

        return person_features

    def get_people_features(self,features = ["height","shirt_color"]):
        """ 
        Return a list with the features of each person detected by people perception API
        """
        people_features =[]
        
        for id in self.ids:
            a = self.get_person_features(id,features)
            people_features.append(a)
        return people_features

    def get_person_position(self,id):
        """
    
        """
        position = [0,0]
        
        if not id in self.ids:
            rospy.logerr("Person is not in the Crowd")
            return None
        try: 
            position =  self.memory.getData("PeoplePerception/Person/"+str(id)+"/PositionInRobotFrame")
            return position
        except Exception as e:
            rospy.logerr("Error getting position from person : " + str(id))
            return None
    
    def get_distance(self,id):
        
        distance = 0

        if not id in self.ids:
            rospy.logerr("Person is not in the Crowd")
            return None
        distance =  self.memory.getData("PeoplePerception/Person/"+str(id)+"/Distance")
        return distance

    def is_person_visible(self,id):

        if not id in self.ids:
            rospy.logerr("Person is not in the Crowd")
            return False
        visible = self.memory.getData("PeoplePerception/Person/"+str(id)+"/IsVisible")
        return visible

    def presence_time(self,id):

        if not id in self.ids:
            rospy.logerr("Person is not in the Crowd")
            return 0
        time =  self.memory.getData("PeoplePerception/Person/"+str(id)+"/PresentSince")
        return time
    
    def reset_population(self):

        try:
            self.people_perception.resetPopulation()
        except Exception as e:
            rospy.logerr("")


"""
ALPeoplePerception
Configuration:

ALPeoplePerception::setFastModeEnabled
ALPeoplePerception::setGraphicalDisplayEnabled
ALPeoplePerception::setMaximumDetectionRange
ALPeoplePerception::setMovementDetectionEnabled
ALPeoplePerception::setTimeBeforePersonDisappears
ALPeoplePerception::setTimeBeforeVisiblePersonDisappears

"""