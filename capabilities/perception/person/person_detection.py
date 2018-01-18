#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill
from geometry_msgs.msg import PoseStamped

class PeoplePerceptionSkill(RobotSkill):
    """
    """
    _type = "person_detector2"
    def __init__(self):
        """
        Person Detector Skill
        """
        super(PeoplePerceptionSkill, self).__init__()
        self._description = "Person detection skill based in Naoqi Apis"
        self.ids = []
    def setup(self):

        self.people_perception = self.robot.session.service("ALPeoplePerception")
        self.memory = self.robot.session.service("ALMemory")
        self.pd_mem_subs = self.memory.subscriber("PeoplePerception/PeopleDetected")
        return True

    def check(self):        
        return True
    
    def start(self):
        try:

            self.__subscriber_name = "PersonDetectionSkill_" + str(rospy.Time.now())
            self.people_subscriber = self.people_perception.subscribe("PeoplePerceptionSkill") # subscribe people_perception service
            # self.subscriber = self.memory.subscriber("PeoplePerception/PeopleList") #subscribe event memory
            self.__reset_population()
            self.pd_mem_subs.signal.connect(self.__on_people_detected) #connect callback
        except Exception as e:
            print(e)
        
        return True
    
    def pause(self):
        try:

            self.people_perception.unsubscribe(self.__subscriber_name)
            self.pd_mem_subs.signal.disconnect(self.pd_mem_subs)

        except Exception as e:
            print(e)
        return True

    
    def shutdown(self):
        self.pause()
        return True

    def person_detection(self):

        poses = []
        if self.ids != []:
            for id in self.ids:
                pose = self.__get_person_pose(id)
                if pose is not None:
                    poses.append(pose)
        return poses

    def tshirt_detection(self):
        
        label = self.__get_people_tshirt()

        total = len(label)
        color = {'black':0, 'white':0, 'red':0, 'blue':0, 'green':0, 'yellow':0}
        for c in label:
            color[c] += 1

        return color, total
    """
    Extra Methods for Maqui
    """
    def __on_people_detected(self,value):
# [
#   [TimeStamp_Seconds, TimeStamp_Microseconds],
#   [PersonData_1, PersonData_2, ... PersonData_n],
#   CameraPose_InTorsoFrame,
#   CameraPose_InRobotFrame,
#   Camera_Id
# ]

# PersonData_i =
# [
#   Id,
#   DistanceToCamera,
#   PitchAngleInImage,
#   YawAngleInImage
# ]

        PersonData = value[1]
        self.ids = []

        if personData == []:
            return 
 
        for person in PersonData:
            self.ids.append(person[0])           
        return
    
    def __reset_population(self):
        try:
            self.people_perception.resetPopulation()
        except Exception, e:
            raise e

    def __get_person_pose(self,id):
        """
    
        """
        try: 
            position =  self.memory.getData("PeoplePerception/Person/"+str(id)+"/PositionInRobotFrame")
        except Exception as e:
            rospy.logerr("Error getting position from person : " + str(id))
            return None
        
        pose = PoseStamped()
        pose.header.stamp=rospy.Time.now()
        pose.header.frame_id = "/maqui" # buscar frame de robot Frame
        pose.pose.Point.x = position[0]
        pose.pose.Point.y = position[1]
        pose.pose.Point.z = position[2]
        pose.pose.Quaternion.w = 1
        return pose

    def __get_people_tshirt(self):

        label = []
        if self.ids != []:
            for id in self.ids:
                tshirt = self.__get_person_tshirt(id)
                if tshirt is not None:
                    label.append(tshirt)
        return label
    
    def __get_person_tshirt(self,id):
        """
    
        """
        try: 
            tshirt =  self.memory.getData("PeoplePerception/Person/"+str(id)+"/ShirtColor")
        except Exception as e:
            rospy.logerr("Error getting position from person : " + str(id))
            return None        
        return tshirt



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