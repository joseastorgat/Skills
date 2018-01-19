#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill
from geometry_msgs.msg import PoseStamped

class PersonDetectionSkill(RobotSkill):
    """
    """
    _type = "person_detector"
    def __init__(self):
        """
        Person Detector Skill
        """
        super(PersonDetectionSkill, self).__init__()
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
            self.loginfo("[{0}]  Detection start  ".format(PersonDetectionSkill._type))                

            self.__subscriber_name = "PersonDetectionSkill_" + str(rospy.Time.now())
            self.people_subscriber = self.people_perception.subscribe("PersonDetectionSkill") # subscribe people_perception service
            # self.subscriber = self.memory.subscriber("PeoplePerception/PeopleList") #subscribe event memory
            self.__reset_population()
            self.pd_mem_subs.signal.connect(self.__on_people_detected) #connect callback
        except Exception as e:
            self.logerr("[{0}] Detection start failed {1}".format(PersonDetectionSkill._type,e))                

            print(e)
        
        return True
    
    def pause(self):
        try:
            self.loginfo("[{0}] Detection Pause".format(PersonDetectionSkill._type))                

            self.people_perception.unsubscribe(self.__subscriber_name)
            self.pd_mem_subs.signal.disconnect(self.pd_mem_subs)

        except Exception as e:
            self.logerr("[{0}] Detection Pause Failed {1} ".format(PersonDetectionSkill._type,e))                

        return True

    
    def shutdown(self):
        self.pause()
        return True

    def person_detection(self):
        poses = []
        for id in self.ids:
            pose = self._get_person_pose(id)
            if pose is not None:
                poses.append(pose)
        self.loginfo("[{0}] Person detection {1}".format(PersonDetectionSkill._type,poses))                
        return poses

    def _get_person_pose(self,id):
        try: 
            position =  self.memory.getData("PeoplePerception/Person/"+str(id)+"/PositionInRobotFrame")
            print(position)
        except Exception as e:
            self.logerr("[{0}] Error getting position from person  {1}, {2}".format(PersonDetectionSkill._type, id, e ))
            return None
        
        pose = PoseStamped()
        pose.header.stamp=rospy.Time.now()
        pose.header.frame_id = "/maqui" # buscar frame de robot Frame
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = 1
        return pose
    
    def tshirt_detection(self):
        
        label = self.__get_tshirt()        
        total = len(label)
        color = {'black':0, 'white':0, 'red':0, 'blue':0, 'green':0, 'yellow':0}
        for c in label:
            color[c] += 1
        
        self.loginfo("[{0}] T-shirt color detection {1} ".format(PersonDetectionSkill._type,color))                

        return color, total

    def tshirt_pose(self):
        """
        ...
        """
        try:
            poses = self.person_detection()
            label = self.__get_tshirt()
        except rospy.ServiceException, e:
            self.logerr("{0} : Couldn't get people tshirt or poses ".format(PersonDetectionSkill._type))
            return None, None

        self.loginfo("[{0}] Person and color detection".format(PersonDetectionSkill._type))                
            
        return poses, label

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

        personData = value[1]
        self.ids = []

        if personData == []:
            return 
 
        for person in personData:
            self.ids.append(person[0])   
        
        # self.loginfo("[{0}] Detections : {1}".format(PersonDetectionSkill._type, len(self.ids)))                
        # self.logdebug("[{0}] Detections : {1}".format(PersonDetectionSkill._type, self.ids))                

        return

    
    def __reset_population(self):
        
        try:
            self.people_perception.resetPopulation()
        except Exception, e:
            raise e


    def __get_tshirt(self):
        
        label = []
        for id in self.ids:
            tshirt = self.__get_person_tshirt(id)
            if tshirt is not None:
                label.append(tshirt.lower())
        return label

    def __get_person_tshirt(self,id):
        """
    
        """
        try: 
            tshirt =  self.memory.getData("PeoplePerception/Person/"+str(id)+"/ShirtColor")
        except Exception as e:
            self.logwarn("[{0}] Error getting t-shirt color from person  {1}, {2}".format(PersonDetectionSkill._type, id, e ))
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