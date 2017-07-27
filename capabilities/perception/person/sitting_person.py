#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class SittingPersonSkill(RobotSkill):
    """
    """
    _type = "sitting_person_detector"
    def __init__(self):
        """
        """
        super(SittingPersonSkill, self).__init__()
        self._description = "Waving detection control"
        self.last_detection_stamp = [0,0]

    def setup(self):
        self.memory = self.robot.session.service("ALMemory")
        self.pp_srv = self.robot.session.service("ALPeoplePerception")
        self.sitting_srv= self.robot.session.service("ALSittingPeopleDetection")

        self.sitting_subscriber = self.sitting_srv.subscribe("SittingPersonSkill")
        self.pp_subscriber = self.pp_srv.subscribe("SittingPersonSkill")

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

        sitting_ids = self._get_detections()
        sitting_poses = []
        
        if sitting_ids == []:
            return []
        else:
            for id in sitting_ids:
                pose = self._get_person_position(id)
                sitting_poses.append(pose)

            return sitting_poses

    def _get_detections(self):

        ids = self._get_people_list()
        sitting_ids = []
        
        if ids is None:
            rospy.loginfo("No person detected")
            pass
        else:
            for person in ids:
                try:
                    sitting = self.memory.getData("PeoplePerception/Person/"+str(person)+"/IsSitting")
                except Exception as e:
                    rospy.logerr(e)
                    return sitting_ids                    
                if sitting == 1:
                    print("Person Sitting")
                    sitting_ids.append(person)
                if sitting == 0:
                    print("Person Standing")

        return sitting_ids


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