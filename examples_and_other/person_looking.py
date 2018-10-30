#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class PersonLookingSkill(RobotSkill):
    """
    """
    _type = "person_looking"
    def __init__(self):
        """
        """
        super(PersonLookingSkill, self).__init__()
        self._description = "Waving detection control"
        self.last_detection_stamp = [0,0]

    def setup(self):
        self.memory = self.robot.session.service("ALMemory")
        self.pp_srv = self.robot.session.service("ALPeoplePerception")
        self.gaze_srv= self.robot.session.service("ALGazeAnalysis")

        self.gaze_subscriber = self.gaze_srv.subscribe("PersonLookingSkill")
        self.pp_subscriber = self.pp_srv.subscribe("PersonLookingSkill")

        return True

    def check(self):
        return True
    
    def start(self):
        return True

    def pause(self):
        return None
    
    def shutdown(self):
        return True

    def get_people_looking(self):

        looking_ids = self._get_people_looking()
        looking_poses = []
        
        if looking_ids == []:
            return []
        else:
            for id in looking_ids:
                pose = self._get_person_position(id)
                looking_poses.append(pose)

            return looking_poses

    def _get_people_looking(self):
        
        looking = []
        try:
            looking = self.memory.getData("GazeAnalysis/PeopleLookingAtRobot")
        except Exception as e:
            rospy.logerr(e)
        return looking


    def get_people_not_looking(self):
        not_looking_ids = self._get_people_not_looking()
        not_looking_poses = []
        
        if not_looking_ids == []:
            return []
        else:
            for id in not_looking_ids:
                pose = self._get_person_position(id)
                not_looking_poses.append(pose)

            return not_looking_poses        

    def _get_people_not_looking(self):

        not_looking = []
        ids = self._get_people_list()

        if ids == []:
            pass
        else:

            for id in ids:
                looking = self.memory.getData("PeoplePerception/Person/"+str(id)+"/IsLookingAtRobot")
                if not looking:
                    print("Person not Looking")
                    not_looking.append(id)
        return not_looking

    # def _get_people_not_looking(self):

    #     people  = self._get_people_list()
    #     looking_people = self._get_people_looking()

    #     for person in looking_people:
    #         try:
    #             people.remove(person)
    #         except:
    #             pass
    #     return people

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