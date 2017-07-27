#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class BasicAwarenessSkill(RobotSkill):
    """
    """
    _type = "basic_awareness"
    def __init__(self):
        """
        
        """
        super(BasicAwarenessSkill, self).__init__()
        self._description = ""
        self.stimulus = ["People","Touch","TabletTouch","Sound","Movement","NavigationMotion"]

    def setup(self):
        self.memory = self.robot.session.service("ALMemory")
        self.basic_awareness = self.robot.session.service("ALBasicAwareness")
        return True

    def check(self):
        return True
    
    def start(self):
        try:
            self.basic_awareness.setEnabled(True)
        except Exception as e:
            rospy.logerr("Failed to start basic awareness")
    
    def pause(self):
        try:
            self.basic_awareness.pauseAwareness()
        except Exception as e:
            rospy.logerr("Failed to pause basic awareness")

    def resume(self):
        try:
            self.basic_awareness.resumeAwareness()
        except Exception as e:
            rospy.logerr("Failed to resume basic awareness")
    
    
    def stop(self):
        try:
            self.basic_awareness.setEnabled(False)
        except Exception as e:
            rospy.logerr("Failed to start basic awareness")
    

    def shutdown(self):
        self.stop()
        return True

    def active_stimulus(self,stimulus = ["People","Touch","TabletTouch","Sound","Movement","NavigationMotion"]):
        for stim in stimulus:
            if stim in self.stimulus:
                try:
                    self.basic_awareness.setStimulusDetectionEnabled(stim,True)
                    rospy.loginfo("activing stimulues" + stim)
                except Exception as e:
                    rospy.logerr("Failed to active stimulus "+ stim)
    
            else:
                rospy.logerr("Stimulues is no allowed")

    def desactive_stimulus(self,stimulus = ["People","Touch","TabletTouch","Sound","Movement","NavigationMotion"]):
        for stim in stimulus:
            if stim in self.stimulus:
                try:
                    self.basic_awareness.setStimulusDetectionEnabled(stim,False)
                    rospy.loginfo("desactiving stimulues" + stim)                    
                except Exception as e:
                    rospy.logerr("Failed to desactive basic awareness")
    
            else:
                rospy.logerr("Stimulues is no allowed")

    def set_engagement_mode(self,mode):

        if mode in ["Unengaged","FullyEngaged","SemiEngaged"]:
            try:
                self.basic_awareness.setEngagementMode(mode)
                rospy.loginfo("setting engagement mode: "+ mode)
            except Exception as e:
                raise e


    def engage_person(self,person_id):
        try:
            self.basic_awareness.engageperson(person_id)
        except Exception as e:
            raise e

    def set_track_mode(self,mode):
        if not mode in ["Head","WholeBody","Move","Navigate"]:
            rospy.logerr("")
            return False
        try:
            self.basic_awareness.setMode(mode)
            self.track_mode = mode
            rospy.loginfo("setting track mode as" + mode)
            return True
        except Exception as e:
            return False

    def look_around(self):

        self.active_stimulus(["Sound","Touch","People"])
        self.set_engagement_mode("Unengaged")
        self.set_track_mode("Head")
        self.start()

