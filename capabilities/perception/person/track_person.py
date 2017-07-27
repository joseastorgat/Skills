#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class TrackPersonSkill(RobotSkill):
    """
    """
    _type = "track_person"
    def __init__(self):
        """
        
        """
        super(TrackPersonSkill, self).__init__()
        self._description = ""


    def setup(self):
        self.motion_service = self.robot.session.service("ALMotion")
        self.tracker_service = self.robot.session.service("ALTracker")


        self.faceWidth = 0.1
        self.tracker_service.setMode("Head")
        self.vel = 0.5

        self.x = 0.001
        self.y = 0.
        self.z = 0.

        self.max_x = 10.
        self.min_x = 0.001

        self.max_y= 10.
        self.min_y = -10.

        self.max_z = 10.
        self.min_z = -10.

        return True

    def check(self):
        return True
    
    def start(self):
        return True

    def pause(self):
        return None
    
    def shutdown(self):
        return True

    def set_track_mode(self,mode):
        if not mode in ["Head","WholeBody","Move","Navigate"]:
            rospy.logerr("")
            return False
        try:
            self.tracker_service.setMode(mode)
            self.mode = mode
            return True
        except Exception as e:
            return False

    def set_track_effector(self,effector = None):
        
        if not effector in ["Arms","LArm","Rarm",None]:
            rospy.logerr("Invalid Effector name")
            return False
        try:
            self.tracker_service.setEffector(effector)
            return True
        except Exception as e:
            return False

    def track_face(self):

        self.targetName = "Face"
        # self.tracker_service.setRelativePosition([-0.5, 0.0, 0.0, 0.1, 0.1, 0.3])
        
        try:
            self.tracker_service.registerTarget(self.targetName, self.faceWidth)
            self.tracker_service.track(self.targetName)
        
        except Exception as e:
            rospy.logerr("Error starting to trak faces")
            return False



    def stop_track(self):
        try:
            self.tracker_service.stopTracker()

        except Exception as e:
            raise e

    def unregister_targets(self):
        try:
            self.tracker_service.unregisterAllTargets()
        except Exception as e:
            raise e

    def point_at(self,effector,x=1.0,y=0.0,z=0.0):

        if self.isTargetValid(x,y,z):
            try:
                
                self.x = x
                self.y = y
                self.z = z
                self.tracker_service.pointAt(effector,[self.x, self.y, self.z], 2, self.vel)
                
                rospy.loginfo("point at : {0} {1} {2}".format(self.x,self.y,self.z))
                return True                
            except RuntimeError:
                rospy.loginfo("Error")
                return False
        else:
            rospy.logerr("target is not valid")
            return False
    
    def set_arms_vel(self,vel):
        self.vel = vel
    
    def isTargetValid(self,x,y,z):
        x_valid = x>=self.min_x and x<=self.max_x
        y_valid = y>=self.min_y and y<=self.max_y
        z_valid = z>=self.min_z and z<=self.max_z        
        is_valid = (x_valid and y_valid and z_valid) 
        return  is_valid

    def get_target_position(self):

        try:
            position = self.tracker_service.getTargetPosition(2)
            return position
        except Exception as e:
            rospy.logerr("Error getting target position")
            return []

    def get_relative_position(self):
        try:
            position = self.tracker_service.getRelativePosition()
            return position
        except Exception as e:
            rospy.logerr("Error getting target position")
            return []

    def set_search(self,enabled):

        try:
            self.tracker_service.toggleSearch(enabled)
        except Exception as e:
            rospy.logerr("")

    # def is_active(self):
    #     try:
    #         return self.tracker_service.isActive()
    #     except Exception as e:
    #         rospy.logerr("")
    #         return False
    # def is_target_lost(self):
    #     try:
    #         self.tracker_service.isTargetLost()
    #     except Exception as e:
    #         rospy.logerr("")
    #         return False