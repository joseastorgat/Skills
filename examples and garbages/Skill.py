#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class Skill(RobotSkill):
    """
    """
    _type = ""
    def __init__(self):
        """
        
        """
        super(Skill, self).__init__()
        self._description = ""


    def setup(self):

        return True

    def check(self):
        return True
    
    def start(self):
        return True

    def pause(self):
        return None
    
    def shutdown(self):
        return True
