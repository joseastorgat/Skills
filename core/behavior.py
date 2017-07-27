#!/usr/bin/env python
# -*- coding: utf-8 -*-
import qi
import sys
import rospy 
import random
from uchile_skills.robot_skill import RobotSkill


class BehaviorsSkill(RobotSkill):
    """
    Class for using behaviors installed in maqui_robot.
    """
    _type = "behavior_manager"

    def __init__(self):
        """
        """
        super(BehaviorsSkill, self).__init__()
        self.session = None
        self.behavior_manager = None
        self.loaded_behaviors = None

        self.last_behavior = None
    
    def check(self, timeout = 1.0):
        return True
    
    def setup(self):
        self.behavior_manager = self.robot.session.service("ALBehaviorManager")
        return True

    def shutdown(self):
        return True

    def start(self):
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True

    def pause(self):
        return True

    def stop_all(self):
        self.behavior_manager.stopAllBehaviors()

    def run_behavior(self,name):
        """
        launch the behavior and wait until the behavior is finish
        """
        self.last_behavior = name
        return self.behavior_manager.runBehavior(name)

    def start_behavior(self,name):
        """
        launch the behavior
        """
        self.last_behavior = name
        return self.behavior_manager.startBehavior(name)

    def stop_behavior(self,name):
        return self.behavior_manager.stopBehavior(name  )

    def preload_behaviors(self,list_behaviors):

        load = True
        for behavior in list_behaviors:
            try:
                self.behavior_manager.preloadBehavior(behavior)
                self.loaded_behavior.append(behavior)
            except RuntimeError:
                rospy.loginfo("Couldn't load behavior : " + behavior )
                load = False
        return load

    def wait_until_done(self,name=None,timeout = 10):
        begin = rospy.get_time()
        if name is None:
            name = self.last_behavior
        while(not rospy.is_shutdown() and self.behavior_manager.isBehaviorRunning(name)  and (rospy.get_time()-begin) < timeout):
            pass
        return True

    def play_behavior_tag(self,tag):
        """
        http://doc.aldebaran.com/2-1/naoqi/audio/alanimatedspeech_advanced.html'
        """

        behaviors_tag = self.behavior_manager.getBehaviorsByTag(tag)
        behavior_to_play = random.choice(behaviors_tag)
        
        self.last_behavior = behavior_to_play

        rospy.loginfo ( " Playing behavior:" + str(behavior_to_play))

        self.start_behavior(behavior_to_play)
