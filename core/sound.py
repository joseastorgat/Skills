#!/usr/bin/env python
# -*- coding: utf-8 -*-
import qi
import sys
import rospy 
import time
from uchile_skills.robot_skill import RobotSkill


class SoundSkill(RobotSkill):
    """
    Base class for sound play.
    """
    _type = "sound"

    def __init__(self):
        """
        Base class for sound play.
        """
        super(SoundSkill, self).__init__()

        self.session = None

        self.sound = None
        self._description = "Sound play skill"
        self.fileID = None

        self.memory = None
        
    def check(self, timeout = 1.0):

        return True
    
    def setup(self):
        self.sound = self.robot.session.service("ALAudioPlayer")
        return True

    def shutdown(self):
        self.stop()
        self.sound.unloadAllFiles()

        return True

    def start(self):
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True

    def pause(self):
        """
        Pause sound

        Examples:
            >>> robot.sound.pause()
        """
        self.sound.pause(self.fileID)
        return True

    def play(self, filename=None,async = True):
        """
        Play sound

        Args:
            filename (String): Filename in folder /home/nao/naoqi/sounds

        Examples:
            >>> robot.sound.play("tone")
        """
        if filename is not None:
            self.fileID = self.sound.loadFile("/home/nao/naoqi/sounds/" + filename)
            rospy.loginfo("Loaded file: "+ filename)
        if self.fileID is not None:
            rospy.loginfo("Playing file")
            self.sound.play(self.fileID, _async=async)
        else:
            rospy.loginfo("There is not a Sound File Loaded!")
            return False
        return True

    def stop(self):
        try:
            self.sound.stopAll()
        except RuntimeError:
            rospy.loginfo("Can't Stop SoundSkill")

    def set_volume(self,volume=1.0):
        """
        set the volume of sound player 
        volume between 0 and 1
        """
        self.sound.setMasterVolume(volume)

