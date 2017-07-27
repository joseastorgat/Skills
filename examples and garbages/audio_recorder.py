#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from uchile_skills.robot_skill import RobotSkill
import qi

class AudioRecorderSkill(RobotSkill):

	_type = "audio_recorder"

	def __init__(self):
		super(AudioRecorderSkill, self).__init__()


	def check(self, timeout = 1.0):
		return True

	def setup(self):
		return True

	def shutdown(self):
		return True

	def start(self):
		return True

	def pause(self):