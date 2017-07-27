#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
from os import environ
import qi 
from uchile_skills.robot_factory import build_robot
import types
import rospy

_core_skills = ['face','base','sound','tts','neck','tablet','behavior_manager','camera','basic_motion']


def get_qi_session():

    connection_url="tcp://" + environ["robot_ip"] + ":" + environ["robot_port"]
    try:
        # Initialize qi framework.
        app = qi.Application(["Maqui Robot", "--qi-url=" + connection_url])        
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + environ["robot_ip"] + "\" on port " + environ["robot_port"] +".\n"
                "Please check your script arguments. Run with -h option for help.")
        return None

    app.start()
    return app.session

def set_session(self):
    self.session = get_qi_session()
    if self.session is None:
        rospy.loginfo("Setup Failed: naoqi connection Failed")
        return False
    return True

def build(skills=[], core=True, autodeps=True, check=True, setup=True):
    """
    builds a robot with the desired skills
    the user has no need to import the skills, just know their names.

    By default, all robots have the core skills, listed on '_core_skills'.
    In order to override this behaviour, use core=False.

    see also: uchile_skills.robot_factory.build()
    """
    _skills = set(skills)
    if core:
        _skills |= set(_core_skills)


    maqui = build_robot(list(_skills), "maqui", autodeps, check, setup=False)

    maqui.set_session = types.MethodType(set_session,maqui)

    if not maqui.set_session():
        raise RuntimeError("Robot setup failed")

    if not maqui.setup():
        raise RuntimeError("Robot setup failed")

    return maqui