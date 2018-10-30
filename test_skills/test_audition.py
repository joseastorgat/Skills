#!/usr/bin/env python

#Basic ROS
import rospy

#Robot building
from maqui_skills import robot_factory


if __name__ == '__main__':

    rospy.init_node("test_audition")
    maqui = robot_factory.build(["audition",'tts','sound_localization','base'],core=False)
    while not rospy.is_shutdown():
        word = maqui.audition.recognize_with_grammar('/Stage1/GPSR/gpsr')
        print word
        maqui.tts.say(word)
        