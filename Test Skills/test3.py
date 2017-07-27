#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from maqui_skills import robot_factory

def main():

    # build the robot

    a= "Hello,  I am Maqui, I am a Pepper Robot from D I E Robotics Lab"
    
    rospy.init_node("test")
    maqui = robot_factory.build(["facial_features","people_perception",'track_person'],core=True)
    maqui.check()
    
    print("Testing Maqui Robot Skills")
    maqui.tts.set_language("English")
    maqui.tts.set_speed(90)
    maqui.tts.set_volume(0.5)
    
    maqui.basic_motion.wake_up()


    maqui.neck.look_front()

    maqui.tts.say("Starting a track test, trackig lukitas ")
    maqui.tts.wait_until_done()
    print("lol ")
    maqui.people_perception.reset_population()
    r =rospy.Rate(2.0)
    a = False
    try:
        
        while not a:

            a = maqui.people_perception.is_people_detected()
            if a: 
                print('Person detected!')
        ids = maqui.people_perception.get_crowd()
        id = ids[0]

        maqui.track_person.track_person(id)
        print ("Starting Tracks")

        try:
            while True:
                rospy.sleep(1)
        except KeyboardInterrupt:
            print
            print "Interrupted by user"
            print "Stopping..."

        maqui.tts.say("Hi Lukitas, How are you?")
        maqui.base.rotate(180)

    except KeyboardInterrupt:
        maqui.base.stop()   
        print("Finished by user")
        maqui.track_person.unregister_targets()
        maqui.track_person.stop_track()


if __name__ == '__main__':
    main()