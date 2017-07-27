#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from maqui_skills import robot_factory

def main():

    #a= "Uh, summa-lumma, dooma-lumma"+"You assumin' I'm a human"+"What I gotta do to get it through to you? I'm superhuman"+"Innovative and I'm made of rubber"+"So that anything you say is ricocheting off of me"+"And it'll glue to you and"+"I'm devastating, more than ever demonstrating"+"How to give a mothafuckin' audience a feelin' like it's levitating"+"Never fading, and I know the haters are forever waiting"+"For the day that they can say I fell off, they'll be celebrating"+"‘Cause I know the way to get 'em motivated"+"I make elevating music, you make elevator music"+"Oh, he's too mainstream."+"Well, that's what they do when they get jealous"+"They confuse it; It's not hip-hop, it's pop."+"‘Cause I found a hella way to fuse it"+"With rock, shock rap with Doc"+"Throw on Lose Yourself and make 'em lose it"+"I don't know how to make songs like that"+"I don't know what words to use."+"Let me know when it occurs to you"+"While I'm rippin' any one of these verses that versus you"+"It's curtains, I'm inadvertently hurtin' you"+"How many verses I gotta murder to"+"Prove that if you were half as nice"+"Your songs you could sacrifice virgins to?"+"Ungh, school flunky, pill junkie"+"But look at the accolades these skills brung me"+"Full of myself, but still hungry"+"I bully myself, ‘cause I make me do what I put my mind to"+"And I'm a million leagues above you"+"Ill when I speak in tongues, but it's still tongue-in-cheek, fuck you!"
    # build the robot

    a= "Hello,  I am Maqui, I am a Pepper Robot from D I E Robotics Lab"
    
    rospy.init_node("test")
    maqui = robot_factory.build(["facial_features","people_perception"],core=True)
    maqui.check()
    
    print("Testing Maqui Robot Skills")
    maqui.tts.set_language("English")
    maqui.tts.set_speed(90)
    maqui.tts.set_volume(0.5)
    
    maqui.basic_motion.wake_up()


    maqui.neck.look_front()

    maqui.tts.say("Starting a facial features Test!, i'm looking for lukitas ")
    maqui.tts.wait_until_done()
    print("lol ")
    maqui.people_perception.reset_population()
    r =rospy.Rate(2.0)
    a = False
    try:
        
        while not a:

            a = maqui.people_perception.is_people_detected()

        ids = maqui.people_perception.get_crowd()
        id = ids[0]

        position = maqui.people_perception.get_position(id)

        x = position[0] - 0.2

        y = position[1]
        print(x,y)
        maqui.base.move(x,y)
        maqui.base.wait_until_done()

        maqui.neck.look_at(x=1.0, z= position[2])

        maqui.tts.say("Hi Lukitas, How are you?")
        maqui.tts.wait_until_done()

        maqui.tts.set_speed(50)
        maqui.tts.say("fuck you meen")
        maqui.base.rotate(180)

    except KeyboardInterrupt:
        maqui.base.stop()
        print("Finished by user")
    # rospy.sleep(5.0)
    # maqui.tts.set_language("English")
    # maqui.tts.set_speed(100)
    
    # rospy.sleep(1.0)
    # maqui.look.look_front()
    # maqui.tts.say(a)
    # maqui.behavior_manager.play_behavior_tag("body language")
    # maqui.behavior_manager.wait_until_done()
    

    # maqui.look.set_vel(0.5)
    
    # maqui.look.look_at(1,1,-0.3)
    # maqui.base.move(0.3,0.3,execution_time = 4)
    # maqui.look.look_at(0.7,0.7,-0.5)
    # maqui.base.move(0.3,0.3,execution_time = 4)
    # rospy.sleep(1.0)
    # maqui.tts.say("Bye")




if __name__ == '__main__':
    main()