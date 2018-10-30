#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from maqui_skills import robot_factory

def main():

    a= "Uh, summa-lumma, dooma-lumma"+"You assumin' I'm a human"+"What I gotta do to get it through to you? I'm superhuman"+"Innovative and I'm made of rubber"+"So that anything you say is ricocheting off of me"+"And it'll glue to you and"+"I'm devastating, more than ever demonstrating"+"How to give a mothafuckin' audience a feelin' like it's levitating"+"Never fading, and I know the haters are forever waiting"+"For the day that they can say I fell off, they'll be celebrating"+"‘Cause I know the way to get 'em motivated"+"I make elevating music, you make elevator music"+"Oh, he's too mainstream."+"Well, that's what they do when they get jealous"+"They confuse it; It's not hip-hop, it's pop."+"‘Cause I found a hella way to fuse it"+"With rock, shock rap with Doc"+"Throw on Lose Yourself and make 'em lose it"+"I don't know how to make songs like that"+"I don't know what words to use."+"Let me know when it occurs to you"+"While I'm rippin' any one of these verses that versus you"+"It's curtains, I'm inadvertently hurtin' you"+"How many verses I gotta murder to"+"Prove that if you were half as nice"+"Your songs you could sacrifice virgins to?"+"Ungh, school flunky, pill junkie"+"But look at the accolades these skills brung me"+"Full of myself, but still hungry"+"I bully myself, ‘cause I make me do what I put my mind to"+"And I'm a million leagues above you"+"Ill when I speak in tongues, but it's still tongue-in-cheek, fuck you!"
    # build the robot
    rospy.init_node("test")
    maqui = robot_factory.build([],core=True)
    maqui.check()
    print("Testing Maqui Robot Skills")
    maqui.tts.set_language("English")
    maqui.tts.set_speed(200)
    rospy.sleep(1.0)
    maqui.tts.set_gestures_mode(1)
    maqui.tts.say_with_gestures(a)
    #maqui.base.move_left(0.75,5)

if __name__ == '__main__':
    main()