#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from maqui_skills import robot_factory

def main():

    #a= "Uh, summa-lumma, dooma-lumma"+"You assumin' I'm a human"+"What I gotta do to get it through to you? I'm superhuman"+"Innovative and I'm made of rubber"+"So that anything you say is ricocheting off of me"+"And it'll glue to you and"+"I'm devastating, more than ever demonstrating"+"How to give a mothafuckin' audience a feelin' like it's levitating"+"Never fading, and I know the haters are forever waiting"+"For the day that they can say I fell off, they'll be celebrating"+"‘Cause I know the way to get 'em motivated"+"I make elevating music, you make elevator music"+"Oh, he's too mainstream."+"Well, that's what they do when they get jealous"+"They confuse it; It's not hip-hop, it's pop."+"‘Cause I found a hella way to fuse it"+"With rock, shock rap with Doc"+"Throw on Lose Yourself and make 'em lose it"+"I don't know how to make songs like that"+"I don't know what words to use."+"Let me know when it occurs to you"+"While I'm rippin' any one of these verses that versus you"+"It's curtains, I'm inadvertently hurtin' you"+"How many verses I gotta murder to"+"Prove that if you were half as nice"+"Your songs you could sacrifice virgins to?"+"Ungh, school flunky, pill junkie"+"But look at the accolades these skills brung me"+"Full of myself, but still hungry"+"I bully myself, ‘cause I make me do what I put my mind to"+"And I'm a million leagues above you"+"Ill when I speak in tongues, but it's still tongue-in-cheek, fuck you!"
    # build the robot

    #a= "Hello,  I am Maqui, I am a Pepper Robot from D I E Robotics Lab"
    
    rospy.init_node("test")
    
    maqui = robot_factory.build(["facial_features","tts","basic_motion","neck"],core=False)
    maqui.check()
    

    maqui.basic_motion.wake_up()

    print("Testing Maqui Robot Skills")
    maqui.tts.set_language("English")
    maqui.tts.set_speed(90)
    maqui.tts.set_volume(0.1)
    

    maqui.neck.look_front()

    dontseeyou = True
    seeyou = True
    maqui.tts.say("Facial Features Test")
    maqui.tts.wait_until_done()
    r =rospy.Rate(3.0)
    

    try:
        while not rospy.is_shutdown():
            
            if maqui.facial_features.is_face_detected:
                if seeyou:
                    #maqui.tts.say("I see your face")
                    face_id = maqui.facial_features._get_face_id()
                    
                    print(str(face_id))
                   
                    name_info = maqui.facial_features.recognize_face()
                    name = name_info[0][2]
                    maqui.tts.wait_until_done()

                    if name !=  "":
                        maqui.tts.say("Hello " + name)
                        maqui.tts.wait_until_done()

                seeyou = False
            
            else:
                seeyou = True

            facial_features = maqui.facial_features.get_facial_features()
            
            if facial_features == []:
                pass
            
            else:
                ages =facial_features[0][0] 
                age = ages[0]
                age_confidence = ages[1]
                
                genders=facial_features[0][1] 

                gender = genders[0]
                gender_confidence = genders[1]
                
                print("AGE : " + str(age) +' , ' + str(age_confidence))
                print("GENDER : " + str(gender) +' , ' + str(gender_confidence))

                gender = {1:"male",0:"female",-1:"lol"}
                           

                g = gender[1]
                maqui.tts.say("You are "+ g + ", and you are " + str(age) + " years old")
                maqui.tts.wait_until_done()
        r.sleep()    
    
    except KeyboardInterrupt:
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