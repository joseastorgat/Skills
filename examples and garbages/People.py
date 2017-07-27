#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: A Simple class to get & read FaceDetected Events"""

import qi
import time
import sys
import argparse
from os import environ

class HumanGreeter(object):
    """
    A simple class to react to face detection events.
    """

    def __init__(self):
        """
        Initialisation of qi framework and event detection.
        """
        super(HumanGreeter, self).__init__()

        try:
            # Initialize qi framework.
            connection_url = "tcp://" + environ["robot_ip"] + ":" + environ["robot_port"]
            app = qi.Application(["HumanGreeter", "--qi-url=" + connection_url])
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + environ["robot_ip"] + "\" on port " + environ["robot_port"] +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)


        app.start()
        session = app.session
        # Get the service ALMemory.
        self.memory = session.service("ALMemory")
        # Connect the event callback.
        self.subscriber = self.memory.subscriber("PeoplePerception/PeopleList")
        
        print str(self.subscriber)
        #self.subscriber.signal.connect(self.on_human_tracked)
        # Get the services ALTextToSpeech and ALFaceDetection.
        self.tts = session.service("ALTextToSpeech")
        self.face_detection = session.service("ALPeoplePerception")
        self.face_detection.subscribe("HumanGreeter")
        self.got_face = False

    def on_human_tracked(self, value):
        """
        Callback for event FaceDetected.
        """
        if value == []:  # empty value when the face disappears
            self.got_face = False
        else:  # only speak the first time a face appears
            self.got_face = True
            print "I saw a face!"
            self.tts.say("Hello, you!")
            # First Field = TimeStamp.
            timeStamp = value[0]
            print "TimeStamp is: " + str(timeStamp)

            # Second Field = array of face_Info's.
            PersonInfoArray = value[1]
            for j in range( len(faceInfoArray)-1 ):
                PersonInfo = faceInfoArray[j]

                # First Field = Shape info.
                id = faceInfo[0]

                # Second Field = Extra info (empty for now).
                distance = faceInfo[1]

                print "Person Infos :  id %.3f - distance %.3f" % (faceShapeInfo[0], faceShapeInfo[1])
                print "Person Infos :  pitch %.3f - yaw %.3f" % (faceShapeInfo[2], faceShapeInfo[3])

    def run(self):
        """
        Loop on, wait for events until manual interruption.
        """
        print "Starting HumanGreeter"
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user, stopping HumanGreeter"
            self.face_detection.unsubscribe("HumanGreeter")
            #stop
            sys.exit(0)


if __name__ == "__main__":

    human_greeter = HumanGreeter()
    human_greeter.run()
