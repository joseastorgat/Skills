#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use Tracking Module to Track an Object"""

import qi
import argparse
import sys
import time
from os import environ

def main(session):
    """
    This example shows how to use ALTracker to track an object with trackEvent api.
    This example is only a subscriber. You need to create another script to raise the tracked event.
    Your events should follow this structure :
        EventNameInfo {
          TargetPositionInFrameWorld,
          TimeStamp,
          EffectorId,
          HeadThreshold (optional)
          }
    All details are available in ALTracker API Documentation.
    """
    # Get the services ALTracker, ALMotion and ALRobotPosture.

    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    tracker_service = session.service("ALTracker")
    memory_service = session.service("ALMemory")

    # First, wake up.
    motion_service.wakeUp()

    fractionMaxSpeed = 0.8

    # Go to posture stand
    posture_service.goToPosture("StandInit", fractionMaxSpeed)
    
    targetName = "Face"
    faceWidth = 0.1
    tracker_service.registerTarget(targetName, faceWidth)
    
    
    # Set target to track.
    eventName = "ALTracker/FaceDetected"

    # set mode
    mode = "Navigate"
    tracker_service.setMode(mode)

    # Set the robot relative position to target
    # The robot stays a 50 centimeters of target with 10 cm precision
    tracker_service.setRelativePosition([-0.5, 0.0, 0.0, 0.1, 0.1, 0.3])

    # Then, start tracker.
    #tracker_service.trackEvent(eventName)
    tracker_service.track(targetName)

    print "ALTracker successfully started."
    print "Use Ctrl+c to stop this script."

    try:
        while True:
            time.sleep(1)
            a=memory_service.getData("ALTracker/FaceDetected")
            print(a)
    except KeyboardInterrupt:
        print
        print "Interrupted by user"
        print "Stopping..."

    # Stop tracker, go to posture Sit.
    tracker_service.stopTracker()
    tracker_service.unregisterAllTargets()
    posture_service.goToPosture("Sit", fractionMaxSpeed)
    motion_service.rest()

    print "ALTracker stopped."

if __name__ == "__main__":

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
    main(session)