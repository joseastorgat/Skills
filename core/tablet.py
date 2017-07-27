#!/usr/bin/env python
# -*- coding: utf-8 -*-
import qi
import sys
import rospy 
import math
from uchile_skills.robot_skill import RobotSkill

class TabletSkill(RobotSkill):
    """
    Base class for Tablet
    """
    _type = "tablet"

    def __init__(self):
        """
        Base class for sound play.
        """
        super(TabletSkill, self).__init__()

        self.tablet = None
        self._description = "Tablet skill"

        self.last_webpage = None
        self.last_image = None
        self.last_video = None

        self._wifi = None
        #self._webView = False

    def check(self, timeout = 1.0):
        return True
    
    def setup(self):

        self.tablet = self.robot.session.service("ALTabletService")                
        #self.tablet.enableWifi()
        
        #self._wifi = self.tablet.connectWifi("bendernet")
        
        # if not self._wifi:
        #     rospy.loginfo("Tablet is not connected to Wifi! won't show web pages")
        # else:
        #     rospy.loginfo("Tablet connected to bendernet wifi")
        return True

    def shutdown(self):
        return True

    def start(self):
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True

    def stop(self):
        return True

    def set_brightness(self,brightness):

        if brightness>0 and brightness<=1:
            rospy.loginfo("Tablet brightness set at {0}".format(brightness))
            return self.tablet.setBrightness(brightness)

        rospy.loginfo("Couldn't change Tablet brightness: Brightness must be a float value between 0 and 1")
        return False


    def set_volume(self,volume=1.0):
        """
        set the volume of the tablet 
        volume between 0 and 1
        volume of tablet is a positive integer between 0 and 15
        """
        tablet_volume = int(math.floor(volume*15))
        return self.tablet.setVolume(tablet_volume)
    

    def sleep(self):
        """
        Put the tablet in sleep mode (standby mode).
        """
        return self.tablet.goToSleep()
    
    def wake_up(self):
        """
        Wake the tablet (from standby mode).
        """
        return self.tablet.wakeUp()

    def screen_on(self):
        """
        """
        return self.tablet.turnScreenOn(True)

    def screen_off(self):
        return self.tablet.turnScreenOn(False)    

   
    def hide_content(self):
        """
        show the default screenSaver
        """
        return self.tablet.hide()
 
    def load_webpage(self,url):
        """
        load the specified web page but don't show it in the tablet screen)
        Note: Url format should be "https://www...."
        Example: "https://www.youtube.com" 
        """

        last_webpage = url

        if not self._wifi:
            rospy.loginfo("Tablet is not connected to Wifi! won't show web pages")
        return self.tablet.loadUrl(url)
    
    def show_webpage(self,url=None):
        """
        show a web page in the tablet screen
        if Url is None show a previously load url
        """
        if url is None:
            return self.tablet.showWebview()
        else:
            self.load_webpage(url)
            self.tablet.showWebview()
            return True
    
    def show_image(self,url=None):
        """
        show a image from a url
        """
        if url is None:
            url = self.last_image

        else:
            self.last_image = url 
        
        if not self._wifi:
            rospy.loginfo("Tablet is not connected to Wifi! won't show web pages")
        return self.tablet.showImage(url)    

    def set_background(self,color):
        """
        Set image background color.
        Color: string that contains hexadecimal color code, from “#000000” to “#FFFFFF”.
        """
        return self.tablet.setBackgroundColor(color)

    # def pauseGif(self):
    #     """
    #     Pause current gif displayed.
    #     """
    #     return self.tablet.pauseGif()

    # def resumeGif(self):
    #     """
    #     Resume current gif displayed.
    #     """
    #     return self.tablet.resumeGif()

    def play_video(self,url = None):
        """
        Play a video from a url
        """
        if url is None:
            url = self.last_video

        last_video = url

        return self.tablet.playVideo(url)

    def pause_video(self):
        """
        Pause the current playing video (do not close the Video Player)
        """
        return self.tablet.pauseVideo()

    def resume_video(self):
        """
        resume the current paused video
        """
        return self.tablet.resumeVideo()
    
    def stop_video(self):
        """
        Close the video Player
        """
        return self.tablet.stopVideo()

    
#    def active_touchscreen(self):


#    def ontouchcoordinates(self)

# self.tablet._launchApk

