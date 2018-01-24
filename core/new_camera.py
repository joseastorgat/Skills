#!/usr/bin/env python
# -*- coding: utf-8 -*-
import qi
import sys
import rospy
from uchile_skills.robot_skill import RobotSkill

class CameraSkill(RobotSkill):

    _type = "camera"

    def __init__(self):
        super(CameraSkill, self).__init__()

        self._subscriber_name = "CameraSkill"
        
        self.__path = "/home/nao/media/"
        self._cameras = {'top':0,'bottom':1}
        self._resolutions = {'40x30':8,'80x60':7,'160x120':0,'320x240':1,'640x480':2,'1280x960':3,'2560x1920':4}
        self._colorspaces = {"Y":0, 'YUV422':9,'YUV':10,'RGB':11,'HSY':12,'BGR':13}
        self._picture_formats = ["bmp", "dib", "jpeg", "jpg", "jpe", "png", "pbm", "pgm", "ppm", "sr", "ras", "tiff", "tif"]
        self._video_formats = ["MJPG","IYUV"]

        self._camera_configuration = { "resolution":self._resolutions["640x480"], "colorspace": self._colorspaces["BGR"] , "framerate": 10}
        self._picture_configuration = {"resolution":self._resolutions["1280x960"], "colorspace": self._colorspaces["BGR"] , "format": "jpeg"}
        self._video_configuration = {"resolution":self._resolutions["640x480"], "colorspace": self._colorspaces["BGR"], "framerate": 10 , "videoformat":"MJPG" }

    def check(self, timeout = 1.0):
        return True
    
    def setup(self):
        self.camera = self.robot.session.service("ALVideoDevice")
        return True

    def shutdown(self):
        
        return success

    def start(self, type = ["picture"]):

        #self.camera_subscriber = self.camera.subscribeCamera(self.subscriberID,self.active_camera,self.resolution,self.colorspace,self.fps)
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True

    def pause(self):
        return True

    def set_camera(self):
        
        if not (camera=="top" or camera=="bottom"):
            self.logwarn("Camera param must be : 'top'  or 'bottom'. USING TOP BY DEFAULT!")
            camera = "top"
        
        self._picture_srv.setCameraID(self._picture_configuration["camera"])
        self._video_srv.setCameraID(self._picture_configuration["camera"])

    def set_picture_params(self, camera="top", resolution="1280x960", colorspace=["BGR"], pic_format = "jpeg"):
        
        if not (camera=="top" or camera=="bottom"):
            self.logwarn("Camera param must be : 'top'  or 'bottom'. USING TOP BY DEFAULT!")
            camera = "top"
        if not resolution in self._resolutions
            self.logwarn("Resolutions available are " + self._resolutions+". USING '1280X960' BY DEFAULT!")
            resolution="1280x960"
        if not (colorspace=="Y" or colorspace=="BGR"):
            self.logwarn("colorspace must be 'BGR' (color pictures)  or  'Y' (gray-scale pictures). USING COLOR PICTURE BY DEFAULT ")
            colorspace="BGR"
        if not (pic_format in self._picture_formats or pic_format.lower() in self._picture_formats):
            self.logwarn("Picture Formats available are " + self._picture_formats+". USING 'jpeg' BY DEFAULT!")
            pic_format="jpeg"

        self._picture_configuration["camera"] = self._cameras[camera]
        self._picture_configuration["colorspace"] = self._colorspaces[colorspace]
        self._picture_configuration["resolution"] = self._resolutions[resolution]
        self._picture_configuration["format"] = pic_format
        return self.__update_picture_params()

    def print_params(self):
        for param, value in self._picture_configuration:
            print "["+param+"]"+str(value)
        return 

    def __update_picture_params(self):
        try:
            self._picture_srv.setResolution(self._picture_configuration["resolution"])
            self._picture_srv.setColorSpace(self._picture_configuration["colorspace"])
            self._picture_srv.setPictureFormat(self._picture_configuration["format"])
        except Exception as e:
            raise e
        return True
    
    def take_picture(self, name):
        
        name = name + "_"+str(rospy.Time.now())
        try:
            self._picture_srv.takePicture(self.__path, name, False)
        except Exception as e:
            raise e

    def take_pictures(self, name="", interval=200, number=10   ):
        self._picture_srv.setInterval(interval)        
        name = name + "_"+str(rospy.Time.now())

        try:
            self._picture_srv.takePictures(self.__path, name, number, False)
        except Exception as e:
            raise e


    def set_video_params(self, camera="top", resolution="1280x960", colorspace=["BGR"], video_format = "MJPG", fps = 10 ):
        
        if not resolution in self._resolutions
            self.logwarn("Resolutions available are " + self._resolutions+". USING '1280X960' BY DEFAULT!")
            resolution="1280x960"
        if not (colorspace=="Y" or colorspace=="BGR"):
            self.logwarn("colorspace must be 'BGR' (color pictures)  or  'Y' (gray-scale pictures). USING COLOR PICTURE BY DEFAULT ")
            colorspace="BGR"
        if not (video_format in self._video_formats or video_format.upper() in self._video_formats):
            self.logwarn("Video Formats available are " + self._video_formats+". USING 'MJPG' BY DEFAULT!")
            video_format="MJPG"
        if fps>30 or fps<0 or type(fps)!=int:
            self.logwarn("Video Fps must be an integer between 0 and 30 . USING '10' BY DEFAULT!")
            fps=10

        self._video_configuration["colorspace"] = self._colorspaces[colorspace]
        self._video_configuration["resolution"] = self._resolutions[resolution]
        self._video_configuration["format"] = video_format
        self._video_configuration["fps"] = fps

        return self.__update_video_params()

    def print_video_params(self):
        for param, value in self._video_configuration:
            print "["+param+"]"+str(value)
        return 

    def __update_picture_params(self):
        try:
            self._video_srv.setResolution(self._video_configuration["resolution"])
            self._video_srv.setColorSpace(self._video_configuration["colorspace"])
            self._video_srv.setVideoFormat(self._video_configuration["format"])
            self._video_srv.setFrameRate(self._video_configuration["fps"])
        except Exception as e:
            raise e
        return True

    def start_video_record(self,name):
        name = name + "_"+str(rospy.Time.now())
        try:
            if not self._video_srv.isRecording():
                self._video_srv.startRecording(name)
                self.loginfo("Start Video Recording with name: " + name)
            else:
                self.logwarn("Tried to start video recording but another video record is in process")

        except Exception as e:
            raise e
    
    def stop_video_record(self,name):
        try:
            if not self._video_srv.isRecording():
                self._video_srv.stopRecording()
                self.loginfo("Video recording Stopped")
            else:
                self.logwarn("Tried to stop video recording but not video recording in process")
        except Exception as e:
            raise e