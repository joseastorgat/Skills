#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from uchile_skills.robot_skill import RobotSkill
import qi

class PhotoSkill(RobotSkill):

    _type = "photo"

    def __init__(self):
        super(PhotoSkill, self).__init__()

        self.photo = None
        self.memory = None

        self._cameras = {'Top':0,'Bottom':1}
        self._resolutions = {'40 x 30':8,'80 x 60':7,'160 x 120':0,'320 x 240':1,'640 x 480':2,'1280 x 960':3,'2560 x 1920':4}
        self._colorspaces = {'YUV422':9,'BGR':13}
        self._picture_formats = ['bmp', 'dib', 'jpeg', 'jpg', 'jpe', 'png', 'pbm', 'pgm', 'ppm', 'sr', 'ras', 'tiff', 'tif']

        self.resolution = None
        self.colorspace = None
        self.format = None
    
    def check(self, timeout = 1.0):
        return True

    def setup(self):

        self.camera = self.robot.session.service("ALVideoDevice")
        self.set_camera(0)
        self.set_resolution("1280 x 960")
        self.set_colorspace("BGR")
        self.set_format('jpg')
        self.set_capture_interval(200)
        return True

    def shutdown(self):
        return True

    def start(self):
        return True

    def pause(self):
        return True

    def set_colorspace(self,colorspace):

        if colorspace in self._colorspaces:

            try:
                self.photo.setColorspace(self._colorspaces[colorspace])
                return True
            except Exception as e:
                return False
        else:
            return False

    def set_resolution(self,resolution):

        if resolution in self._resolutions:

            try:
                self.photo.setResolution(self._resolutions[resolution])
                return True
            except Exception as e:
                return False
        else:
            return False

    def set_format(self,format):
        
        if format in self._picture_formats:
            try:
                self.photo.setPictureFomat(self.format)
                return True
            except Exception as e:
                return False
        else:
            return False

    def set_camera(self,cameraid):

        if cameraid in [0,1]:
            try:
                self.photo.setCameraID(cameraid)
                return True
            except Exception as e:
                return False
        else:
            return False

    def set_capture_interval(self,interval):

        try:
            self.photo.setCaptureInterval.(interval)
            return True
        except Exception as e:
            return False



    def take_picture(self,folder = None, name = None):

        if folder is None:
            folder = "/home/nao/recordings/photos/"

        if name is None:
            name = "picture"
        
        try:
            self.photo.takePicture(folder, name,True)
            return True
        except Exception as e:
            return False

    def take_pictures(self,folder = None, names = None, picture_number):

        if names is None:
            names = []
            for i in range(0,picture_number):
                names[i] = "picture_"+ str(i)
        
        if folder is None:
            folder = "/home/nao/recordings/photos/"
        try:
            self.photo.takePictures(folder, names , True)
            return True
        except Exception as e:
            return False