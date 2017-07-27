#!/usr/bin/env python
# -*- coding: utf-8 -*-
import qi
import sys
import rospy
from uchile_skills.robot_skill import RobotSkill
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import Image
import ImageDraw

class CameraSkill(RobotSkill):

    _type = "camera"

    def __init__(self):
        super(CameraSkill, self).__init__()

        self.session = None

        self.camera = None
        self.bridge = CvBridge()
        
        self._cameras = {'Top':0,'Bottom':1}
        self._resolutions = {'40 x 30':8,'80 x 60':7,'160 x 120':0,'320 x 240':1,'640 x 480':2,'1280 x 960':3,'2560 x 1920':4}
        self._colorspaces = {'YUV422':9,'YUV':10,'RGB':11,'HSY':12,'BGR':13}
        
        self.subscriber_name = "CameraSkill"
        self.camera_subscriber = None

        self.active_camera = None
        self.resolution = None
        self.fps = None
        self.colorspace = None


    def check(self, timeout = 1.0):
        return True
    
    def setup(self):
        self.camera = self.robot.session.service("ALVideoDevice")

        self.active_camera = 0
        self.resolution = 2
        self.fps = 5
        self.colorspace = 11

#        self.subscriberID = self.camera.subscribeCamera(self.subscriber_name,self.active_camera,self.resolution,self.colorspace,self.fps)

        #self.set_active_camera(0)
        #self.set_resolution('640 x 480')
        #self.set_fps(5) 
        #self.set_colorspace("RGB")
        
        #initialize cameras
        self.camera.openCamera(self.active_camera)
        self.camera.startCamera(self.active_camera)
        
        return True

    def shutdown(self):
        
        success = self.camera.unsubscribe(self.subscriberID)
        return success

    def start(self):

        self.camera_subscriber = self.camera.subscribeCamera(self.subscriberID,self.active_camera,self.resolution,self.colorspace,self.fps)
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True

    def pause(self):
        return True

    def set_active_camera(self,cameraid):
   
        self.active_camera = cameraid

        return self.camera.setActiveCamera(self.subscriberID, cameraid)
    
    def set_resolution(self,resolution):
   
        self.resolution=self._resolutions[resolution]
        return self.camera.setResolution(self.subscriberID, self.resolution)

    def set_fps(self,fps):
        self.fps = fps
        return self.camera.setFrameRate(self.subscriberID, self.fps)

    def set_colorspace(self,colorspace):
        self.colorspace = self._colorspaces[colorspace]

        return self.camera.setColorSpace(self.subscriberID, self.colorspace)
    
    def _get_binary_image(self):
        
        try: 
            image_container = self.camera.getImageRemote(self.subscriberID)
            if image_container is None:
                rospy.loginfo("no imagen")
                return None

        except Exception as e:
            rospy.loginfo("Couldn't get image from Camera, Exception {0}".format(e))
            return None

        self.camera.releaseImage(self.subscriberID)
        return image_container

    def get_last_image(self):

        image_container = self._get_binary_image()
        if image_container is None:
            return None
        bin_image = image_container[6]
        width = int(image_container[0])
        height = int(image_container[1])

        im=Image.new('RGB',(width,height),(0,0,255))
        
        draw=ImageDraw.Draw(im,'RGB')
        lenb = len(bin_image)/3
        for i in range(0,lenb):
            j = 3*i
            x = i%(width)
            y = i//(height)
            value = (bin_image[j],bin_image[j+1],bin_image[j+2])
            draw.point((x,y),value)
        im.show()
        return im
"""
From here methods are inherits from bender skill camera
"""
    # def imgmsg_to_cv2(self, msg):
    #     if msg is None:
    #         rospy.logwarn("Invalid image 'None'")
    #         return None
    #     try:
    #         return self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #     except CvBridgeError as e:
    #         return None

    # def cvimg_to_file(self, cv_image, filename):
    #     if cv_image is None:
    #         rospy.logwarn("Invalid image 'None'")
    #         return False
    #     try:
    #         cv2.imwrite(filename, cv_image)
    #         rospy.loginfo("Successfully saved image to {}".format(filename))
    #         return True
    #     except:
    #         rospy.logwarn("Failed to save image to {}".format(filename))
    #         return False

    # def imgmsg_to_file(self, msg, filename):
    #     if msg is None:
    #         rospy.logwarn("Invalid image 'None'")
    #         return False

    #     image = self.imgmsg_to_cv2(msg)
    #     if image is None:
    #         rospy.logwarn("Failed to convert msg to cv2 image")
    #         return False
    #     return self.cvimg_to_file(image, filename)

    # def apply_effect_cartoon(self, cv_image):
    #     if cv_image is None:
    #         return None

    #     num_down = 2  # number of downsampling steps
    #     num_bilateral = 7  # number of bilateral filtering steps

    #     # downsample image using Gaussian pyramid
    #     img_color = cv_image
    #     for _ in xrange(num_down):
    #         img_color = cv2.pyrDown(img_color)

    #     # repeatedly apply small bilateral filter instead of
    #     # applying one large filter
    #     for _ in xrange(num_bilateral):
    #         img_color = cv2.bilateralFilter(img_color, d=9,
    #                                         sigmaColor=9,
    #                                         sigmaSpace=7)

    #     # upsample image to original size
    #     for _ in xrange(num_down):
    #         img_color = cv2.pyrUp(img_color)

    #     # convert to grayscale and apply median blur
    #     img_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    #     img_blur = cv2.medianBlur(img_gray, 7)

    #     # detect and enhance edges
    #     img_edge = cv2.adaptiveThreshold(img_blur, 255,
    #                                      cv2.ADAPTIVE_THRESH_MEAN_C,
    #                                      cv2.THRESH_BINARY,
    #                                      blockSize=9,
    #                                      C=2)

    #     # convert back to color, bit-AND with color image
    #     img_edge = cv2.cvtColor(img_edge, cv2.COLOR_GRAY2RGB)
    #     img_cartoon = cv2.bitwise_and(img_color, img_edge)

    #     return img_cartoon

    # def save_last_img(self, filename):
    #     msgimage = self.get_last_image()
    #     if msgimage is None:
    #         return False
    #     cv_image = self.imgmsg_to_cv2(msgimage)
    #     if cv_image is None:
    #         return False
    #     self.cvimg_to_file(cv_image, filename)

    # def display(self, cv_image, timeout=0):
    #     if cv_image is None:
    #         rospy.logwarn("Invalid image 'None'")
    #         return False
    #     cv2.imshow("Image window", cv_image)
    #     cv2.waitKey(timeout)
    #     return cv_image

    # def get_path(self, filename):
    #     return os.path.abspath(filename)