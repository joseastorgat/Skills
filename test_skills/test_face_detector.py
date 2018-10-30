#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from maqui_skills import robot_factory
from sensor_msgs.msg import Image
import cv2

from std_srvs.srv import Empty, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import RegionOfInterest
from uchile_srvs.srv import ImageDetectorResponse

class FaceDetector():

    def __init__(self, robot):

        self.robot = robot
        self.image_subscriber = rospy.Subscriber('/maqui/camera/front/image_raw', Image, self._process_image)
        
        self.publisher = rospy.Publisher('/maqui/face_detector', Image, queue_size=10)
        self.bridge = CvBridge()
        self.cv_image = Image()

        self.ff = self.robot.get("facial_features")
        self.ff.start(["detector"])
        self.response = ImageDetectorResponse()

        self.min_area = 20

        self._resolutions = {4:(2560.0,1920.0), 3:(1280.0,960.0), 2:(640.0,480.0), 1:(320.0,240.0), 0:(160.0,120.0), 7:(80.0,60.0), 6:(40.0,30.0)}

        res = int(self.ff._get_resolution())

        print res
        self.xt = 640.0 / self._resolutions[res][0]
        self.yt = 480.0 / self._resolutions[res][1]


    def _process_image(self,img):
        #Se cambiar mensage tipo ros a imagen opencv

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        #Se deja en frame la imagen actual
        frame = self.cv_image

        self.response = self.ff.get_main_face()
        if self.response is not None:
            bboxes = self.response.BBoxes

            for bbox in bboxes:

                if bbox is None:
                    pass
                rospy.loginfo("Face Detected in: {0} {1} {2} {3}".format(bbox.x_offset,bbox.y_offset,bbox.width,bbox.height ))
                #Obtener rectangulo
                x = int(bbox.x_offset * self.xt)
                y = int(bbox.y_offset * self.yt)
                w = int(bbox.width * self.xt)
                h = int(bbox.height * self.yt)

                #Filtrar por area minima
                if w*h > self.min_area:
                    x1 = x
                    y1 = y
                    x2 = x + w
                    y2 = y + h
                    cv2.rectangle(frame, (x1,y1), (x2,y2), (255,0,0), 3)
                rospy.loginfo("Face Detected in: {0} {1} {2} {3}".format(x,y,w,h))

        try:
            new_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        except CvBridgeError as e:
            print(e)

        #Publicar frame
        self.publisher.publish(new_img)

def main():
    
    rospy.init_node("test")
    maqui = robot_factory.build(["facial_features"],core=True)
    maqui.check()
    try:
        FaceDetector(maqui)
        rospy.spin()

    except KeyboardInterrupt:
        maqui.base.stop()   
        print("Finished by user")

if __name__ == '__main__':
    main()
