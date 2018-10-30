#!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill

class FaceDetectionSkill(RobotSkill):
    """
    """
    _type = "FaceDetection"
    def __init__(self):
        """
        Base Movement Skill
        """
        super(FaceDetectionSkill, self).__init__()
        self._description = "Maqui FaceDetection skill"

        self.memory = None
        self.face_detection = None

        self.face = []
        self.last_face = []
    
        self.is_tracking = False
        self.is_recognizing = False

    def setup(self):

        self.face_detection = self.robot.session.service("ALFaceDetection")
        self.memory = self.robot.session.service("ALMemory")

        self.start()
        
        self.subscriber = self.memory.subscriber("FaceDetected")
        self.subscriber.signal.connect(self.face_callback)

        self.set_recognition(True)
        self.set_tracking(False)
        return True

    def check(self):
        return True
    

    def start(self):
        self.face_detection.subscribe("FaceDetectionSkill")
        return True

    def pause(self):
        self.face_detection.unsubscribe("FaceDetectionSkill")
        return None
    
    def shutdown(self):
        self.pause()

        return True

    
    def face_callback(self,value):
                """
        Callback for event FaceDetected.
        """

        if value == []: 
            self.got_face = False
        else : 
            self.got_face = True

        self.face = value  

    def is_face_detected(self,timeout = 2.0):
        """
        evaluar si es necesario esta función o entregar las caras directamente del callback     
        """
        begin = rospy.get_time()
        counter = 0
        self.last_face = []

        while(not rospy.is_shutdown() and (rospy.get_time()-begin) < timeout):
            
            if self.face != last_face and self.face != []:
                counter +=1
            self.last_face = self.face

            if counter == 3:
                return True

        return False

    def get_face_number(self, timeout = None):
        if self.is_face_detected():
            face_number = len ( self.last_face[1])-1
        else:
            face_number = 0
        return face_number

    
    def learn_face(self,name):
        self.face_detection.learnFace(name)

    def relearn_face(self,name):
        self.face_detection.reLearnFace(name)

    def remove_face(self,name):
        self.face_detection.forgetPerson(name)

    def get_known_faces(self):
        self.face_detection.getLenedFacesList()

    def set_recognition(self,bool,confidenceThreshold = 0.4):
        
        try:
            self.face_detection.setRecognitionEnabled(bool)
        except Exception as e:
            rospy.logerr("{0}".format(e))
        
        if bool:
            try:
                self.face_detection.setRecognitionConfidenceThreshold(confidenceThreshold)
            except Exception as e:
                rospy.logerr("{0}".format(e))
        else:
            pass
        
        self.is_recognizing = bool


    def set_tracking(self,bool):
        
        try:
            self.face_detection.setTrackingEnabled(bool)
        except Exception as e:
            rospy.logerr("{0}".format(e))
        self.is_tracking = bool

    # def get_face_position(self,id = None):
    #     pass

    def get_face_name(self, id = None):
        
        if not self.face.isRecognitionEnabled():
            rospy.logerr("Face Recognition is turn off")
            return None

        if self.face == []:
        #if self.last_face == [] :
            rosy.loginfo("No Face Detected")
            return None
        
        faces = self.face[1]
        faces_number = 1 if id == None else len(face_info)-1

        if id == "all":
            name = []

            for i in range(0,faces_number):
                info = faces[i][1]
                name.append(info[2])
                _id =  = info[0]
        else:
            name = []
            faces = self.face[1]

            for i in range(0,faces_number):
                info = faces[i][1]
                _id = info[0]
                if _id in ids:
                    name.append(info[2])
        return name









