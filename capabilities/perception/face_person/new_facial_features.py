    #!/usr/bin/env python
# -*- coding: utf-8 -*-

import qi
import rospy 
from uchile_skills.robot_skill import RobotSkill
from geometry_msgs.msg import PoseStamped

class FacialFeaturesSkill(RobotSkill):
    """
    """
    _type = "facial_features"

    _type_detector = "face_detector"
    _type_detector_fast = "face_detector_fast"
    _type_recognition = "face_recognition"
    _type_gender = "gender_recognition"
    _type_age = "age_recognition"
    _type_postures = "posture_recognition"

    def __init__(self):
        """
        
        """
        super(FacialFeaturesSkill, self).__init__()
        self._description = "facial features control for pepper robot using naoqi services"
        self.is_tracking = False
        self.is_recognizing = False
        self._features_clients = ["detector", "age", "gender", "recognition", "posture"]
    def setup(self):
        self._memory_srv = self.robot.session.service("ALMemory")
        self._face_det_srv = self.robot.session.service("ALFaceDetection")
        self._face_char_srv = self.robot.session.service("ALFaceCharacteristics")
        self._people_per_srv = self.robot.session.service("ALPeoplePerception")
        return True

    def check(self):
        return True
    
    def start(self,features = ['detector']):

        self._face_det_srv.subscribe("FaceDetectorSkill")
        self._people_per_srv.subscribe("FaceDetectorSkill")
        
        if "detector" in features or "recognition" in features:
            self.subscriber = self.memory.subscriber("FaceDetected")
            self.subscriber.signal.connect(   )
            
        if "recognition" in features:
            ## ACTIVE RECOGNITION
        
        if "gender" in features or "age" in features or "posture" in features:
            self._face_char_srv.subscribe("FaceDetectorSkill")
        
        if "posture" in features:
            self._sit_person_srv.subscribe("FaceDetectorSkill")
        
        return True

    def pause(self):
        return None
    
    def shutdown(self):
        return True
"""
Old Bender Methods
State Machine Compability
"""
    def get_complete_image(self):
        pass

    def get_detections(self):
        pass
    
    def get_faces(self):
        self.is_face_detected()
        return self.last_face
    
    def get_big_faces(self):
        pass

    def get_main_face_fast(self):
        pass

    def get_main_face(self):

    def save_face(self,name):
        return self.learn_face(name)

    def recognize_main_face(self):
        pass
    
    def print_facial_features(self):
        print "Facial features available"
        for feat in self._features_clients:
            if not feat == "detector":
                print feat

     
    def get_facial_features(self,features= ["gender"]):
        timeout = 0.5
        # self.confidence = 0.4

        #check that at least a face is detected
        rate = rospy.Rate(10)
        people_ids = self.get_people_ids()
        # return if not face detected
        if len(people_ids) == 0:
            self.loginfo("Request of facial featues {0}: No Face Detected ".format(features))
            return False, False
        
        self.loginfo("Request of facial featues {0}: Faces {1}".format(features, people_ids))
        
        general_features = {}
        
        for face in people_ids:
            begin = rospy.get_time()    
            face_analyzed = False        

            while not time_is_over:
              
                face_analyzed, person_features = self.get_person_features(features)
                    
                if face_analyzed is None:
                    break

                elif face_analyzed:
                    general_features[face] = person_features

                if rospy.get_time()-begin > timeout:
                    rospy.loginfo("Couldn't Analyze Face '{0}', Maybe person is not looking to the camera".format(face))
                    time_is_over = True
                    break                
                
                #try to update face characteristics               
                try:
                    try_analysis = self._face_char_srv.analyzeFaceCharacteristics(face)

                except Exception as e:
                    rospy.logerr("Exception while Analyzing FaceCharacteristics of face '{0}' : \n {1}".format(face,e))
                    break                

                rate.sleep()
            ###FOR #####
        



        return out_features

    def get_person_features(self,person, features=["gender"]):

        features = {}
        
        if "ages" in features:
            try:
                age_info = self.memory.getData("PeoplePerception/Person/"+str(person)+"/AgeProperties")

                if age_info == []:
                    ages = [0, 0]
                
                else:
                    ages = [age_info[0],age_info[1]] #age - confidence

            
            except Exception as e:
                if person is not in self.__persons_detected:
                    return None, None
                ages = [0, 0]                
                self.logerr("")

            features["age"] = [ages]
        
        if "gender" in features:
            try:
                gender_info = self.memory.getData("PeoplePerception/Person/"+str(person)+"/GenderProperties")
                
                if gender_info == []:
                    gender = ['',0]
                else:
                    gender = [ gender_info[0], gender_info[1]] # gender - confidence                

            except Exception as e:
                if person is not in self.__persons_detected:
                    return None, None
                gender = ['',0]
                self.logerr("")
    
            features["gender"] = gender 

        if "recognize" in features: 
            ####
            #recognition
            ####

        if "posture" in features:


        return features



    def draw_facial_features(self,bboxes,facial_features):
        pass
    
    def request(self, facial_features, input_info, output_request):
        pass
    
    def get_string_age(self, age):#(60, 100)
        str_age = age
        str_age = str_age.replace("(", "")
        str_age = str_age.replace(")", "")
        str_age = str_age.replace(",", " and")

        return "between "+str_age


    """
##############
Exclusive Pepper Methods 
#############

    """

    def on_human_detected(self, value):

        if value == []:
            self.id = []
        else:
            self.id = value


    #pepper
    def is_face_detected(self):
        """
        evaluar si es necesario esta función o entregar las caras directamente del callback     
        """
        if self.face == []:
            self.last_face = []
            return False
        else:
            self.last_face = self.face
            return True
    

    #pepper
    def get_number_of_faces(self, timeout = None):
        
        if self.is_face_detected():
            face_number = len (self.last_face[1]) - 1
        else:
            face_number = 0
        return face_number
    
    def _get_ids(self):
        
        return self.memory.getData("PeoplePerception/PeopleList")

    def get_face_id(self):
        if self.is_face_detected():
            
            faceInfoArray = self.last_face[1]
            ids = []

            for i in range( len(faceInfoArray)-1):
                faceInfo = faceInfoArray[i]
                faceExtraInfo = faceInfo[1]
                id =faceExtraInfo[0]
                ids.append(id)
            return ids
        else: 
            rospy.loginfo("No Face Detected")

    def recognize_face(self):

        if self.is_face_detected():
            faceInfoArray = self.last_face[1]
            
            info = []

            for i in range( len(faceInfoArray)-1):
                faceInfo = faceInfoArray[i]
                faceExtraInfo = faceInfo[1]
              
                id =faceExtraInfo[0]
                score = faceExtraInfo[1]
                name = faceExtraInfo[2]
                info.append([id,score,name])
            return info
        else: 
            rospy.loginfo("No Face Detected")


"""
New Pepper Exclusive Methods

"""


    # def on_human_tracked(self, value):
    #     """
    #     Callback for event FaceDetected.
    #     """
    #     if value == []:  # empty value when the face disappears
    #         self.got_face = False
    #         self.face  = []
    #     else:  # only speak the first time a face appears
    #         self.got_face = True
    #         self.face = value
    
    # #pepper
    # def set_recognition(self,bool,confidenceThreshold = 0.4):
        
    #     try:
    #         self.face_detection.setRecognitionEnabled(bool)
    #     except Exception as e:
    #         rospy.logerr("{0}".format(e))
        
    #     if bool:
    #         try:
    #             self.face_detection.setRecognitionConfidenceThreshold(confidenceThreshold)
    #         except Exception as e:
    #             rospy.logerr("{0}".format(e))
        
    #     self.is_recognizing = bool



    # def set_tracking(self,bool):
        
    #     try:
    #         self.face_detection.setTrackingEnabled(bool)
    #     except Exception as e:
    #         rospy.logerr("{0}".format(e))
    #     self.is_tracking = bool

    # def learn_face(self,name):
    #     self.face_detection.learnFace(name)
    #     pass

    # def relearn_face(self,name):
    #     self.face_detection.reLearnFace(name)
    #     pass

    # def remove_face(self,name):
    #     self.face_detection.forgetPerson(name)
    #     pass

    # def get_known_faces(self):
    #     return self.face_detection.getLearnedFacesList()


"""

Métodos faltantes (implementados en Bender):

get_complete_image

"get_detection"

save_face

print facial

"""