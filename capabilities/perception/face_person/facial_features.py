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
    def __init__(self):
        """
        
        """
        super(FacialFeaturesSkill, self).__init__()
        self._description = "facial features control for pepper robot using naoqi services"
        
        self.face_detection = None
        self.memory = None
        #self.face_characteristic = None

        self.got_face = False
        self.face = []
        self.last_face = []
    
        self.is_tracking = False
        self.is_recognizing = False

        self.id =[]
    
    def setup(self):

        self.memory = self.robot.session.service("ALMemory")
        
        self.subscriber = self.memory.subscriber("FaceDetected")
        self.subscriber2 = self.memory.subscriber("PeoplePerception/PeopleList")

        self.subscriber.signal.connect(self.on_human_tracked)
        #self.subscriber2.signal.connect(self.on_human_detected)

        self.face_detection = self.robot.session.service("ALFaceDetection")
        self.face_detection.subscribe("FDskill_FD")

        self.face_characteristic = self.robot.session.service("ALFaceCharacteristics")
        self.face_characteristic.subscribe("FDskill_FC")

        self.people_perception = self.robot.session.service("ALPeoplePerception")
        self.people_perception.subscribe("FDskill_PP")
        return True

    def check(self):
        return True
    
    def start(self):
        return True

    def pause(self):
        return None
    
    def shutdown(self):
        return True

    #pepper
    def on_human_detected(self, value):

        if value == []:
            self.id = []
        else:
            self.id = value

    def on_human_tracked(self, value):
        """
        Callback for event FaceDetected.
        """
        if value == []:  # empty value when the face disappears
            self.got_face = False
            self.face  = []
        else:  # only speak the first time a face appears
            self.got_face = True
            self.face = value
    
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
        # begin = rospy.get_time()
        # counter = 0
        # self.last_face = []

        # while(not rospy.is_shutdown() and (rospy.get_time()-begin) < timeout):
            
        #     if self.face != self.last_face and self.face != []:
        #         counter +=1
        #     self.last_face = self.face

        #     if counter == 2:
        #         return True

        # return False
    

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
     
    def get_facial_features(self,features= ["gender","ages"]):
        self.timeout = 1.0
        self.confidence = 0.4
        if self.is_face_detected():
            
            ids = self._get_ids()
            out_features = []
            print ids
    
            if len(ids) == 0:
                self.logerr("No face detected")

            else:
                for id in ids:
                    begin = rospy.get_time()    
                    a = False
                    
                    while not a:
                        a = self.face_characteristic.analyzeFaceCharacteristics(id)
                        rospy.sleep(0.1)
                        if rospy.get_time()-begin > self.timeout:
                            rospy.logerr("couldn't analyze your face")
                            break
                
                for id in ids:
                    id_features = []
                    
                    if "ages" in features:
                        age_info = self.memory.getData("PeoplePerception/Person/"+str(id)+"/AgeProperties")

                        if age_info == []:
                            age_info = [0,0]

                        ages = [age_info[0],age_info[1]]
                        id_features.append(ages)

                    if "gender" in features:
                        gender_info = self.memory.getData("PeoplePerception/Person/"+str(id)+"/GenderProperties")

                        if gender_info == []:
                            gender_info = ['',-1]
                        gender =[gender_info[0],gender_info[1]]
                        id_features.append(gender)

                    out_features.append(id_features)
            return out_features
        
        else:
            rospy.loginfo("No Face Detected")
            return []


    #pepper
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
        
        self.is_recognizing = bool



    def set_tracking(self,bool):
        
        try:
            self.face_detection.setTrackingEnabled(bool)
        except Exception as e:
            rospy.logerr("{0}".format(e))
        self.is_tracking = bool

    def learn_face(self,name):
        self.face_detection.learnFace(name)
        pass

    def relearn_face(self,name):
        self.face_detection.reLearnFace(name)
        pass

    def remove_face(self,name):
        self.face_detection.forgetPerson(name)
        pass

    def get_known_faces(self):
        return self.face_detection.getLearnedFacesList()


#Bender

    def get_complete_image(self):
        pass
    
    def get_faces(self):
        self.is_face_detected()
        return self.last_face

    def get_detections(self):
        pass
    
    def get_big_faces(self):
        pass

    def get_main_face_fast(self):
        pass


    def save_face(self,name):
        return self.learn_face(name)

    def recognize_main_face(self):
        pass
    
    def print_facial_features(self):
        pass

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

Métodos faltantes (implementados en Bender):

get_complete_image

"get_detection"

save_face

print facial

"""