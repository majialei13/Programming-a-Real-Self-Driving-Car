from styx_msgs.msg import TrafficLight

from styx_msgs.msg import TrafficLight
import tensorflow as tf
import os
import numpy as np
import rospy
from functools import partial
import cv2
from random import randint
from keras.models import load_model

os.environ["TF_CPP_MIN_LOG_LEVEL"] = '2'
DIR_PATH = os.path.dirname(os.path.realpath(__file__))


class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        # load the graph
        print(DIR_PATH)

        self.model = load_model(DIR_PATH + '/model.h5')
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()

        # current light default is unknown
        self.light_state = TrafficLight.UNKNOWN

    def get_classification(self, image):

        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # TODO implement light color prediction

        # prediction key
        classification_tl_key = {0: TrafficLight.RED, 1: TrafficLight.YELLOW, 2: TrafficLight.GREEN,
                                 3: TrafficLight.UNKNOWN}
        #new_img = cv2.imwrite("temp.jpg",image)
        #image = cv2.imread("temp.jpg")

        #resized = cv2.resize(image, (128, 256), interpolation=cv2.INTER_AREA)
        resized = cv2.resize(image, (160, 320), interpolation=cv2.INTER_AREA)
        test_img = np.asarray(resized, dtype=np.float32)
        
        with self.graph.as_default():
            pre_result = self.model.predict_classes(test_img[None,:,:,:])

        print(pre_result)    
        if pre_result == 0:
            self.light_state = TrafficLight.RED
        if pre_result == 1:
            self.light_state = TrafficLight.YELLOW
        if pre_result == 2:
            self.light_state = TrafficLight.GREEN
        if pre_result == 3:
            self.light_state = TrafficLight.UNKNOWN

        return self.light_state