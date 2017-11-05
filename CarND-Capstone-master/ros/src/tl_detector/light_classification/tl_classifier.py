#uncomment the following in real test
from styx_msgs.msg import TrafficLight

import numpy as np
import cv2
import tensorflow as tf
from PIL import Image
import os
from collections import defaultdict
from io import StringIO

cwd = os.path.dirname(os.path.realpath(__file__))


class TLClassifier(object):
    def __init__(self, threshold):
        self.threshold = threshold
        self.signal_status = None
        self.trafficLightLocationBox = None

        os.chdir(cwd)
        self.setupGraphForTrafficLightLocalization()


    def classifyTrafficLightState(self,image):

        #After detection of the traffic lights on the camera frame, their images are separated and resized for classification of traffic color light. This is done by counting the color of the pixels on the image. If the majority of the pixels are in Green color, for example, the Traffic Light is classified as showing the Green Light.
 
        #https://docs.opencv.org/3.0.0/df/d9d/tutorial_py_colorspaces.html
        #https://stackoverflow.com/questions/10948589/choosing-correct-hsv-values-for-opencv-thresholding-with-inranges
        #openCV uses H 0-180, S 0-255, V 0-255. Others uses H 0-360, S 0-100, V 0-100
        #Changing Colorspaces
        #you take [H-10, 100,100] and [H+10, 255, 255] as lower bound and upper bound respectively
        tfRedMinHSV = np.array([0,100,100])
        tfRedMaxHSV = np.array([20,255,255])

        tfYellowMinHSV = np.array([20,100,100])
        tfYellowMaxHSV = np.array([40,255,255])

        tfGreenMinHSV = np.array([50,100,100])
        tfGreenMaxHSV = np.array([70,255,255])

        #cv2.imshow('Original Read',image)

        imageBGR = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        #cv2.imshow('Read to BGR',imageBGR)

        hsvImage = cv2.cvtColor(imageBGR, cv2.COLOR_BGR2HSV)
        #cv2.imshow('HSVRepresentation',hsvImage)

        mask_red = cv2.inRange(hsvImage,tfRedMinHSV,tfRedMaxHSV)
        numberRedPixels = cv2.countNonZero(mask_red)
        #cv2.imshow('RedMask',mask_red)
        #print "Red pixels detected: " + str(numberRedPixels)

        mask_yellow = cv2.inRange(hsvImage,tfYellowMinHSV,tfYellowMaxHSV)
        numberYellowPixels = cv2.countNonZero(mask_yellow)
        #cv2.imshow('YellowMask',mask_yellow)	
        #print "Yellow pixels detected: " + str(numberYellowPixels)

        mask_green = cv2.inRange(hsvImage,tfGreenMinHSV,tfGreenMaxHSV)
        numberGreenPixels = cv2.countNonZero(mask_green)
        #cv2.imshow('GreenMask',mask_green)	
        #print "Green pixels detected: " + str(numberGreenPixels)       

        listPixels = [numberRedPixels,numberYellowPixels,numberGreenPixels]
        largestValueInList = max(listPixels)

        if (largestValueInList == numberRedPixels):
            self.signal_status = TrafficLight.RED
        elif (largestValueInList == numberYellowPixels):
            self.signal_status = TrafficLight.YELLOW
        elif (largestValueInList == numberGreenPixels):
            self.signal_status = TrafficLight.GREEN

        return self.signal_status


    def setupGraphForTrafficLightLocalization(self):
        #SSD w/ MobileNet Trained with COCO, presented by the TensorFlow API, being used for localization of Traffic Lights
        #https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = 'ssdMobileNetTfApiExGraph.pb'

        #Load a (frozen) Tensorflow model into memory.
        self.detection_graph = tf.Graph()

        #GPU if present
        configGPU = tf.ConfigProto()
        configGPU.gpu_options.allow_growth = True

        #Detection Setup
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
               serialized_graph = fid.read()
               od_graph_def.ParseFromString(serialized_graph)
               tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph, config=configGPU)

            # Definite input and output Tensors for detection_graph
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores =self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections =self.detection_graph.get_tensor_by_name('num_detections:0')


    def locateTrafficLightsOnFrame(self, image, visual=False):
        #Uses frames from the camera to locate traffic lights, returns boxes with location of the traffic lights detected.
        with self.detection_graph.as_default():
            image_expanded = np.expand_dims(image, axis=0)
            (detection_boxes, detection_scores, detection_classes, num_detections) = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],feed_dict={self.image_tensor: image_expanded})

            detection_boxes=np.squeeze(detection_boxes)
            detection_classes =np.squeeze(detection_classes)
            detection_scores = np.squeeze(detection_scores)

            cls = detection_classes.tolist()

            #ID 10 equals Traffic Light class on the SSD w/ MobileNet Trained with COCO
            indexClass = next((i for i, v in enumerate(cls) if v == 10.), None)

            #if there is no index class provided, no detection was made.
            if (indexClass is None):
                trafficLightLocalized=[0, 0, 0, 0]
                print('No detection was made [indexClass is None]')

            #if the detection_score for the provided indexClass (10 for traffic light) is less than the provided threshold, consider that no detection was made
            elif detection_scores[indexClass]<=self.threshold:
                trafficLightLocalized=[0, 0, 0, 0]
                print('No detection was made [low detection_score for indexClass 10: ' + detection_scores[indexClass] + ']')

            #if a detection was made
            else:

                #get dimensions of image
                imageDimensions = image.shape[0:2]
                imageHeight, imageWidth = imageDimensions[0], imageDimensions[1]

                #Use dimensions of the image to calculate detected traffic light box dimensions in pixels
                trafficLightLocalized = np.array([int(detection_boxes[indexClass][0]*imageHeight), int(detection_boxes[indexClass][1]*imageWidth), int(detection_boxes[indexClass][2]*imageHeight), int(detection_boxes[indexClass][3]*imageWidth)])

                trafficLightLocalizedHeight = trafficLightLocalized[2] - trafficLightLocalized[0]
                trafficLightLocalizedWidth = trafficLightLocalized[3] - trafficLightLocalized[1]

                # Unconsider if the boxes are too small. Meaning that the traffic light detected is too far away to be considered by the planner at the moment.
                if ((trafficLightLocalizedHeight < 15) or (trafficLightLocalizedWidth< 15)):
                    trafficLightLocalized =[0, 0, 0, 0]
                    print('No detection was made [detected traffic light box width or height below 15 pixels]')

                # Unconsider if the detection box has a weird aspect ratio. For example, a traffic light with a 1:1 aspect ratio, meaning that it is a square, that TF detection is probably wrong...
                elif ((trafficLightLocalizedHeight/(trafficLightLocalizedWidth + 0.01)) < 1.4):
                    trafficLightLocalized =[0, 0, 0, 0]
                    print('No detection was made [detected traffic light box height/width ratio is uncommon, smaller than 1.4]')

		# If the two hypothesis above are false, this probably is a good detection.
                else:
                    print(trafficLightLocalized)
                    print('Traffic Light localized with a detection_score of: ', detection_scores[indexClass])

            self.trafficLightLocationBox = trafficLightLocalized

        return trafficLightLocalized

if __name__ == '__main__':
    tl_cls =TLClassifier()
    os.chdir(cwd)
