#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Image
import sys

import cv2
from cv_bridge import CvBridge

import numpy as np

class image_error(object):

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher('/image_error',Image, queue_size=10)
        self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber('/carla/ego_vehicle/camera/rgb/front/image_color', Image, self.process_image_callback)
        print "subscribed to /carla/ego_vehicle/camera/rgb/front/image_color"
        self.img = None

    def showImage(self,img):
        cv2.imshow('image', img)
        cv2.waitKey(1)


    def blur(self,img):
        img_blur = cv2.blur(img,(5,5))
        return(img_blur)

    def noise(self,img):
        noise = np.random.normal(0,0.5,img.size)
        noise = noise.reshape(img.shape[0],img.shape[1],img.shape[2]).astype("uint8")
        img_noise = img + img*noise
        return(img_noise)

    def process_image_callback(self,msg):
        try:
           
           orig = self.bridge.imgmsg_to_cv2(msg, "bgr8")
           drawImg = self.noise(orig)
        except Exception as err:
            print err
        #showImage(drawImg)
        imgMsg = self.bridge.cv2_to_imgmsg(drawImg, "bgr8")
        #self.image_pub.publish(imgMsg)
        self.img = imgMsg

def main():
    rospy.init_node('error_injector')
    rospy.loginfo('error_injector node started')
    im = image_error()
    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
       # rospy.loginfo("publishing image with error %s" % rospy.get_time())
        if im.img is not None:
            im.image_pub.publish(im.img)
            #rospy.loginfo("publishing image with error")
        rate.sleep()


if __name__ == '__main__':
    main()

