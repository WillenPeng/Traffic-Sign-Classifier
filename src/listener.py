#!/usr/bin/env python
#Author: Weilun Peng
#06/04/2018

import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from traffic_sign_classifier import sign_predict

class Image_regonize:

    def __init__(self, sg_pred):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.callback)
        self.sg_pred = sg_pred

    def callback(self, data):
        try:
          cv_image_mat = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)
        cv_image = np.asarray(cv_image_mat, dtype=np.uint8)
        sub_img = cv_image[:32, :32]
        Result_pred = self.sg_pred.predict(sub_img)
        print (Result_pred)
        # cv2.imshow("Sub window", sub_img)
        # cv2.imshow("Full window", cv_image)
        # cv2.waitKey(1)
        # cv2.imshow("Image window", sub_img)
def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('image_classify', anonymous=True)

    sg_pred = sign_predict()
    img_conv = Image_regonize(sg_pred)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
