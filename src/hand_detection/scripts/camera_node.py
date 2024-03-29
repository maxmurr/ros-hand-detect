#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

publisherNodeName='camera_sensor_publisher'
topicName='video_topic'

class CameraNode():
  def __init__(self):
    self.publisher=rospy.Publisher(topicName, Image, queue_size=60)
    self.rate = rospy.Rate(30)
    self.videoCaptureObject = cv2.VideoCapture(0)
    self.bridgeObject = CvBridge()

    while not rospy.is_shutdown():
      returnValue, capturedFrame = self.videoCaptureObject.read()
      if returnValue == True:
        imageToTransmit=self.bridgeObject.cv2_to_imgmsg(capturedFrame, "bgr8")
        self.publisher.publish(imageToTransmit)

      self.rate.sleep()

def main():
  rospy.init_node(publisherNodeName, anonymous=True)

  # create node
  CameraNode()

  #use node
  rospy.spin()

  # destory node
  rospy.signal_shutdown("Shutting down")


if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass