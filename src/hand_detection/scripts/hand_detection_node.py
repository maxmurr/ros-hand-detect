#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

subscriberNodeName='hand_detection_subscriber'
topicName='video_topic'

class HandDetectionNode():
  def __init__(self):
    self.sub = rospy.Subscriber(topicName, Image, callback=self.videoCallback)
    rospy.wait_for_message(topicName, Image)
    self.pub = rospy.Publisher("hand_detection_result", String, queue_size=10)

    self.bridge = CvBridge()
    self.mp_drawing = mp.solutions.drawing_utils
    self.mp_drawing_styles = mp.solutions.drawing_styles
    self.mp_hands = mp.solutions.hands

  def videoCallback(self, data):
    self.convertedFrameBackToCV = self.bridge.imgmsg_to_cv2(data)
    image = self.convertedFrameBackToCV

    with self.mp_hands.Hands(
      model_complexity=0,
      min_detection_confidence=0.6,
      min_tracking_confidence=0.5) as hands:

        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        fingerCount = 0
        L = 0
        R = 0
        H = 0

        if results.multi_hand_landmarks:
          for hand_landmarks in results.multi_hand_landmarks:
            handIndex = results.multi_hand_landmarks.index(hand_landmarks)
            handLabel = results.multi_handedness[handIndex].classification[0].label

            handLandmarks = []

            for landmarks in hand_landmarks.landmark:
              handLandmarks.append([landmarks.x, landmarks.y])

            if handLabel == "Left" and handLandmarks[4][0] > handLandmarks[3][0]:
              fingerCount = fingerCount + 1
            elif handLabel == "Right" and handLandmarks[4][0] < handLandmarks[3][0]:
              fingerCount = fingerCount + 1

            if handLandmarks[8][1] < handLandmarks[6][1]:
              fingerCount = fingerCount + 1
            if handLandmarks[12][1] < handLandmarks[10][1]:
              fingerCount = fingerCount + 1
            if handLandmarks[16][1] < handLandmarks[14][1]:
              fingerCount = fingerCount + 1
            if handLandmarks[20][1] < handLandmarks[18][1]:
              fingerCount = fingerCount + 1

            self.mp_drawing.draw_landmarks(
              image,
              hand_landmarks,
              self.mp_hands.HAND_CONNECTIONS,
              self.mp_drawing_styles.get_default_hand_landmarks_style(),
              self.mp_drawing_styles.get_default_hand_connections_style()
            )
            if handLabel == "Left":
              L = 1
            elif handLabel == "Right":
              R = 2

        #Hand L - R
        H = L + R
        if H == 1:
          cv2.putText(image, "Left", (500,50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,255), 2)
          self.pub.publish("Left")
        elif H == 2:
          cv2.putText(image, "Right", (500,50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0,0,255), 2)
          self.pub.publish("Right")

        if results.multi_hand_landmarks == None:
          H = 0

        cv2.putText(image, str(fingerCount), (20,50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, (0,0,255), 3)
        cv2.imshow("camera", image)
        cv2.waitKey(1)

def main():
  rospy.init_node(subscriberNodeName, anonymous=True)

  # create node
  HandDetectionNode()

  # use node
  rospy.spin()

  # destroy
  cv2.destroyAllWindows()
  rospy.signal_shutdown("Shutting down hand_detection_node")

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
