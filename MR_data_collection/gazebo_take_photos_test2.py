
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class TakePhoto:
    def __init__(self):
        self.image_received = False
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.image_received = True
        rospy.loginfo("Image received!")
        
    def take_photo(self,i):
        while not self.image_received:
            rospy.sleep(0.1)
        self.image_received = False
        # Save the image with a specific name
        cv2.imwrite('image'+str(i)+'.png', self.cv_image)

if __name__ == '__main__':
    rospy.init_node('take_photo', anonymous=True)
    photo_taker = TakePhoto()
    rospy.loginfo("Ready to take photos!")
    n = 10 # number of photos to take
    for i in range(n):
        photo_taker.take_photo(i)
        rospy.loginfo("Taken photo "+str(i+1)+" of "+str(n))
    rospy.spin()
    