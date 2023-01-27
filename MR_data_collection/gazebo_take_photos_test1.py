import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    # convert the ROS image message to a cv2 image
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    # save the image to disk
    for i in range(10):
        cv2.imwrite('image_'+str(i)+'.jpg', cv_image)

if __name__ == '__main__':
    try:
        # initialize the ROS node
        rospy.init_node('image_saver', anonymous=True)
        # subscribe to the 'camera/rgb/image_raw' topic
        image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)
        # spin to keep the script running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass