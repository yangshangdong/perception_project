# import os
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2

# class Camera:
#     def __init__(self, path):
#         self.photos_taken = 0
#         self.path = path
#         if not os.path.exists(self.path):
#             os.makedirs(self.path)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

#     def image_callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             print(e)

#         filename = 'image' + str(self.photos_taken) + '.jpg'
#         file_path = os.path.join(self.path, filename)
#         cv2.imwrite(file_path, cv_image)
#         self.photos_taken += 1
#         print("Taken photo: ", self.photos_taken)

# if __name__ == '__main__':
#     rospy.init_node('image_listener', anonymous=True)
#     camera = Camera('/home/xtark/Desktop/perception_project/MR_data_collection/photos')
#     rate = rospy.Rate(1) # take a photo every second
#     num_photos = 11
#     while camera.photos_taken < num_photos:
#         rospy.sleep(0.1)
#         rospy.spinOnce()
#     rospy.signal_shutdown("Finished taking photos.")




# import os
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import time

# class Camera:
#     def __init__(self, path):
#         self.photos_taken = 0
#         self.path = path
#         if not os.path.exists(self.path):
#             os.makedirs(self.path)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
#         self.last_photo_time = 0

#     def image_callback(self, data):
#         current_time = time.time()
#         if current_time - self.last_photo_time > 1: # check if at least 1 second has passed since the last photo was taken
#             self.last_photo_time = current_time
#             try:
#                 cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#             except CvBridgeError as e:
#                 print(e)

#             filename = 'image' + str(self.photos_taken) + '.jpg'
#             file_path = os.path.join(self.path, filename)
#             cv2.imwrite(file_path, cv_image)
#             self.photos_taken += 1
#             print("Taken photo: ", self.photos_taken)

# if __name__ == '__main__':
#     rospy.init_node('image_listener', anonymous=True)
#     camera = Camera('/home/xtark/Desktop/perception_project/MR_data_collection/photos')
#     num_photos = 11
#     while camera.photos_taken < num_photos:
#         rospy.spinOnce()
#     rospy.signal_shutdown("Finished taking photos.")

# import os
# import rospy
# from sensor_msgs.msg import Image
# from std_msgs.msg import Empty
# from cv_bridge import CvBridge, CvBridgeError
# import cv2

# class Camera:
#     def __init__(self, path):
#         self.photos_taken = 0
#         self.path = path
#         if not os.path.exists(self.path):
#             os.makedirs(self.path)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
#         self.trigger_sub = rospy.Subscriber('/camera/take_photo', Empty, self.take_photo_callback)

#     def image_callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             print(e)

#         filename = 'image' + str(self.photos_taken) + '.jpg'
#         file_path = os.path.join(self.path, filename)
#         cv2.imwrite(file_path, cv_image)
#         self.photos_taken += 1
#         print("Taken photo: ", self.photos_taken)

#     def take_photo_callback(self, msg):
#         self.image_sub.unregister()
#         self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

# if __name__ == '__main__':
#     rospy.init_node('image_listener', anonymous=True)
#     camera = Camera('/home/xtark/Desktop/perception_project/MR_data_collection/photos')
#     trigger_pub = rospy.Publisher('/camera/take_photo', Empty, queue_size=1)
#     rate = rospy.Rate(1) # take a photo every second
#     num_photos = 11
#     for i in range(num_photos):
#         trigger_pub.publish(Empty())
#         rate.sleep()
#         rospy.loginfo("dddd")
#     rospy.signal_shutdown("Finished taking photos.")

import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class Camera:
    def __init__(self, path):
        self.photos_taken = 0
        self.path = path
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        filename = 'image' + str(self.photos_taken) + '.jpg'
        file_path = os.path.join(self.path, filename)
        cv2.imwrite(file_path, cv_image)
        self.photos_taken += 1
        print("Taken photo: ", self.photos_taken)

    def take_photo(self):
        self.image_sub.unregister()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

if __name__ == '__main__':
    rospy.init_node('image_listener', anonymous=True)
    camera = Camera('/home/xtark/Desktop/perception_project/MR_data_collection/photos')
    num_photos = 4
    # while camera.photos_taken < num_photos:
    #     camera.take_photo()
    #     rospy.sleep(2) # sleep for 1 second between each photo

    while camera.photos_taken < num_photos:
        if camera.photos_taken < num_photos:
            camera.take_photo()
            rospy.sleep(2)
    rospy.signal_shutdown("Finished taking photos.")

# import os
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2

# class Camera:
#     def __init__(self, num_photos, path):
#         self.num_photos = num_photos
#         self.photos_taken = 0
#         self.path = path
#         if not os.path.exists(self.path):
#             os.makedirs(self.path)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

#     def image_callback(self, data):
#         if self.photos_taken < self.num_photos:
#             try:
#                 cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#             except CvBridgeError as e:
#                 print(e)

#             filename = 'image' + str(self.photos_taken) + '.jpg'
#             file_path = os.path.join(self.path, filename)
#             cv2.imwrite(file_path, cv_image)
#             self.photos_taken += 1
#             print("Taken photo: ", self.photos_taken)
#         else:
#             self.image_sub.unregister()
#             rospy.signal_shutdown("Finished taking photos.")

#     def start(self):
#         rospy.spin()
        

# if __name__ == '__main__':
#     rospy.init_node('image_listener', anonymous=True)
#     camera = Camera(11, '/home/xtark/Desktop/perception_project/MR_data_collection/photos')
#     camera.start()
