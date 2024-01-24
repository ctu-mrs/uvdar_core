import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import numpy as np
import math
import itertools
import sklearn.neighbors as neighbors



class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        image_topic = "/uav2/uvdar_bluefox/left/image_raw"
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.image_height=None
        self.image_width=None
        self.latest_image = None
        self.cut_img_height = None
        self.mask_img = None
        self.lock = threading.Lock()
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'mono8')
            with self.lock:
                self.latest_image = cv_image
                self.image_height = len(self.latest_image)
                self.image_width  = len(self.latest_image[0]) 
        except Exception as e:
            rospy.logerr(e)

    def save_latest_image(self, filename):
        with self.lock:
            if self.latest_image is not None:
                cv2.imwrite(filename, self.latest_image)
                rospy.loginfo("Latest image saved as {}".format(filename))
            else:
                rospy.logwarn("No image received yet")

    def pointInRect(self,point,rect):

        x1, y1, w, h = rect
        x2, y2 = x1+w, y1+h
        x, y = point
        if (x1 < x and x < x2):
            if (y1 < y and y < y2):
                return True
        return False

    def connect_regions(self, bright_points):
        
        tree = neighbors.KDTree( bright_points, leaf_size=12 )
        
        dist, ind = tree.query(bright_points, k=500)
        for idx, i in enumerate(ind):
            poly=[]
            for k, val in enumerate(i):
                if dist[idx][k] < 50:
                    v = [bright_points[ind[idx][k]][1],bright_points[ind[idx][k]][0]]
                    poly.append(v)
            contour = np.array(poly)
            cv2.fillPoly(self.mask_img, pts= [contour], color=255)
        
        
        contours, hierarchy = cv2.findContours(self.mask_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.mask_img,contours , -1, 255, 40)

    
    def create_mask(self):
        with self.lock:
            self.cut_img_height=int(3.0/4.0 * self.image_height)
            self.mask_img = np.zeros([self.image_height,self.image_width,1],dtype=np.uint8)
            self.mask_img.fill(0)
            bright_points = [] 
            for x in range(self.cut_img_height,self.image_height):
                for y in range(self.image_width):
                    if self.latest_image[x,y] > 120:
                        # print("(" , x , ", " , y, ") - ", self.latest_image[x,y])
                        val = [x,y]
                        bright_points.append(val)
            line_size = 10


            self.connect_regions(bright_points)            
            self.mask_img = cv2.bitwise_not(self.mask_img)            


            combined = self.latest_image.copy()
            
            for x in range(0, self.image_height):
                for y in range(0, self.image_width): 
                    if self.mask_img[x,y] == 0:
                        combined[x,y] = 0

            cv2.imshow("Combined", combined)
            cv2.imshow("Contours", self.mask_img)

    def display_image_stream(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.latest_image is not None:
                    cv2.imshow('Image Stream', self.latest_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.create_mask()
                #self.save_latest_image('latest_image.png')
            elif key == 27:  # Press 'Esc' key to exit the loop
                cv2.destroyAllWindows()
                break


if __name__ == '__main__':
    image_subscriber = ImageSubscriber()

    display_thread = threading.Thread(target=image_subscriber.display_image_stream)
    display_thread.start()

    rospy.spin()  # Run the ROS event loop

    display_thread.join()  # Wait for the display thread to finish
