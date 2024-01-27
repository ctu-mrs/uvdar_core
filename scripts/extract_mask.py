import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import numpy as np
import math
import itertools
import sklearn.neighbors as neighbors
import os
import subprocess 
from colorama import Fore, Back, Style
import time
import re
import signal
import sys

class ImageSubscriber:
    def __init__(self, _camera_position, _bluefox_id, _mrs_id, _uav_name):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        image_topic = "/" + str(_uav_name) + "/uvdar_bluefox/left/image_raw"
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.image_height=None
        self.image_width=None
        self.latest_image = None
        self.cut_img_height = None
        self.cut_img_width = None
        self.mask_img = None
        self.camera_position = _camera_position
        self.bluefox_id = _bluefox_id
        self.mrs_id = _mrs_id
        self.uav_name = _uav_name
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

    def save_image(self, filename):

        if self.mask_img is not None:
            cv2.imwrite(filename, self.mask_img)
            rospy.loginfo("Image saved with the name: " + filename)
            print("You can move the file to your UAV with the following command:")
            print("scp " + filename + " mrs@" + self.uav_name + ":/opt/ros/noetic/share/mrs_uav_deployment/config/uvdar_calibrations/masks/")
        else:
            rospy.logwarn("No mask image received yet")

    def connect_regions(self, bright_points):
        
        tree = neighbors.KDTree( bright_points)
        if (len(bright_points) < 20):
            print(Fore.RED + "Less than 20 bright points extracted. The scene seems to be too dark!") 
            exit(1)
        dist, ind = tree.query(bright_points, k=20)
        for idx, i in enumerate(ind):
            poly=[]
            for k, val in enumerate(i):
                if dist[idx][k] < 50:
                    v = [bright_points[ind[idx][k]][1],bright_points[ind[idx][k]][0]]
                    poly.append(v)
            contour = np.array(poly)
            cv2.fillPoly(self.mask_img, pts= [contour], color=255)
        
        
        contours, hierarchy = cv2.findContours(self.mask_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.mask_img,contours , -1, 255, BORDER_THICKNESS)

    def verify_point(self, _p):
        if self.camera_position == "left":
           if ( 122 < _p[1] and _p[1] < 452 ) or 632 < _p[1]:
            return False
        elif self.camera_position == "right":
           if _p[1] < 120 or (300 < _p[1] and _p[1] < 630):
            return False
        elif self.camera_position == "back":
            if _p[1] < 630 and _p[0] < 30 and 150 < _p[0]:
                return False
        else: 
            print("Invalid camera position. This should not happen!")
        return True
   
    def create_mask(self):
        with self.lock:
            if self.camera_position == "left" or self.camera_position == "right": 
                self.cut_img_height=int(3.0/4.0 * self.image_height)
                self.cut_img_width = 0 
            else:
                self.cut_img_width = int(3.0/4.0 * self.image_width)
                self.cut_img_height = 0
            
            self.mask_img = np.zeros([self.image_height,self.image_width,1],dtype=np.uint8)
            self.mask_img.fill(0) # make image black for opencv contour recognition algorithm
            
            bright_points = [] 
            for x in range(self.cut_img_height,self.image_height):
                for y in range(self.cut_img_width, self.image_width):
                    if self.latest_image[x,y] > BINARIZATION_THRESHOLD:
                        val = [x,y]
                        add_val = self.verify_point(val)
                        if add_val:
                            bright_points.append(val)

            self.connect_regions(bright_points)            
            self.mask_img = cv2.bitwise_not(self.mask_img)            


            combined = self.latest_image.copy()
            
            for x in range(0, self.image_height):
                for y in range(0, self.image_width): 
                    if self.mask_img[x,y] == 0:
                        combined[x,y] = 0

            image_name=self.mrs_id + "_" + str(self.bluefox_id) + ".png"
            self.save_image(image_name)

            cv2.imshow("Combined", combined)
            cv2.imshow("Mask", self.mask_img)

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


def signal_handler(sig, frame):
    os.system("rm -f tmp_launch.txt")
    sys.exit(0)

if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    BINARIZATION_THRESHOLD = 120
    BORDER_THICKNESS = 50 # [pixel] - change value, if mask output is not as you like. smaller-> finer edges / larger -> bigger mask
   
    print("\n++++++++++++++++++++++++++++++++++++++++++++++\n")
    print("Automated Mask Generation Script")
    print("++++++++++++++++++++++++++++++++++++++++++++++\n")
    
    print("Please call this script for each camera of the UAV individually and only with a UI! Otherwise you cannot verify the quality of the mask generation!")

    print(Fore.BLUE + '\nPlease enter the \'MRS-ID\' of the UAV (e.g X02,X13):'+ Fore.GREEN)
    mrs_id=input() 
    
    print(Fore.BLUE + '\nPlease enter the \'UAV NAME\' of the UAV (e.g UAV1,UAV37):'+ Fore.GREEN)
    uav_name=input() 
    
    print(Fore.BLUE + '\nPlease enter for which camera you want to create the mask:') 
    print(Fore.BLUE + 'Enter: \n1 = LEFT camera \n2 = RIGHT camera \n3 = BACK camera')
    print(Fore.GREEN)
    cam_select = int(input())
    if cam_select == 1:
        print(Fore.GREEN + "LEFT camera selected.")
        cam_position="left"
    elif cam_select == 2:
        print(Fore.GREEN + "RIGHT camera selected.") 
        cam_position="right"
    elif cam_select == 3:
        print(Fore.GREEN + "BACK camera selected.") 
        cam_position="back"
    else: 
        print(Fore.RED + "No valid selection! Stopping script.") 
        exit(1)
    print(Style.RESET_ALL)
    print("Image stream will start in a moment...")

    os.system("rosrun bluefox2 bluefox2_list_cameras > tmp_cam_id.txt")
    time.sleep(0.2) 
    f = open("tmp_cam_id.txt")
    temp = re.findall(r'\d+', f.read())
    res = list(map(int, temp))
    bluefox_id = res[2]
    os.system("rm -f tmp_cam_id.txt")
    start_camera_cmd = "roslaunch uvdar_core camera_only.launch device:=" + str(bluefox_id) + " aec:=false expose_us:=100000 gain_db:=200 uav_name:=" + str(uav_name) + " > tmp_launch.txt 2>&1"
    camera_process = subprocess.Popen([start_camera_cmd], shell=True)

    time.sleep(3)

    image_subscriber = ImageSubscriber(cam_position, bluefox_id, mrs_id, uav_name)

    print("As soon as the Image Stream opens: Please place the UV-Calibration pattern above the UAV to light up the image and press \"s\" to create a mask")

    display_thread = threading.Thread(target=image_subscriber.display_image_stream)
    display_thread.start()

    rospy.spin()  # Run the ROS event loop

    display_thread.join()  # Wait for the display thread to finish