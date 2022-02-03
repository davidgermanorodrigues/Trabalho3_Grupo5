#!/usr/bin/env python3
import copy
import math

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
import tf2_geometry_msgs

from sensor_msgs.msg import Image, LaserScan

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from colorama import Fore, Style


class Driver:

    def __init__(self):

        self.name = None
        self.my_team = None
        self.width = None
        self.centroid = None
        self.goal_active = None
        self.goal = PoseStamped()
        self.angle = None
        self.speed = None

        self.name = rospy.get_name().strip('/')
        team_red = rospy.get_param('/red_players')
        team_green = rospy.get_param('/green_players')
        team_blue = rospy.get_param('/blue_players')

        self.laser_scan = rospy.Subscriber("/" + self.name + "/scan", LaserScan, self.laser)


        if self.name in team_red:
            self.my_team = "red"
            self.prey_team_players = team_green
            self.hunter_team_players = team_blue

        elif self.name in team_green:
            self.my_team = "green"
            self.prey_team_players = team_blue
            self.hunter_team_players = team_red

        elif self.name in team_blue:
            self.my_team = "blue"
            self.prey_team_players = team_red
            self.hunter_team_players = team_green

        print("Im " + self.name + ". I am team " + str(self.my_team))

        self.bridge = CvBridge()
        image_topic = ('/' + self.name +'/camera/rgb/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.Image_Processing)
        self.publisher_command = rospy.Publisher( '/' + self.name + '/cmd_vel', Twist, queue_size=1)
        #self.timer = rospy.Timer(rospy.Duration(0.1), self.Move)

    def laser(self, msg):
        min=1
        #Incremento=0.0175
        #ranges[0] = para trás
        #angle_max=2*pi
        #São 359 medidas (angle_max/incremento~359), logo o index pode ser traduzido em graus

        med_E  = msg.ranges[90]
        med_NE = msg.ranges[45]
        med_N  = msg.ranges[0]
        med_NW = msg.ranges[315]
        med_W  = msg.ranges[270]


        #------WALL LEFT------
        if (med_E < min and med_NE < min) or (med_N < min and med_E < min):
            self.Turn_Right = True
            self.Turn_Left = False
            self.Stop = False

        # ------WALL RIGHT------
        elif med_W < min and med_NW < min or (med_N < min and med_W < min):
            self.Turn_Right = False
            self.Turn_Left = True
            self.Stop = False

        # ------WALL FRONT------
        elif med_N < min:
            self.Turn_Right = False
            self.Turn_Left = False
            self.Stop = True

        # ------NOTHING------
        else:
            self.Turn_Right = False
            self.Turn_Left = False
            self.Stop = False

    def Image_Processing(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            print('Erro com a Camara')


        if self.my_team == 'red':
            self.limit_hunt = [(0,150,0),(20, 255, 20)]
            self.limit_escape = [(150,0,0),(20, 20, 255)]

        elif self.my_team == 'blue':
            self.limit_hunt = ([0,0,100],[50,50,255])
            self.limit_escape = [(0,100,0),(50, 255, 50)]


        elif self.my_team == 'green':
            self.limit_hunt = [(50,0,0),(255, 50, 50)]
            self.limit_escape = [(0,0,50),(50, 50, 255)]


                # Creating Mask with limits defined

        hunt_limit_inf = np.array(self.limit_hunt[0])
        hunt_limit_sup = np.array(self.limit_hunt[1])

        escape_limit_inf = np.array(self.limit_escape[0])
        escape_limit_sup = np.array(self.limit_escape[1])

        Mask_hunt= cv2.inRange(cv_image, hunt_limit_inf, hunt_limit_sup)
        Mask_escape = cv2.inRange(cv_image, escape_limit_inf, escape_limit_sup)

        #------------------MASK HUNT-----------------------

        # Find largest contour in mask
        try:
            cnts, _ = cv2.findContours(Mask_hunt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnt = max(cnts, key=cv2.contourArea)
            Mask_unique = np.zeros(Mask_hunt.shape, np.uint8)
            cv2.drawContours(Mask_unique, [cnt], -1, 255, cv2.FILLED)
            Mask_unique = cv2.bitwise_and(Mask_hunt, Mask_unique)
        # If there is a black mask, there are not objects
        except:
            Mask_unique = Mask_hunt

        # Morphologicall Transformation - Closing
        kernel = np.ones((5, 5), np.uint8)
        # Mask_closed = cv2.morphologyEx(Mask_unique, cv2.MORPH_CLOSE, kernel)
        Mask_closed = cv2.dilate(Mask_unique, kernel, iterations=20)
        Mask_closed = cv2.erode(Mask_closed, kernel, iterations=20)

        # Find centroid
        try:
            M = cv2.moments(Mask_closed)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        except:
            cX = 0
            cY = 0
        centroid_hunt = (cX, cY)

        # Find Bounding Box
        try:
            x, y, w, h = cv2.boundingRect(cnt)
        except:
            x = 0
            y = 0
            w = 0
            h = 0
        Bounding_Box = (x, y, w, h)
        cv2.add(cv_image, (-10, 100, -10, 0), dst=cv_image, mask=Mask_closed)

        if centroid_hunt != (0, 0):

            # Drawing a cross
            x = centroid_hunt[0]
            y = centroid_hunt[1]
            w = 5
            h = 1
            cv2.rectangle(cv_image, (x - w, y - h), (x + w, y + h), (0, 0, 255), 2)
            cv2.rectangle(cv_image, (x - h, y - w), (x + h, y + w), (0, 0, 255), 2)

            # Drawing a Bounding_Box
            x1 = Bounding_Box[0]
            y1 = Bounding_Box[1]
            x2 = x1 + Bounding_Box[2]
            y2 = y1 + Bounding_Box[3]
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (20, 100, 20), 2)
            cv2.putText(cv_image, 'Hunt', (x1, y1-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            self.goal_active = True

        else:
            self.goal_active=False
            cv_image = cv_image

        # ------------------MASK ESCPAPE-----------------------

        # Find largest contour in mask
        try:
            cnts, _ = cv2.findContours(Mask_escape, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnt = max(cnts, key=cv2.contourArea)
            Mask_unique = np.zeros(Mask_escape.shape, np.uint8)
            cv2.drawContours(Mask_unique, [cnt], -1, 255, cv2.FILLED)
            Mask_unique = cv2.bitwise_and(Mask_escape, Mask_unique)
        # If there is a black mask, there are not objects
        except:
            Mask_unique = Mask_escape

        # Morphologicall Transformation - Closing
        kernel = np.ones((5, 5), np.uint8)
        # Mask_closed = cv2.morphologyEx(Mask_unique, cv2.MORPH_CLOSE, kernel)
        Mask_closed = cv2.dilate(Mask_unique, kernel, iterations=20)
        Mask_closed = cv2.erode(Mask_closed, kernel, iterations=20)

        # Find centroid
        try:
            M = cv2.moments(Mask_closed)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        except:
            cX = 0
            cY = 0
        centroid_escape = (cX, cY)

        # Find Bounding Box
        try:
            x, y, w, h = cv2.boundingRect(cnt)
        except:
            x = 0
            y = 0
            w = 0
            h = 0
        Bounding_Box = (x, y, w, h)

        cv2.add(cv_image, (-10, -10, 100, 0), dst=cv_image, mask=Mask_closed)

        if centroid_escape != (0, 0):


            # Drawing a cross
            x = centroid_escape[0]
            y = centroid_escape[1]
            w = 5
            h = 1
            cv2.rectangle(cv_image, (x - w, y - h), (x + w, y + h), (0, 0, 255), 2)
            cv2.rectangle(cv_image, (x - h, y - w), (x + h, y + w), (0, 0, 255), 2)

            # Drawing a Bounding_Box
            x1 = Bounding_Box[0]
            y1 = Bounding_Box[1]
            x2 = x1 + Bounding_Box[2]
            y2 = y1 + Bounding_Box[3]
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (20, 20, 100), 2)
            cv2.putText(cv_image, 'Escape', (x1, y1-20), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 250), 2, cv2.LINE_AA)
            self.goal_escape = True
        else:
            cv_image = cv_image
            self.goal_escape = False


        #cv2.namedWindow('Webcam', cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Image window", cv_image)
        #cv2.imshow('CM', Mask_closed)
        self.width = cv_image.shape[1]
        self.centroid_hunt = centroid_hunt[0]
        self.Move(self.goal_active,self.width,self.centroid_hunt,self.Turn_Right,self.Turn_Left,self.Stop)
        cv2.waitKey(1)

    def Move(self,goal_active,width,centroid_hunt,Turn_Right,Turn_Left,Stop):

        print(Turn_Right)

        if not goal_active:
            self.angle = 0
            self.speed = 0

        elif Turn_Right == True:
            self.speed = 0.5
            self.angle = 0.7

        elif Turn_Left == True:
            self.speed = 0.5
            self.angle = -0.70

        elif Stop == True:
            self.speed = 0
            self.angle = 1

        else:
            self.speed = 1.2
            print('Veiculo detetado : Perseguindo')


            if centroid_hunt < width * 0.5:
                if centroid_hunt < (0.8 * (width / 2)):
                    self.angle = 0.70
                else:
                    self.angle = 0.35
            if centroid_hunt > width * 0.5:
                if centroid_hunt > (1.20 * (width / 2)):
                    self.angle = -0.70
                else:
                    self.angle = -0.35
            if centroid_hunt == width * 0.5:
                self.angle = -0.35


        twist = Twist()
        twist.linear.x = self.speed
        twist.linear.x = 1
        twist.angular.z = self.angle
        #print(twist)
        self.publisher_command.publish(twist)


def main():

    rospy.init_node('p_mpinto_driver', anonymous=False)
    driver = Driver()
    rospy.spin()



if __name__ == '__main__':
    main()