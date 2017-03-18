"""
Sam D Kershaw - KER14468703
"""

import rospy
import numpy
import cv2
#from cv2 import COLOR_BGR2GRAY
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
 
RED = 0
GREEN = 1
YELLOW = 2
BLUE = 3
                                          
class RobotController:
    def __init__(self):
        self.cv_bridge = CvBridge()  
        cv2.namedWindow("Image window", 1) 
        cv2.startWindowThread()
        self.twist = Twist()
        self.found_red = False
        self.found_yellow = False
        self.found_green = False
        self.found_blue = False
        self.moving_towards = None
        self.finished = False
        self.defineColVals()
        self.cmd_vel_pub = rospy.Publisher("/turtlebot/cmd_vel",
                                           Twist, queue_size=10)
        self.depth_sub = rospy.Subscriber("/turtlebot/camera/depth/image_raw",
                                          Image, self.depthCallback)
        self.rgb_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",
                                   Image, self.rgbCallback)
        self.laser_sub = rospy.Subscriber("/turtlebot/scan",
                                          LaserScan, self.laserCallback)
                                         
    def defineColVals(self):
        self.hsv_lower_red = numpy.array([0,100,100])
        self.hsv_upper_red = numpy.array([5,255,140])
        self.hsv_lower_green = numpy.array([50,30,30])
        self.hsv_upper_green = numpy.array([70,255,255])
        self.hsv_lower_yellow = numpy.array([30,100,100])
        self.hsv_upper_yellow = numpy.array([30,255,153])
        self.hsv_lower_blue = numpy.array([110,50,50])
        self.hsv_upper_blue = numpy.array([130,255,250])
        
    def depthCallback(self, data):
        print("depth callback")
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(data,
                                                        desired_encoding="bgr8")
        
    def laserCallback(self, data):
        self.laser_data = data
            
    def rgbCallback(self,data):
        self.rgb_data = data
        if self.finished:
            self.depth_sub.unregister()
            self.rgb_sub.unregister()
            self.cmd_vel_pub.unregister()
            print("Finished.")
            return
            
        self.image = self.cv_bridge.imgmsg_to_cv2(self.rgb_data,desired_encoding="bgr8")
        self.image_to_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
#        hsv_lower_green = numpy.array([40,50,100])
#        hsv_upper_green = numpy.array([60,255,255])
        print("not moving towards")
        # Build HSV masks from range of colour values
        red_hsv_mask = cv2.inRange(self.image_to_hsv,
                                   self.hsv_lower_red, self.hsv_upper_red)
        green_hsv_mask = cv2.inRange(self.image_to_hsv, 
                                     self.hsv_lower_green, self.hsv_upper_green)
        yellow_hsv_mask = cv2.inRange(self.image_to_hsv, 
                                      self.hsv_lower_yellow, self.hsv_upper_yellow)
        blue_hsv_mask = cv2.inRange(self.image_to_hsv,
                                    self.hsv_lower_blue, self.hsv_upper_blue)
        
        self.h, self.w, d = self.image.shape
        
        moments_list = {RED: 0.0, GREEN: 0.0, YELLOW: 0.0, BLUE: 0.0}
        
        if not(self.found_red):
            red_M = cv2.moments(red_hsv_mask)
            if red_M['m00'] > 0:
                moments_list[RED] = red_M['m00']
        else:
            red_M = {'m00', 0.0}
        if not(self.found_green):
            green_M = cv2.moments(green_hsv_mask)
            if green_M['m00'] > 0:
                moments_list[GREEN] = green_M['m00']
        else:
            green_M = {'m00', 0.0}
        if not(self.found_yellow):
            yellow_M = cv2.moments(yellow_hsv_mask)
            if yellow_M['m00'] > 0:
                moments_list[YELLOW] = yellow_M['m00']
        else:
            yellow_M = {'m00', 0.0}
        if not(self.found_blue):
            blue_M = cv2.moments(blue_hsv_mask)
            if blue_M['m00'] > 0:
                moments_list[BLUE] = blue_M['m00']
        else:
            blue_M = {'m00', 0.0}
            
        if (sum(moments_list) > 0.0):
            print("at least one of the colours was found...")
            if ((not(self.found_red) or (self.moving_towards == RED)) and
                (red_M['m00'] > 50000.0)):
                self.moveTowards(red_M, RED)
            elif ((not(self.found_green) or (self.moving_towards == GREEN)) and
                (green_M['m00'] > 50000.0)):
                self.moveTowards(green_M, GREEN)
            elif ((not(self.found_yellow) or (self.moving_towards == YELLOW)) and
                (yellow_M['m00'] > 50000.0)):
                self.moveTowards(yellow_M, YELLOW)
            elif ((not(self.found_blue) or (self.moving_towards == BLUE)) and
                (blue_M['m00'] > 50000.0)):
                self.moveTowards(blue_M, BLUE)
            else:
                print("didnt find anything")
                self.explore()
                
            self.cmd_vel_pub.publish(self.twist)
            cv2.imshow("Image window", self.image)
        else:
            self.explore()
        cv2.waitKey(3)
        
    def explore(self):
        print("exploring...")
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        try:
            ranges = self.depth_image.all
            middle = int(round((len(ranges)-1)/2))
            center = ranges[middle]
            leftRanges = ranges[middle-100:middle]
            rightRanges = ranges[middle:middle+100]
            leftRange = numpy.zeros((self.h,self.w,1), numpy.uint8)
            rightRange = numpy.zeros((self.h,self.w,1), numpy.uint8)
            if numpy.nanmin(leftRanges) > 2.0 and numpy.nanmin(rightRanges) > 2.0:
                print "explore forward"
                self.twist.linear.x = 0.5
                self.twist.angular.z = 0.0
            # Turns the robot left when something is on the right of the robot.
            elif numpy.nanmin(leftRanges) > 2.0 and numpy.nanmin(rightRanges) < 2.0:
                print "explore left"
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.5
            # Turns the robot right when something is on the left.
            elif numpy.nanmin(leftRanges) < 2.0 and numpy.nanmin(rightRanges) > 2.0:
                print "explore right"
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.5
            # If the robot is unsure about ranges will spin until sure.
            else:
                print "explore recover"
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.5
        except NameError:
            print(NameError.message)
        
    def moveTowards(self, M, colour):
        if not(self.moving_towards == colour):
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0
            self.cmd_vel_pub.publish(self.twist)
        print(str(colour) + " is seen: " + str(M['m00']))
        if M['m00'] < 9500000.0:
            print(str(colour) + " < 2500000.0")
            self.moving_towards = colour
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(self.image, (cx,cy), 20, (0,0,255), -1)
            err = cx - self.w/2
            self.twist.linear.x = 1.0
            self.twist.angular.z = -float(err) / 100
        else:
            self.setFound(colour)
            self.moving_towards = None
            self.explore()
            
    def setFound(self, colour):
        if colour == 0:
            self.found_red = True
        elif colour == 1:
            self.found_green = True
        elif colour == 2:
            self.found_yellow = True
        elif colour == 3:
            self.found_blue = True
        
rospy.init_node("rgb_converter")
RobotController()
rospy.spin()
cv2.destroyAllWindows()