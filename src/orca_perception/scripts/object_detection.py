#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from std_msgs.msg import String

class ObjectDetector:
    """
    Basic object detection using OpenCV
    """
    
    def __init__(self):
        rospy.init_node('object_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Get parameters
        self.camera_topic = rospy.get_param('~camera_topic', '/front_camera/image_raw')
        self.detection_method = rospy.get_param('~detection_method', 'contour')  # 'contour' or 'color'
        self.min_area = rospy.get_param('~min_area', 1000)
        self.target_color_lower = rospy.get_param('~target_color_lower', [0, 100, 100])  # HSV lower bound
        self.target_color_upper = rospy.get_param('~target_color_upper', [10, 255, 255])  # HSV upper bound
        
        # Convert color bounds to numpy arrays
        self.target_color_lower = np.array(self.target_color_lower, dtype=np.uint8)
        self.target_color_upper = np.array(self.target_color_upper, dtype=np.uint8)
        
        # Initialize subscribers and publishers
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        self.image_pub = rospy.Publisher('detection_image', Image, queue_size=1)
        self.object_pub = rospy.Publisher('detected_object', Point, queue_size=1)
        self.object_type_pub = rospy.Publisher('object_type', String, queue_size=1)
        
        rospy.loginfo("Object detector initialized with method: %s", self.detection_method)
    
    def image_callback(self, msg):
        """
        Process incoming image messages
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return
        
        # Detect objects based on the selected method
        if self.detection_method == 'contour':
            result_image, objects = self.detect_by_contour(cv_image)
        elif self.detection_method == 'color':
            result_image, objects = self.detect_by_color(cv_image)
        else:
            rospy.logerr("Unknown detection method: %s", self.detection_method)
            return
        
        # Publish the result image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
        
        # Publish detected objects
        if objects:
            for obj in objects:
                # Publish object position (image coordinates)
                point_msg = Point()
                point_msg.x = obj['center'][0]
                point_msg.y = obj['center'][1]
                point_msg.z = 0.0  # No depth information
                self.object_pub.publish(point_msg)
                
                # Publish object type
                self.object_type_pub.publish(obj['type'])
                
                rospy.loginfo("Detected %s at (%d, %d) with area %d", 
                             obj['type'], obj['center'][0], obj['center'][1], obj['area'])
    
    def detect_by_contour(self, image):
        """
        Detect objects using contour analysis
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create a copy of the original image for drawing
        result_image = image.copy()
        
        # List to store detected objects
        objects = []
        
        # Process each contour
        for contour in contours:
            # Calculate contour area
            area = cv2.contourArea(contour)
            
            # Filter small contours
            if area < self.min_area:
                continue
            
            # Calculate contour center
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            
            # Approximate the contour
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Determine object type based on the number of vertices
            if len(approx) == 3:
                object_type = "triangle"
                color = (0, 255, 0)  # Green
            elif len(approx) == 4:
                object_type = "rectangle"
                color = (0, 0, 255)  # Red
            elif len(approx) >= 8:
                object_type = "circle"
                color = (255, 0, 0)  # Blue
            else:
                object_type = "unknown"
                color = (255, 255, 0)  # Yellow
            
            # Draw contour and center
            cv2.drawContours(result_image, [approx], 0, color, 2)
            cv2.circle(result_image, (cx, cy), 5, color, -1)
            cv2.putText(result_image, object_type, (cx - 20, cy - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Add object to the list
            objects.append({
                'type': object_type,
                'center': (cx, cy),
                'area': area
            })
        
        return result_image, objects
    
    def detect_by_color(self, image):
        """
        Detect objects using color thresholding
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create a mask for the target color
        mask = cv2.inRange(hsv, self.target_color_lower, self.target_color_upper)
        
        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create a copy of the original image for drawing
        result_image = image.copy()
        
        # List to store detected objects
        objects = []
        
        # Process each contour
        for contour in contours:
            # Calculate contour area
            area = cv2.contourArea(contour)
            
            # Filter small contours
            if area < self.min_area:
                continue
            
            # Calculate contour center
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            
            # Draw contour and center
            cv2.drawContours(result_image, [contour], 0, (0, 255, 0), 2)
            cv2.circle(result_image, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(result_image, "Object", (cx - 20, cy - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add object to the list
            objects.append({
                'type': "colored_object",
                'center': (cx, cy),
                'area': area
            })
        
        return result_image, objects

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
