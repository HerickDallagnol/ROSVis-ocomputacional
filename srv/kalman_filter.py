#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class KalmanTracker:
    def __init__(self):
        rospy.init_node('kalman_image_filter', anonymous=True)

        #topicos
        self.image_sub = rospy.Subscriber("/game/cam2", CompressedImage, self.image_callback)
        self.target_pub = rospy.Publisher("/target_coordinates", Point, queue_size=10)
        
        self.bridge = CvBridge()
        
        self.kalman = self.initialize_kalman()
        self.detected_point = None
        self.prev_gray = None
        self.x_center_ant = None
        self.detectado = False
        self.old_detectado = False
        self.max_x = 1440
        self.min_x = 280
        self.max_area = 2000

    def initialize_kalman(self):
        kalman = cv2.KalmanFilter(4, 2)
        dt = 1/30.0 #30 FPS  tem q ver se não é 60fps
        dt_adjusted = dt * 4
        kalman.transitionMatrix = np.array([[1, 0, dt_adjusted, 0],
                                            [0, 1, 0, dt_adjusted],
                                            [0, 0, 1, 0],
                                            [0, 0, 0, 1]], np.float32)
        kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                             [0, 1, 0, 0]], np.float32)
        kalman.processNoiseCov = np.eye(4, 4, dtype=np.float32) * 1e-4
        kalman.measurementNoiseCov = np.eye(2, 2, dtype=np.float32) * 1e-1
        kalman.errorCovPost = np.eye(4, 4, dtype=np.float32)
        return kalman

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            rospy.logerr(f"Erro ao converter a imagem: {e}")
            return
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.prev_gray is None:
            self.prev_gray = gray
            return
        
        # diferença 
        diff = cv2.absdiff(self.prev_gray, gray)
        _, thresholded = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 6:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > self.max_area:
                x, y, w, h = cv2.boundingRect(largest_contour)
                x_center = int((2 * x + w) / 2)
                y_center = int((2 * y + h) / 2)

                self.x_center_ant = x_center
                self.detected_point = np.array([x_center, y_center], np.float32)
                self.max_area = cv2.contourArea(largest_contour) * .97
            else:
                self.detected_point = None

        if self.detected_point is not None:
            self.kalman.correct(self.detected_point)
            predicted = self.kalman.predict()
            real_coords = Point()
            real_coords.x = float(self.detected_point[0])
            real_coords.y = float(self.detected_point[1])
            real_coords.z = 0.0  
            self.target_pub.publish(real_coords)
        else:
            predicted = self.kalman.predict()
            predicted_coords = Point()
            predicted_coords.x = float(predicted[0])
            predicted_coords.y = float(predicted[1])
            predicted_coords.z = 0.0  
            self.target_pub.publish(predicted_coords)

        self.prev_gray = gray

if __name__ == '__main__':
    try:
        KalmanTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
