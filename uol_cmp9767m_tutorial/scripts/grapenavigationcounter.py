#!/usr/bin/env python
import rospy, image_geometry, tf
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
# from sklearn.cluster import DBSCAN

class IdentifyGrapes():
    camera_model = None
    image_depth_ros = None
    visualisation = True
    color2depth_aspect = (84.1/1920) / (70.0/512)
    object_location = []

    def __init__(self):
        self.bridge = CvBridge()
        
        #subscribers for the camera's topics
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_right_camera/hd/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/thorvald_001/kinect2_right_sensor/sd/image_depth_rect", Image, self.image_depth_callback)
        self.front_camera = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, self.callback)
        self.tf_listener = tf.TransformListener()

        #object location publisher
        self.object_location_pub = rospy.Publisher('/thorvald_001/object_location', PoseStamped, queue_size=10)

    
    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Subscribe only once

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def callback(self, data):
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        try:
            camera_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, desired_encoding="32FC1")
        except CvBridgeError as e:
            print(e)

        try:
            # Resize image
            test_image = cv2.resize(camera_image, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
            cv2.imshow("GCM", test_image)

            # Extract region of interest (roi)
            roi = camera_image[0: 950, 1000: 1920]

            # Convert the extracted image from RGB to HSV
            camera_image_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # Set range for grape colour
            grape_lower_rangecolour = np.array([100, 18, 46])
            grape_higher_rangecolour = np.array([107, 255, 255])

            # detect only grapes in the color image
            grape_colour_mask = cv2.inRange(camera_image_hsv, grape_lower_rangecolour, grape_higher_rangecolour)
            # Threshold HSV image to get only grape colours
            _, grape_colour_mask = cv2.threshold(grape_colour_mask, 254, 255, cv2.THRESH_BINARY)

            kernel = np.ones((5,5),np.uint8)
            dilation = cv2.dilate(grape_colour_mask, kernel, iterations = 1)

            # Retrieve contour for analysis and shape detection
            contours, _ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]


            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 350:
                    #draw a rectangle around each grape bunch
                    x,y,w,h = cv2.boundingRect(contour)
                    cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255),2)

                    #inspired by https://github.com/LCAS/CMP9767M/blob/master/uol_cmp9767m_tutorial/scripts/image_projection_3.py
                    #to determine the location of each grape bunch and transform them to the map coordinate
                    M = cv2.moments(contour)

                    if M["m00"] == 0:
                        print('No object detected.')
                        return

                    # calculate the y,x centroid
                    image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])

                    # "map" from color to depth image
                    depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - camera_image.shape[0]/2)*self.color2depth_aspect, 
                        image_depth.shape[1]/2 + (image_coords[1] - camera_image.shape[1]/2)*self.color2depth_aspect)

                    # get the depth reading at the centroid location
                    depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!

                    if np.isnan(depth_value):
                        continue

                    # calculate object's 3d location in camera coords
                    camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) #project the image coords (x,y) into 3D ray in camera coords 
                    camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
                    camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

                    # object location
                    object_location = PoseStamped()
                    object_location.header.frame_id = "thorvald_001/kinect2_right_rgb_optical_frame"                
                    object_location.pose.position.x = camera_coords[0]
                    object_location.pose.position.y = camera_coords[1]
                    object_location.pose.position.z = camera_coords[2]
                    object_location.pose.orientation.w = 1.0

                    self.object_location_pub.publish(object_location)

                    p_camera = self.tf_listener.transformPose('map', object_location)
                    self.object_location.append([p_camera.pose.position.x, p_camera.pose.position.y, p_camera.pose.position.z] )
                    print('map coords: ', p_camera.pose.position)
                    print()

            #remove duplicate/similar coordinate from the list of grape bunches transformed to the map coordinate
            # inspired by https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
            #eps is the maximum distance between two samples for one to be considered as in the neighborhood of the other
            #min_sample is the number of samples in a neighborhood for a point to be considered as a core point.
            epsilon = 0.05
            minimum_space = 17

            
            # DB = DBSCAN(eps = epsilon, min_samples= minimum_space).fit(self.object_location)
            # No_of_grapebunches = len(np.unique(DB.labels_))
            # print('Number of grape bunches', No_of_grapebunches)

            cv2.imshow("Region Of Interest", roi)
            cv2.waitKey(30)
        except:
            print()

def main():
    rospy.init_node("tracker", anonymous=True)
    IdentifyGrapes()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()