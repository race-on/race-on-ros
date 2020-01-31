#!/usr/bin/python3

## Get image data
## to the "imu_data" topic

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from raceon.msg import TrackPosition

# Dependencies for estimation
import numpy as np
from scipy.signal import find_peaks, butter, filtfilt
from skimage.color import rgb2gray

class PosEstimator():
    
    def __init__(self):
        self.topic_name_camera = rospy.get_param("topic_name_image", "camera/image")
        self.topic_name_pos_err = rospy.get_param("topic_name_position_error", "position/error")
        self.topic_name_pos_track = rospy.get_param("topic_name_position_track", "position/track")
        self.frame_name = rospy.get_param("frame_name", "camera")
        
        # Parameters for estimation
        self.scan_line = rospy.get_param("~scan_line", 170)
        self.peak_thres = rospy.get_param("~peak_threshold", 0.7)
        self.track_width = rospy.get_param("~track_width", 600)
        self.camera_center = rospy.get_param("~camera_center", 320)
        
        self.butter_b, self.butter_a = butter(3, 0.1)
    
    def start(self):
        self.sub_camera = rospy.Subscriber(self.topic_name_camera, Image, self.camera_callback)
        self.pub_pos_err = rospy.Publisher(self.topic_name_pos_err, Pose, queue_size=10)
        self.pub_pos_track = rospy.Publisher(self.topic_name_pos_track, TrackPosition, queue_size=10)
        rospy.spin()

    def camera_callback(self, img_msg):
        width = img_msg.width
        height = img_msg.height
        
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((width, height, 3))
        
        I = rgb2gray(img)
        
        rospy.loginfo("Image with shape " + str(I.shape) + " received.")        
        line_pos = self.camera_center - self.pos_estimate(I)
        
        rospy.loginfo("Estimated line_pos = " + str(line_pos))
        
        pos_msg = Pose()
        pos_msg.position.x = line_pos
        self.pub_pos_err.publish(pos_msg)
        
    def pos_estimate(self, I):

        # Select a horizontal line in the middle of the image
        L = I[self.scan_line, :]

        # Smooth the transitions so we can detect the peaks 
        Lf = filtfilt(self.butter_b, self.butter_a, L)

        # Find peaks which are higher than 0.5
        peaks, p_val = find_peaks(Lf, height=self.peak_thres)
        
        rospy.loginfo(peaks)

        line_pos    = self.camera_center
        line_left   = None
        line_right  = None
        peaks_left  = peaks[peaks < self.camera_center]
        peaks_right = peaks[peaks > self.camera_center]
        
        # Peaks on the left
        if peaks_left.size:
            line_left = peaks_left.max()

        # Peaks on the right
        if peaks_right.size:
            line_right = peaks_right.min()
        
        # Log track position
        track_msg = TrackPosition()
        track_msg.left = 0 if line_left == None else int(line_left)
        track_msg.right = 0 if line_right == None else int(line_right)
        self.pub_pos_track.publish(track_msg)

        # Evaluate the line position
        if line_left and line_right:
            line_pos    = (line_left + line_right ) // 2
            self.track_width = line_right - line_left
            
        elif line_left and not line_right:
            line_pos    = line_left + int(self.track_width / 2)
            
        elif not line_left and line_right:
            line_pos    = line_right - int(self.track_width / 2)
            
        else:
            rospy.loginfo("no line")

        return line_pos

if __name__ == "__main__":
    rospy.init_node("pos_estimation")
    estimator = PosEstimator()
    try:
        estimator.start()
    except rospy.ROSInterruptException:
        pass
