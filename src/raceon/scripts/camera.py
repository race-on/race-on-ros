#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image, CompressedImage

import picamera
import signal
import numpy as np

stop_process = False

def signal_handler(signal, frame):
        global stop_process
        stop_process = True
signal.signal(signal.SIGINT, signal_handler)

RES = (640, 480)

# Class to process camera messages
class Stream():
    
    def __init__(self):
        self.topic_name_camera_image = rospy.get_param("topic_name_camera_image", "camera/image")
    
        # Set up ros publisher to publish on img topic, using Image message
        self.pub_img = rospy.Publisher(self.topic_name_camera_image, Image, queue_size=1)

    # Called when new image is available
    def write(self, data):
        
        # Publish raw image
        data_y = data[:RES[0]*RES[1]]
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.width = RES[0]
        msg.height = RES[1]
        msg.encoding = "8UC1"
        msg.step = len(data_y) // RES[1]
        msg.data = data_y
        self.pub_img.publish(msg)
        
if __name__ == "__main__":

    # Set up node using NODENAME
    rospy.init_node("camera")

    # Start capturing camera images
    with picamera.PiCamera() as camera:
        res_width = rospy.get_param("~resolution/width", 640)
        res_height = rospy.get_param("~resolution/height", 480)
        RES = (res_width, res_height)
        fps = rospy.get_param("~fps", 50)
        
        rospy.loginfo("Start to streamming images of size = " + str(RES) + ", and FPS = " + str(fps))
        
        camera.resolution = RES
        camera.framerate = fps
        
        try:
            camera.start_recording(Stream(), format='yuv')
            while not stop_process:
                camera.wait_recording(1)
        except:
            pass
        
        rospy.loginfo("Program closing ...")
        #camera.stop_recording()
        camera.close()
