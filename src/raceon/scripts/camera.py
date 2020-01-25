#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image

import picamera

# Names
TOPIC = "/camera"
NODE  = "camera"

RES   = (640, 480)
FPS   = 50


# Class to process camera messages
class Stream(object):

    # Called when new image is available
    def write(self, data):
        
        # Create an image instance and publish
        img = Image(width=RES[0], height=RES[1], data=data[:(RES[0] * RES[1])])
        
        pub.publish(img)
        
        
if __name__ == "__main__":
    
    # Set up ros publisher to publish on img topic, using Image message
    pub = rospy.Publisher(TOPIC, Image, queue_size=1)

    # Set up node using NODENAME
    # Anonymous makes sure the node has a unique name by adding numbers to it
    rospy.init_node(NODE, anonymous=True)

    # Start capturing camera images
    with picamera.PiCamera() as camera:
        camera.resolution = RES
        camera.framerate = FPS
        camera.start_recording(Stream(), format='yuv')
        
        while True:
            camera.wait_recording(1)
        
        camera.stop_recording()