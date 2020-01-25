#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Post2D

# TODO
SERVO_MIN = 0
SERVO_MAX = 0

#TODO
Kp = 0.1

def pid(error):
    return error * Kp

def control(servo_middle, motor_speed, error):
    correction = pid(error)
    servo_middle += correction
             
    if servo_middle > SERVO_MAX:
        servo_middle = SERVO_MAX
    if servo_middle < SERVO_MIN:
        servo_middle = SERVO_MIN
             
    return servo_middle, motor_speed

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        
        #TODO Provide the values to the funtion
        control(servo_middle, motor_speed, error)
        rate.sleep()
        
    if __name__ == '__main__':
        try:
            talker()
        except rospy.ROSInterruptException:
            pass
