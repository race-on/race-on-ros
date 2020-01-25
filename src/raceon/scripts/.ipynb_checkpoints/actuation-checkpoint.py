#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Twist

# TODO
SERVO_MIN = 0
SERVO_MIDDLE = 512
SERVO_MAX = 1024

class Actuator():
    
    def __init__(self):
        self.topic_name_control = rospy.get_param("~topic_name_control", "/control")
    
    def start(self):
        self.sub_control = rospy.Subscriber(self.topic_name_control, Twist, self.control_callback)
        rospy.spin()

    def control_callback(self, control_msg):
        servo_pos = control_msg.angular.y
        motor_speed = control_msg.linear.y
        
        rospy.loginfo("Control command received: servo_pos = " + str(servo_pos) + ", motor_speed = " + str(motor_speed))
        
        self.drive_motor(motor_speed)
        self.drive_servo(servo_pos)
        
    # TODO
    def drive_motor(self, motor_speed):
        rospy.loginfo("Set motor speed to: " + str(motor_speed))

    # TODO
    def drive_servo(self, servo_pos):
        rospy.loginfo("Set servo position to: " + str(servo_pos))

if __name__ == "__main__":
    rospy.init_node("actuation")
    actuator = Actuator()
    try:
        actuator.start()
    except rospy.ROSInterruptException:
        pass

