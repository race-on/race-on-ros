#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Pose, Twist

# TODO
SERVO_MIN = 0
SERVO_MIDDLE = 512
SERVO_MAX = 1024

class Controller():
    
    def __init__(self):
        self.topic_name_pos = rospy.get_param("topic_name_position", "position")
        self.topic_name_control = rospy.get_param("topic_name_control", "control")
        
        # Parameters for control
        self.motor_speed = rospy.get_param("~motor_speed", 100)
        self.target = rospy.get_param("~target", 0)
        self.kp = rospy.get_param("~kp", 0.1)
    
    def start(self):
        self.sub_pos = rospy.Subscriber(self.topic_name_pos, Pose, self.pos_callback)
        self.pub_control = rospy.Publisher(self.topic_name_control, Twist, queue_size=10)
        rospy.spin()

    def pos_callback(self, pos_msg):
        pos_err = pos_msg.position.x - self.target
        
        rospy.loginfo("Current error: pos_err = " + str(pos_err))
        
        servo_pos = self.control_servo(pos_err)
        motor_speed = self.motor_speed
        
        rospy.loginfo("Control command: servo_pos = " + str(servo_pos) + ", motor_speed = " + str(motor_speed))
        
        control_msg = Twist()
        control_msg.linear.y = motor_speed
        control_msg.angular.y = servo_pos
        self.pub_control.publish(control_msg)
        
    def pid(self, error):
        return error * self.kp

    def control_servo(self, error):
        correction = self.pid(error)
        servo_pos = SERVO_MIDDLE + correction

        if servo_pos > SERVO_MAX:
            servo_pos = SERVO_MAX
        if servo_pos < SERVO_MIN:
            servo_pos = SERVO_MIN

        return servo_pos

if __name__ == "__main__":
    rospy.init_node("control")
    controller = Controller()
    try:
        controller.start()
    except rospy.ROSInterruptException:
        pass

