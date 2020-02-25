#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from raceon.msg import AckermannDrive

SERVO_MIN = -900
SERVO_MIDDLE = 0
SERVO_MAX = 900

class Controller():
    
    def __init__(self):
        self.topic_name_pos = rospy.get_param("topic_name_position_pose", "position/pose")
        self.topic_name_control = rospy.get_param("topic_name_control", "control")
        self.topic_name_manual_mode = rospy.get_param("topic_name_manual_mode", "control/manual_mode")
        
        # Parameters for control
        self.motor_speed = rospy.get_param("~motor_speed", 200)
        self.target = rospy.get_param("~target", 0)
        self.kp = rospy.get_param("~kp", 1)

        # Manual mode
        self.manual_mode = False
    
    def start(self):
        self.sub_pos = rospy.Subscriber(self.topic_name_pos, Pose, self.pos_callback)
        self.sub_manual_mode = rospy.Subscriber(self.topic_name_manual_mode, Bool, self.manual_mode_callback)
        self.pub_control = rospy.Publisher(self.topic_name_control, AckermannDrive, queue_size=10)
        rospy.spin()

    def manual_mode_callback(self, mode_msg):
        self.manual_mode = mode_msg.data
        if self.manual_mode:
            control_msg = AckermannDrive()
            control_msg.speed = 0
            control_msg.steering_angle = 0
            self.pub_control.publish(control_msg)

    def pos_callback(self, pos_msg):
        if self.manual_mode:
            rospy.loginfo("Mannual mode is on. Not running control")
        else:
            pos_err = self.target - pos_msg.position.x
            
            rospy.loginfo("Current error: pos = {:.2f}".format(pos_err))
            
            servo_pos = self.control_servo(pos_err)
            motor_speed = self.motor_speed
            
            rospy.loginfo("Control command: servo_pos = " + str(servo_pos) + ", motor_speed = " + str(motor_speed))
            
            control_msg = AckermannDrive()
            control_msg.speed = motor_speed
            control_msg.steering_angle = servo_pos
            self.pub_control.publish(control_msg)
        
    # TODO: Implement PID
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
