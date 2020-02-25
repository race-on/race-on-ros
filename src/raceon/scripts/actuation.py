#!/usr/bin/env python
# license removed for brevity

import rospy
from raceon.msg import AckermannDrive
from car import Car

MOTOR_PIN = 0
SERVO_PIN = 1
SERVO_LEFT = 2000
SERVO_RIGHT = 1000
SERVO_MID = 1500

class Actuator():
    
    def __init__(self, car):
        self.topic_name_control = rospy.get_param("topic_name_control", "control")
        self.car = car
        
    def start(self):
        self.sub_control = rospy.Subscriber(self.topic_name_control, AckermannDrive, self.control_callback)
        rospy.spin()
    
    def stop(self):
        rospy.loginfo("Stop the car ...")
        self.car.brake()
        self.car.steer(0)
        #self.car.disable()

    def control_callback(self, control_msg):
        servo_pos = control_msg.steering_angle
        motor_speed = control_msg.speed
        
        rospy.loginfo("Control command received: servo_pos = " + str(servo_pos) + ", motor_speed = " + str(motor_speed))
        
        self.drive_motor(motor_speed)
        self.drive_servo(servo_pos)
        
    def drive_motor(self, motor_speed):
        rospy.loginfo("Set motor speed to: " + str(motor_speed))
        self.car.speed(motor_speed)

    def drive_servo(self, servo_pos):
        rospy.loginfo("Set servo position to: " + str(servo_pos))
        self.car.steer(servo_pos)

if __name__ == "__main__":
    car = Car(MOTOR_PIN, SERVO_PIN, SERVO_LEFT, SERVO_MID, SERVO_RIGHT)
    car.enable()
    
    rospy.init_node("actuation")
    actuator = Actuator(car)
    try:
        actuator.start()
    except rospy.ROSInterruptException:
        pass
    actuator.stop()
