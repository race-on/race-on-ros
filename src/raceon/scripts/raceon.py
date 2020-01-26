import time
from pwm import PWM

class Car:
    '''
    A class to control an ackerman drive car using a motor and a servo
    '''

    def __init__(self, motor_pin=None, servo_pin=None, servo_left=1000, servo_mid=1500, servo_right=2000, motor_reverse=False):
        assert motor_pin is not None, "motor_pin is not defined"
        assert servo_pin is not None, "servo_pin is not defined"

        if servo_left < servo_right:
            assert servo_left < servo_mid < servo_right, "servo_mid is not in between servo_left and servo_right"
        else:
            assert servo_left > servo_mid > servo_right, "servo_mid is not in between servo_left and servo_right"

        # setup motor
        self.motor = PWM(motor_pin)
        self.motor.period = 20000000

        if motor_reverse:
            self.MOTOR_REVERSE = True
            self.MOTOR_BRAKE = 1500
            self.MOTOR_MIN = 1000
            self.MOTOR_MAX = 2000
        
        else:
            self.MOTOR_REVERSE = False
            self.MOTOR_MIN = self.MOTOR_BRAKE = 1000
            self.MOTOR_MAX = 2000

        # setup servo
        self.servo = PWM(servo_pin)
        self.servo.period = 20000000
        
        self.SERVO_MID = servo_mid
        self.SERVO_MIN = servo_left
        self.SERVO_MAX = servo_right

    def _map(self, value, from_min, from_max, to_min, to_max):
        from_range = from_max - from_min
        to_range = to_max - to_min

        scaled_value = float(value - from_min) / float(from_range)

        return int(to_min + (scaled_value * to_range))
    
    def _limit(self, value, min_, max_):
        if value < min_:
            return min_
        elif value > max_:
            return max_
        else:
            return value


    def enable(self):
        self.motor.duty_cycle = self.MOTOR_BRAKE * 1000
        self.servo.duty_cycle = self.SERVO_MID * 1000

        self.motor.enable = True
        self.servo.enable = True
    
    def disable(self):
        self.motor.duty_cycle = self.MOTOR_BRAKE * 1000
        self.servo.duty_cycle = self.SERVO_MID * 1000

        self.motor.enable = False
        self.servo.enable = False
    
    def brake(self):
        self.motor.duty_cycle = self.MOTOR_BRAKE * 1000

    def speed(self, _speed):
        if self.MOTOR_REVERSE:
            _speed = self._limit(_speed, -1000, 1000)
            self.motor.duty_cycle = self._map(_speed, -1000, 1000, self.MOTOR_MIN, self.MOTOR_MAX) * 1000
        
        else:
            _speed = self._limit(_speed, 0, 1000)
            self.motor.duty_cycle = self._map(_speed, 0, 1000, self.MOTOR_MIN, self.MOTOR_MAX) * 1000

    def steer(self, _steer):
        _steer = self._limit(_steer, -1000, 1000)
        if _steer < 0:
            self.servo.duty_cycle = self._map(_steer, -1000, 0, self.SERVO_MIN, self.SERVO_MID) * 1000
        else:
            self.servo.duty_cycle = self._map(_steer, 0, 1000, self.SERVO_MID, self.SERVO_MAX) * 1000