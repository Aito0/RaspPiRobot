import math
import time
import brickpi3

BP = brickpi3.BrickPi3()

print("BrickPi3 loaded")

BP.reset_all()

LEFT_WHEEL = BP.PORT_D
RIGHT_WHEEL = BP.PORT_A

DPS = 90 # degrees per second

# CARPET
OUTER_WIDTH=230

TEST_CONSTANT = 0.85

# Table
#OUTER_WIDTH=163.5
TIRE_WIDTH=37
WHEEL_WIDTH=OUTER_WIDTH # width between wheels (mm)

WHEEL_RADIUS=23 # mm

#####V = WHEEL_RADIUS * DPS / 360 # mm per second

DISTANCE_THRESHOLD = 3 # mm
DEGREES_THRESHOLD = 7 # degrees

####

kp = 150
kd = 40


BP.set_motor_limits(LEFT_WHEEL, 100, 1000)
BP.set_motor_limits(RIGHT_WHEEL, 100, 1000)

def turnLeft(degrees):
    BP.offset_motor_encoder(LEFT_WHEEL, BP.get_motor_encoder(LEFT_WHEEL))
    BP.offset_motor_encoder(RIGHT_WHEEL, BP.get_motor_encoder(RIGHT_WHEEL))

    BP.set_motor_limits(LEFT_WHEEL, 60, 120)
    BP.set_motor_limits(RIGHT_WHEEL, 60, 120)

    target_mm = WHEEL_WIDTH * 2 * (degrees / 360)
    target_deg = 180 * target_mm / (math.pi * WHEEL_RADIUS)
    
    print("target : ", target_deg)

    left_status = BP.get_motor_status(LEFT_WHEEL)
    right_status = BP.get_motor_status(RIGHT_WHEEL)
    BP.set_motor_position_kp(LEFT_WHEEL, kp)
    BP.set_motor_position_kp(RIGHT_WHEEL, kp)
    BP.set_motor_position_kd(LEFT_WHEEL, kd)
    BP.set_motor_position_kd(RIGHT_WHEEL, kd)

    BP.set_motor_position(LEFT_WHEEL, -target_deg)
    BP.set_motor_position(RIGHT_WHEEL, target_deg)

    while abs(left_status[2] - target_deg) >= DEGREES_THRESHOLD and (abs(right_status[2] - target_deg) >= DEGREES_THRESHOLD):
        left_status = BP.get_motor_status(LEFT_WHEEL)
        right_status = BP.get_motor_status(RIGHT_WHEEL)
        
        #print(left_status[0], left_status[1], left_status[2], left_status[3], right_status[0], right_status[1], right_status[2], right_status[3], sep=",")
        time.sleep(0.02)


def moveForward(dist):
    BP.offset_motor_encoder(LEFT_WHEEL, BP.get_motor_encoder(LEFT_WHEEL))
    BP.offset_motor_encoder(RIGHT_WHEEL, BP.get_motor_encoder(RIGHT_WHEEL))
    
    
    # target = 619.4
    target = 180 * dist / (math.pi * WHEEL_RADIUS) * TEST_CONSTANT
    #target = 350
    print("target : ", target)
    
    left_status = BP.get_motor_status(LEFT_WHEEL)
    right_status = BP.get_motor_status(RIGHT_WHEEL)
    
    BP.set_motor_position_kp(LEFT_WHEEL, kp)
    BP.set_motor_position_kp(RIGHT_WHEEL, kp)
    BP.set_motor_position_kd(LEFT_WHEEL, kd)
    BP.set_motor_position_kd(RIGHT_WHEEL, kd)
    
    BP.set_motor_limits(LEFT_WHEEL, 60, 120)
    BP.set_motor_limits(RIGHT_WHEEL, 60, 120)
    
    BP.set_motor_position(LEFT_WHEEL, target)
    BP.set_motor_position(RIGHT_WHEEL, target)
                                       
    while abs(left_status[2] - target) >= DISTANCE_THRESHOLD and (abs(right_status[2] - target) >= DISTANCE_THRESHOLD):
        
        left_status = BP.get_motor_status(LEFT_WHEEL)
        right_status = BP.get_motor_status(RIGHT_WHEEL)
        time.sleep(0.02)
        
        # TODO: Robot is stuck in here

try:
    for i in range(0, 4):
        moveForward(400)
        turnLeft(90)
    
except KeyboardInterrupt:
    print("Terminated: Ctrl+C pressed")
    BP.reset_all()

