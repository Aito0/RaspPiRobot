import math
import brickpi3

BP = brickpi3.BrickPi3()

print("BrickPi3 loaded")

BP.reset_all()

"""
Variables and constants
"""
LEFT_WHEEL = BP.PORT_D
RIGHT_WHEEL = BP.PORT_A

WHEEL_WIDTH = 999  # width between wheels TODO
WHEEL_RADIUS = 35  # mm

DPS = 45  # degrees per second

DISTANCE_THRESHOLD = 0  # mm

BP.set_motor_limits(LEFT_WHEEL, 70, 1000)
BP.set_motor_limits(RIGHT_WHEEL, 70, 1000)


def turnLeft90():
    pass


def moveForward(dist):
    BP.offset_motor_encoder(LEFT_WHEEL, BP.get_motor_encoder(LEFT_WHEEL))
    BP.offset_motor_encoder(RIGHT_WHEEL, BP.get_motor_encoder(RIGHT_WHEEL))

    # target = 619.4
    target = 180 * dist / (math.pi * WHEEL_RADIUS)

    left_status = BP.get_motor_status(LEFT_WHEEL)
    right_status = BP.get_motor_status(RIGHT_WHEEL)

    BP.set_motor_limits(LEFT_WHEEL, 60, 90)
    BP.set_motor_limits(RIGHT_WHEEL, 60, 90)

    while abs(left_status[2] - target) >= DISTANCE_THRESHOLD and (abs(right_status[2] - target) >= DISTANCE_THRESHOLD):
        BP.set_motor_position(LEFT_WHEEL, target)
        BP.set_motor_position(RIGHT_WHEEL, target)

        left_status = BP.get_motor_status(LEFT_WHEEL)
        right_status = BP.get_motor_status(RIGHT_WHEEL)

        # TODO: Robot is stuck in here


try:
    moveForward(400)
except KeyboardInterrupt:
    print("Terminated: Ctrl+C pressed")
    BP.reset_all()

