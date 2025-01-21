import brickpi3
BP = brickpi3.BrickPi3()

print("BrickPi3 loaded")

BP.reset_all()

LEFT_WHEEL = BP.PORT_D
RIGHT_WHEEL = BP.PORT_A


BP.set_motor_limits(LEFT_WHEEL, 70, 1000)
BP.set_motor_limits(RIGHT_WHEEL, 70, 1000)

        
while True:
    try:
        BP.set_motor_dps(LEFT_WHEEL, 360)
        BP.set_motor_dps(RIGHT_WHEEL, 360)
    except KeyboardInterrupt:
        BP.reset_all()
        print("Terminated: Ctrl+C pressed")
        exit()

