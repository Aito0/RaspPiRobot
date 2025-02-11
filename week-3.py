import brickpi3

import random
import math
import time

#### BrickPi3 init

BP = brickpi3.BrickPi3()
BP.reset_all()

print("BrickPi3 loaded")


#### Variables and constants
class Config:
    LEFT_WHEEL = BP.PORT_D
    RIGHT_WHEEL = BP.PORT_C

    # Degrees per second
    MOVE_DPS = 150
    TURN_DPS = 150

    # Measurements
    WHEEL_RADIUS = 26.5  ## mm
    WHEEL_WIDTH = 212

    # Calibration constants
    DIST_CONSTANT = 1.01
    TURN_CONSTANT = 1.25

    # Thresholds (?)
    DISTANCE_THRESHOLD = 10
    TURN_THRESHOLD = 5

    # PID consts
    LW_KP = 50
    RW_KP = 50
    LW_KD = 10
    RW_KD = 10


#### Particle set

class ParticleSet:
    NUMBER_OF_PARTICLES = 100

    def __init__(self, e_sampler, f_sampler, g_sampler):
        self.e_sampler = e_sampler
        self.f_sampler = f_sampler
        self.g_sampler = g_sampler
        self.particles = [(0.0, 0.0, 0.0)] * self.NUMBER_OF_PARTICLES
        self.weights = [1 / self.NUMBER_OF_PARTICLES] * self.NUMBER_OF_PARTICLES

    def __iter__(self):
        return iter(self.particles)

    def estimate_position(self):
        x_bar = 0
        y_bar = 0
        theta_bar = 0
        for (x, y, theta), w in zip(self.particles, self.weights):
            x_bar += x * w
            y_bar += y * w
            theta_bar += theta * w

        return x_bar, y_bar, theta_bar

    def after_moving_forward(self, D):
        for idx, (x, y, theta) in enumerate(self.particles):
            e = self.e_sampler.sample()
            f = self.f_sampler.sample()

            x_new = x + (D + e) * math.cos(math.pi * theta / 180)
            y_new = y + (D + e) * math.sin(math.pi * theta / 180)
            theta_new = theta + f

            self.particles[idx] = (x_new, y_new, theta_new)

    def after_turning(self, alpha):
        for idx, (x, y, theta) in enumerate(self.particles):
            g = self.g_sampler.sample()

            theta_new = theta + alpha + g

            self.particles[idx] = (x, y, theta_new)


class DisplaySquare:
    def __init__(self, ps, D):
        self.particles = ps
        self.x_ofs = 0
        self.y_ofs = 0
        self.x_scale = 1
        self.y_scale = 1
        self.D = D

    def draw(self):
        # x0, y0, x1, y1
        lines = [
            (0, 0, 0, self.D),
            (0, 0, self.D, 0),
            (self.D, 0, self.D, self.D),
            (0, self.D, self.D, self.D)
        ]

        lines_transformed = [(x0 * self.x_scale + self.x_ofs,
                              y0 * self.y_scale + self.x_ofs,
                              x1 * self.x_scale + self.x_ofs,
                              y1 * self.y_scale + self.y_ofs)
                             for x0, y0, x1, y1 in lines]

        for line in lines_transformed:
            print("drawLine:" + str(line))

        particles_transformed = [(x * self.x_scale + self.x_ofs,
                                  y * self.y_scale + self.y_ofs,
                                  theta)
                                 for x, y, theta in self.particles]

        print("drawParticles:" + str(particles_transformed))


class Sampler:
    def __init__(self, sigma):
        self.sigma = sigma

    def sample(self):
        return random.gauss(mu=0, sigma=self.sigma)


####
class Robot:
    def __init__(self, stddev_e, stddev_f, stddev_g):
        self.particles = ParticleSet(Sampler(stddev_e), Sampler(stddev_f), Sampler(stddev_g))
        self.graphics = DisplaySquare(self.particles, 400)

    def move_forward(self, mm):
        BP.offset_motor_encoder(Config.LEFT_WHEEL, BP.get_motor_encoder(Config.LEFT_WHEEL))
        BP.offset_motor_encoder(Config.RIGHT_WHEEL, BP.get_motor_encoder(Config.RIGHT_WHEEL))

        # target = 619.4
        target = 180 * mm / (math.pi * Config.WHEEL_RADIUS) * Config.DIST_CONSTANT
        # target = 350
        print("target : ", target)

        left_status = BP.get_motor_status(Config.LEFT_WHEEL)
        right_status = BP.get_motor_status(Config.RIGHT_WHEEL)

        BP.set_motor_position_kp(Config.LEFT_WHEEL, Config.LW_KP)
        BP.set_motor_position_kp(Config.RIGHT_WHEEL, Config.RW_KP)
        BP.set_motor_position_kd(Config.LEFT_WHEEL, Config.LW_KD)
        BP.set_motor_position_kd(Config.RIGHT_WHEEL, Config.RW_KD)

        BP.set_motor_limits(Config.LEFT_WHEEL, 60, 120)
        BP.set_motor_limits(Config.RIGHT_WHEEL, 60, 120)

        BP.set_motor_position(Config.LEFT_WHEEL, target)
        BP.set_motor_position(Config.RIGHT_WHEEL, target)

        while abs(left_status[2] - target) >= Config.DISTANCE_THRESHOLD and (
                abs(right_status[2] - target) >= Config.DISTANCE_THRESHOLD):
            left_status = BP.get_motor_status(Config.LEFT_WHEEL)
            right_status = BP.get_motor_status(Config.RIGHT_WHEEL)
            time.sleep(0.02)

            # TODO: Robot is stuck in here
        self.particles.after_moving_forward(mm)
        self.graphics.draw()

    def move_forward_repeat(self, mm, repeat, pause):
        for _ in range(repeat):
            self.move_forward(mm)
            time.sleep(pause)

    def turn_left(self, degrees):
        BP.offset_motor_encoder(Config.LEFT_WHEEL, BP.get_motor_encoder(Config.LEFT_WHEEL))
        BP.offset_motor_encoder(Config.RIGHT_WHEEL, BP.get_motor_encoder(Config.RIGHT_WHEEL))

        BP.set_motor_limits(Config.LEFT_WHEEL, 60, 120)
        BP.set_motor_limits(Config.RIGHT_WHEEL, 60, 120)

        target_mm = Config.WHEEL_WIDTH * 2 * (degrees / 360)
        target_deg = 180 * target_mm / (math.pi * Config.WHEEL_RADIUS) * Config.TURN_CONSTANT

        print("target : ", target_deg)

        left_status = BP.get_motor_status(Config.LEFT_WHEEL)
        right_status = BP.get_motor_status(Config.RIGHT_WHEEL)

        BP.set_motor_position_kp(Config.LEFT_WHEEL, Config.LW_KP)
        BP.set_motor_position_kp(Config.RIGHT_WHEEL, Config.RW_KP)
        BP.set_motor_position_kd(Config.LEFT_WHEEL, Config.LW_KD)
        BP.set_motor_position_kd(Config.RIGHT_WHEEL, Config.RW_KD)

        BP.set_motor_position(Config.LEFT_WHEEL, -target_deg)
        BP.set_motor_position(Config.RIGHT_WHEEL, target_deg)

        while abs(left_status[2] - target_deg) >= Config.TURN_THRESHOLD and (
                abs(right_status[2] - target_deg) >= Config.TURN_THRESHOLD):
            left_status = BP.get_motor_status(Config.LEFT_WHEEL)
            right_status = BP.get_motor_status(Config.RIGHT_WHEEL)

            # print(left_status[0], left_status[1], left_status[2], left_status[3], right_status[0], right_status[1], right_status[2], right_status[3], sep=",")
            time.sleep(0.02)
        self.particles.after_turning(degrees)
        self.graphics.draw()

    def turn_right(self, degrees):
        BP.offset_motor_encoder(Config.LEFT_WHEEL, BP.get_motor_encoder(Config.LEFT_WHEEL))
        BP.offset_motor_encoder(Config.RIGHT_WHEEL, BP.get_motor_encoder(Config.RIGHT_WHEEL))

        BP.set_motor_limits(Config.LEFT_WHEEL, 60, 120)
        BP.set_motor_limits(Config.RIGHT_WHEEL, 60, 120)

        target_mm = Config.WHEEL_WIDTH * 2 * (degrees / 360)
        target_deg = 180 * target_mm / (math.pi * Config.WHEEL_RADIUS) * Config.TURN_CONSTANT

        print("target : ", target_deg)

        left_status = BP.get_motor_status(Config.LEFT_WHEEL)
        right_status = BP.get_motor_status(Config.RIGHT_WHEEL)

        BP.set_motor_position_kp(Config.LEFT_WHEEL, Config.LW_KP)
        BP.set_motor_position_kp(Config.RIGHT_WHEEL, Config.RW_KP)
        BP.set_motor_position_kd(Config.LEFT_WHEEL, Config.LW_KD)
        BP.set_motor_position_kd(Config.RIGHT_WHEEL, Config.RW_KD)

        BP.set_motor_position(Config.LEFT_WHEEL, target_deg)
        BP.set_motor_position(Config.RIGHT_WHEEL, -target_deg)

        while abs(left_status[2] - target_deg) >= Config.TURN_THRESHOLD and (
                abs(right_status[2] - target_deg) >= Config.TURN_THRESHOLD):
            left_status = BP.get_motor_status(Config.LEFT_WHEEL)
            right_status = BP.get_motor_status(Config.RIGHT_WHEEL)

            # print(left_status[0], left_status[1], left_status[2], left_status[3], right_status[0], right_status[1], right_status[2], right_status[3], sep=",")
            time.sleep(0.02)
        self.particles.after_turning(degrees)
        self.graphics.draw()

    def navigateToWaypoint(self, Wx, Wy):

        (x, y, theta) = self.particles.estimate_position()

        dx = Wx - x
        dy = Wy - y

        # 1. Turn the robot to face the waypoint in a straight line
        absolute_angle_rad = math.atan2(dy, dx)
        turn_angle_deg = (absolute_angle_rad * 180 / math.pi) - theta

        self.turn_left(turn_angle_deg)

        # 2. Move in a straight line
        D = math.sqrt(dx ** 2 + dy ** 2)
        self.move_forward(D)


rob = Robot(5, 5, 5)

waypoints = [(200, 0), (200, -200), (0, 0)]

try:
    for wp in waypoints:
        rob.navigateToWaypoint(*wp)

except KeyboardInterrupt:
    print("Terminated: Ctrl+C pressed")
    BP.reset_all()


