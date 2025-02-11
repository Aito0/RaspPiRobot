# import brickpi3

import random
import math
import time


#### BrickPi3 init

# BP = brickpi3.BrickPi3()
# BP.reset_all()
#
# print("BrickPi3 loaded")

#### Variables and constants
class Config:
    LEFT_WHEEL = 0#BP.PORT_D
    RIGHT_WHEEL = 0#BP.PORT_A

    # Degrees per second
    MOVE_DPS = 120
    TURN_DPS = 120

    # Measurements
    WHEEL_RADIUS = 23  ## mm
    TIRE_WIDTH = 99999

#### Particle set

class ParticleSet:
    NUMBER_OF_PARTICLES = 100

    def __init__(self, e_sampler, f_sampler, g_sampler):
        self.e_sampler = e_sampler
        self.f_sampler = f_sampler
        self.g_sampler = g_sampler
        self.particles = [(0.0, 0.0, 0.0)] * self.NUMBER_OF_PARTICLES
        self.weights = [1 / self.NUMBER_OF_PARTICLES] * self.NUMBER_OF_PARTICLES

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
        self.ofs = 10
        self.scale = 1
        self.D = D

    def draw(self):
        # x0, y0, x1, y1
        lines = [
            (0, 0, 0, self.D),
            (0, 0, self.D, 0),
            (self.D, 0, self.D, self.D),
            (0, self.D, self.D, self.D)
        ]
        for line in lines:
            print("drawLine:" + str(line))

        print("drawParticles:" + str(self.particles))


class Sampler:
    def __init__(self, sigma):
        self.sigma = sigma

    def sample(self):
        return random.gauss(mu=0, sigma=self.sigma)

####
class Robot:
    def __init__(self, lw, rw, stddev_e, stddev_f, stddev_g):
        self.left_wheel = lw
        self.right_wheel = rw
        self.particles = ParticleSet(Sampler(stddev_e), Sampler(stddev_f), Sampler(stddev_g))
        self.graphics = DisplaySquare(self.particles, 400)

    def move_forward(self, mm):
        self.particles.after_moving_forward(mm)
        # self.graphics.draw()

    def move_forward_repeat(self, mm, repeat, pause):
        for _ in range(repeat):
            self.move_forward(mm)
            time.sleep(pause)

    def turn_left(self, degrees):
        self.particles.after_turning(degrees)
        # self.graphics.draw()

    # TODO
    def navigateToWaypoint(self, Wx, Wy):
        (x, y, theta) = self.particles.estimate_position()

        dx = Wx - x
        dy = Wy - y

        # 1. Turn the robot to face the waypoint in a straight line
        turn_angle_rad = math.atan(dy / dx)
        self.turn_left(turn_angle_rad * 180 / math.pi)

        # 2. Move in a straight line
        D = math.sqrt(dx ** 2 + dy ** 2)
        self.move_forward(D)


rob = Robot(Config.LEFT_WHEEL, Config.RIGHT_WHEEL, 5, 5, 5)

try:
    for i in range(0, 3):
        rob.move_forward_repeat(100, 4, 0.5)
        rob.turn_left(90)
    rob.turn_left(180)

    rob.move_forward_repeat(100, 4, 0.5)

    print(rob.particles.estimate_position())

except KeyboardInterrupt:
    print("Terminated: Ctrl+C pressed")
    # BP.reset_all()


