# import brickpi3

import random
import math

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

    def __init__(self, s):
        self.sampler = s
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
            e = self.sampler.sample()
            f = self.sampler.sample()

            x_new = x + (D + e) * math.cos(theta)
            y_new = y + (D + e) * math.sin(theta)
            theta_new = theta + f

            self.particles[idx] = (x_new, y_new, theta_new)

    def after_turning(self, alpha):
        for idx, (x, y, theta) in enumerate(self.particles):
            g = sampler.sample()

            theta_new = theta + alpha + g

            self.particles[idx] = (x, y, theta_new)

class DisplaySquare:
    def __init__(self, ps, D):
        self.particles = ps
        self.ofs = 10
        self.scale = 1
        self.D = D

    def draw(self):
        #x0, y0, x1, y1)
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
    def __init__(self, lw, rw, s):
        self.left_wheel = lw
        self.right_wheel = rw
        self.particles = ParticleSet(s)

    def move_forward(self, mm):
        self.particles.after_moving_forward(mm)

    def move_forward_repeat(self, mm, repeat):
        for _ in range(repeat):
            self.move_forward(mm)

    def turn_left(self, degrees):
        self.particles.after_turning(degrees)


sampler = Sampler(1)
rob = Robot(Config.LEFT_WHEEL, Config.RIGHT_WHEEL, sampler)

try:
    for i in range(0, 3):
        rob.move_forward_repeat(100, 4)
        rob.turn_left(90)

    rob.move_forward_repeat(100, 4)

    print(rob.particles.estimate_position())

except KeyboardInterrupt:
    print("Terminated: Ctrl+C pressed")
    # BP.reset_all()


