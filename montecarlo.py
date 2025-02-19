import brickpi3

import random
import math
import time

#### BrickPi3 init

BP = brickpi3.BrickPi3()
BP.reset_all()

print("BrickPi3 loaded")

"""
Custom maths functions
"""
def sign(x):
    if x > 0:
        return 1
    else:
        return -1

def mymod(theta):
    while abs(theta) > 180:
        theta -= sign(theta) * 360
    return theta

"""
Variables and constants
"""
class Config:
    LEFT_WHEEL = BP.PORT_D
    RIGHT_WHEEL = BP.PORT_C
    ULTRASONIC_SENSOR = BP.PORT_2

    # Degrees per second
    MOVE_DPS = 180
    TURN_DPS = 180

    # Measurements
    WHEEL_RADIUS = 26.5  ## mm
    WHEEL_WIDTH = 212

    # Dist from sonar to centre (mm)
    T = 65

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

    # Standard deviations
    STDDEV_e = 1
    STDDEV_f = 1
    STDDEV_g = 1



"""
Particle modelling
"""
class ParticleSet:
    NUMBER_OF_PARTICLES = 100

    def __init__(self, e, f, g, origin=(0.0, 0.0, 0.0)):
        self.e_sampler = Sampler(e)
        self.f_sampler = Sampler(f)
        self.g_sampler = Sampler(g)
        self.particles = [origin] * self.NUMBER_OF_PARTICLES
        self.weights = self.init_weights()

    def __iter__(self):
        return iter(self.particles)
    
    def init_weights(self):
        return [1 / self.NUMBER_OF_PARTICLES] * self.NUMBER_OF_PARTICLES

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

    def normalise_weights(self):
        sum_w = sum(self.weights)

        for i in range(len(self.weights)):
            self.weights[i] /= sum_w

    def resampling_genetic(self):
        cum_sum = 0
        cum_sums = []
        for weight in self.weights:
            cum_sum += weight
            cum_sums.append(cum_sum)

        new_particles = []
        for _ in range(self.NUMBER_OF_PARTICLES):
            r = random.random()
            i = 0

            while cum_sums[i] <= r:
                i += 1

            new_particles.append(self.particles[i])

        self.particles = new_particles
        self.weights = self.init_weights()


"""
Graphics handler
"""
class Display:
    def __init__(self, lines):
        self.x_ofs = 50
        self.y_ofs = 500
        self.x_scale = 1
        self.y_scale = -1
        self.lines = lines

    @staticmethod
    def make_new_square(D):
        lines = [(0, 0, 0, D),
                (0, 0, D, 0),
                (D, 0, D, D),
                (0, D, D, D)]

        return Display(lines)

    def draw(self, ps):
        lines_transformed = [(x0 * self.x_scale + self.x_ofs,
                              y0 * self.y_scale + self.y_ofs,
                              x1 * self.x_scale + self.x_ofs,
                              y1 * self.y_scale + self.y_ofs)
                             for x0, y0, x1, y1 in self.lines]

        for line in lines_transformed:
            print("drawLine:" + str(line))

        particles_transformed = [(x * self.x_scale + self.x_ofs,
                                  y * self.y_scale + self.y_ofs,
                                  theta)
                                 for x, y, theta in ps]

        print("drawParticles:" + str(particles_transformed))


"""
Gaussian sampler
"""
class Sampler:
    def __init__(self, sigma):
        self.sigma = sigma

    def sample(self):
        return random.gauss(mu=0, sigma=self.sigma)


"""
Robot commands
"""
class Robot:
    @staticmethod
    def move_forward(mm):
        BP.offset_motor_encoder(Config.LEFT_WHEEL, BP.get_motor_encoder(Config.LEFT_WHEEL))
        BP.offset_motor_encoder(Config.RIGHT_WHEEL, BP.get_motor_encoder(Config.RIGHT_WHEEL))

        target = 180 * mm / (math.pi * Config.WHEEL_RADIUS) * Config.DIST_CONSTANT
        print("[forward] target :", target)

        left_status = BP.get_motor_status(Config.LEFT_WHEEL)
        right_status = BP.get_motor_status(Config.RIGHT_WHEEL)

        BP.set_motor_position_kp(Config.LEFT_WHEEL, Config.LW_KP)
        BP.set_motor_position_kp(Config.RIGHT_WHEEL, Config.RW_KP)
        BP.set_motor_position_kd(Config.LEFT_WHEEL, Config.LW_KD)
        BP.set_motor_position_kd(Config.RIGHT_WHEEL, Config.RW_KD)

        BP.set_motor_limits(Config.LEFT_WHEEL, 70, 180)
        BP.set_motor_limits(Config.RIGHT_WHEEL, 70, 180)

        BP.set_motor_position(Config.LEFT_WHEEL, target)
        BP.set_motor_position(Config.RIGHT_WHEEL, target)

        while abs(left_status[2] - target) >= Config.DISTANCE_THRESHOLD and (
                abs(right_status[2] - target) >= Config.DISTANCE_THRESHOLD):
            left_status = BP.get_motor_status(Config.LEFT_WHEEL)
            right_status = BP.get_motor_status(Config.RIGHT_WHEEL)
            time.sleep(0.02)

    @staticmethod
    def turn_left(degrees):
        BP.offset_motor_encoder(Config.LEFT_WHEEL, BP.get_motor_encoder(Config.LEFT_WHEEL))
        BP.offset_motor_encoder(Config.RIGHT_WHEEL, BP.get_motor_encoder(Config.RIGHT_WHEEL))

        BP.set_motor_limits(Config.LEFT_WHEEL, 60, 120)
        BP.set_motor_limits(Config.RIGHT_WHEEL, 60, 120)

        target_mm = Config.WHEEL_WIDTH * 2 * (degrees / 360)
        target_deg = 180 * target_mm / (math.pi * Config.WHEEL_RADIUS) * Config.TURN_CONSTANT

        print("[turn] target :", target_deg)

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

            time.sleep(0.02)

    @staticmethod
    def read_sensor():
        return BP.get_sensor(Config.ULTRASONIC_SENSOR) * 10 + Config.T


"""
Controller for everything
"""
class Simulator:
    waypoints = [(84, 30), (180, 30), (180, 54), (138, 54), (138, 168), (114, 168), (114, 84), (84, 84), (84, 30)]

    points = {
        "O": (0, 0),
        "A": (0, 168),
        "B": (84, 168),
        "C": (84, 126),
        "D": (84, 210),
        "E": (168, 210),
        "F": (168, 84),
        "G": (210, 84),
        "H": (210, 0),
    }

    def __init__(self):
        self.particles = ParticleSet(Config.STDDEV_e, Config.STDDEV_f, Config.STDDEV_g)
        self.graphics = Display.make_new_square(400)

    def run_move_forward(self, mm):
        Robot.move_forward(mm)
        self.particles.after_moving_forward(mm)
        self.graphics.draw(self.particles)

    def run_turn_left(self, degrees):
        Robot.turn_left(degrees)
        self.particles.after_turning(degrees)
        self.graphics.draw(self.particles)

    def run_navigate_waypoint(self, waypoint, pause_seconds=0.2):
        Wx, Wy = waypoint

        (x, y, theta) = self.particles.estimate_position()
        dx = Wx - x
        dy = Wy - y

        # 1. Turn the robot to face the waypoint in a straight line
        absolute_angle_rad = math.atan2(dy, dx)
        absolute_angle_deg = (absolute_angle_rad * 180 / math.pi)

        turn_angle_deg = mymod(absolute_angle_deg - theta)

        self.run_turn_left(turn_angle_deg)

        # 2. Move in a straight line
        D = math.sqrt(dx ** 2 + dy ** 2)
        self.run_move_forward(D)

        time.sleep(pause_seconds)

    def run_navigate_all_waypoints(self, pause_seconds=0.2):
        for waypoint in self.waypoints:
            self.run_navigate_waypoint(waypoint, pause_seconds)

    def calculate_likelihood(x, y, theta, z):
        for particle in self.particles:
            for (point, (x,y)) in points:
                z = pass

        likelihood = math.exp(-((z-m)**2) / (2 * (sigma)**2))

        #m = ...  # calculated from x, y, theta and points
        # NOTE UPDATE M USING T
        #z - m
        # maybe check incidence angle
        # if incidence too high return 1

        # calc likelihood using gaussian (with constant) use sd (2-3cm)

    def update_weight(self, z):
        for particle in self.particles:
            x, y, theta = particle.x, particle.y, particle.theta
        cur_weight *= self.calculate_likelihood(x, y, theta, Robot.read_sensor())


sim = Simulator()

try:
    sim.run_navigate_all_waypoints()

except KeyboardInterrupt:
    print("Terminated: Ctrl+C pressed")
    BP.reset_all()

