import brickpi3

import random
import math
import time
import numpy as np

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


def mycos(theta):
    return math.cos(theta * math.pi / 360)


def mysin(theta):
    return math.sin(theta * math.pi / 360)


"""
Variables and constants
"""


class Config:
    LEFT_WHEEL = BP.PORT_D
    RIGHT_WHEEL = BP.PORT_C
    ULTRASONIC_SENSOR = BP.PORT_1

    # Degrees per second
    MOVE_DPS = 60
    TURN_DPS = 60

    # Measurements (cm)
    WHEEL_RADIUS = 2.65
    WHEEL_WIDTH = 21.2

    # Dist from sonar to centre (cm)
    T = 11.6

    # Calibration constants
    DIST_CONSTANT = 0.98
    TURN_CONSTANT = 1.30

    # Thresholds
    DISTANCE_THRESHOLD = 2.75
    TURN_THRESHOLD = 4.0

    # Standard deviations
    STDDEV_e = 0.8
    STDDEV_f = 0.5
    STDDEV_g = 1
    STDDEV_SENSOR = 2

    SENSOR_READ_ATTEMPTS = 35


"""
Particle modelling
"""


class ParticleSet:
    NUMBER_OF_PARTICLES = 200

    def __init__(self, e, f, g, origin=(0.0, 0.0, 0.0)):
        self.e_sampler = Sampler(e)
        self.f_sampler = Sampler(f)
        self.g_sampler = Sampler(g)
        self.particles = [origin] * self.NUMBER_OF_PARTICLES
        self.weights = self.init_weights()

    def __iter__(self):
        return iter(self.particles)

    def init_weights(self):
        return [1.0 / self.NUMBER_OF_PARTICLES] * self.NUMBER_OF_PARTICLES

    def estimate_position(self):
        x_bar = 0.0
        y_bar = 0.0
        theta_bar = 0.0
        for (x, y, theta), w in zip(self.particles, self.weights):
            x_bar += x * w
            y_bar += y * w
            theta_bar += theta * w

        return x_bar, y_bar, theta_bar

    def after_moving_forward(self, cm):
        for idx, (x, y, theta) in enumerate(self.particles):
            e = self.e_sampler.sample()
            f = self.f_sampler.sample()
            dx = (cm + e) * np.cos(np.deg2rad(theta))
            dy = (cm + e) * np.sin(np.deg2rad(theta))
            x_new = x + dx
            y_new = y + dy
            theta_new = theta + f

            self.particles[idx] = (x_new, y_new, theta_new)

    def after_turning(self, alpha):
        for idx, (x, y, theta) in enumerate(self.particles):
            g = self.g_sampler.sample()
            theta_new = theta + alpha + g

            self.particles[idx] = (x, y, theta_new)

    def normalise_weights(self):
        sum_w = sum(self.weights)

        if sum_w == 0:
            return self.init_weights()

        for i in range(len(self.weights)):
            self.weights[i] /= sum_w

    def resampling_genetic(self):
        self.normalise_weights()
        cum_sum = 0.0
        cum_sums = []
        for weight in self.weights:
            cum_sum += weight
            cum_sums.append(cum_sum)

        new_particles = []
        for _ in range(self.NUMBER_OF_PARTICLES):
            r = random.random()
            i = 0

            while i + 1 < len(cum_sums) and cum_sums[i] <= r:
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
        self.x_scale = 2
        self.y_scale = -2
        self.lines = lines

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
    def init_bp():
        BP.set_sensor_type(Config.ULTRASONIC_SENSOR, BP.SENSOR_TYPE.NXT_ULTRASONIC)
        BP.set_motor_limits(Config.LEFT_WHEEL, 70, 360)
        BP.set_motor_limits(Config.RIGHT_WHEEL, 70, 360)

    @staticmethod
    def move_forward(cm):
        BP.offset_motor_encoder(Config.LEFT_WHEEL, BP.get_motor_encoder(Config.LEFT_WHEEL))
        BP.offset_motor_encoder(Config.RIGHT_WHEEL, BP.get_motor_encoder(Config.RIGHT_WHEEL))

        target = 180.0 * cm / (math.pi * Config.WHEEL_RADIUS) * Config.DIST_CONSTANT
        print("[forward] target :", target)

        left_status = BP.get_motor_status(Config.LEFT_WHEEL)
        right_status = BP.get_motor_status(Config.RIGHT_WHEEL)

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

        target_cm = Config.WHEEL_WIDTH * 2.0 * (degrees / 360.0)
        target_deg = 180.0 * target_cm / (math.pi * Config.WHEEL_RADIUS) * Config.TURN_CONSTANT

        print("[turn] target :", target_deg)

        left_status = BP.get_motor_status(Config.LEFT_WHEEL)
        right_status = BP.get_motor_status(Config.RIGHT_WHEEL)

        BP.set_motor_position(Config.LEFT_WHEEL, -target_deg)
        BP.set_motor_position(Config.RIGHT_WHEEL, target_deg)

        while abs(left_status[2] - target_deg) >= Config.TURN_THRESHOLD and (
                abs(right_status[2] - target_deg) >= Config.TURN_THRESHOLD):
            left_status = BP.get_motor_status(Config.LEFT_WHEEL)
            right_status = BP.get_motor_status(Config.RIGHT_WHEEL)

            time.sleep(0.02)

    @staticmethod
    def read_sensor():
        counter = 0
        sum = 0.0
        for _ in range(Config.SENSOR_READ_ATTEMPTS):
            v = 0.0
            try:
                v = BP.get_sensor(Config.ULTRASONIC_SENSOR) + Config.T
            except:
                continue

            print(f"sensor read : {v}cm")
            sum += float(v)
            counter += 1
            if counter == 3:
                break

        value = sum / counter
        print(f"Read sensor: {value}cm")
        return value


"""
Controller for everything
"""


class Simulator:
    # Waypoint 1 origin: (84.0, 30.0)
    waypoints = [(180.0, 30.0), (180.0, 54.0), (138.0, 54.0), (138.0, 168.0), (114.0, 168.0), (114.0, 84.0),
                 (84.0, 84.0), (84.0, 30.0)]

    points = {
        "O": (0.0, 0.0),
        "A": (0.0, 168.0),
        "B": (84.0, 168.0),
        "C": (84.0, 126.0),
        "D": (84.0, 210.0),
        "E": (168.0, 210.0),
        "F": (168.0, 84.0),
        "G": (210.0, 84.0),
        "H": (210.0, 0.0),
    }

    walls = [("O", "A"), ("A", "B"), ("B", "C"), ("B", "D"), ("D", "E"), ("E", "F"), ("F", "G"), ("G", "H"), ("H", "O")]

    @staticmethod
    def walls_to_lines():
        return [(*Simulator.points[a], *Simulator.points[b]) for a, b in Simulator.walls]

    def __init__(self, lines=None):
        if lines is None:
            lines = Simulator.walls_to_lines()

        self.particles = ParticleSet(Config.STDDEV_e, Config.STDDEV_f, Config.STDDEV_g, origin=(84.0, 30.0, 0.0))
        self.graphics = Display(lines)
        self.graphics.draw(self.particles)
        Robot.init_bp()

    @staticmethod
    def make_new_square(cm):
        lines = [(0.0, 0.0, 0.0, cm),
                 (0.0, 0.0, cm, 0.0),
                 (cm, 0.0, cm, cm),
                 (0.0, cm, cm, cm)]
        Simulator(lines)

    def run_move_forward_raw(self, cm):
        Robot.move_forward(cm)
        self.particles.after_moving_forward(cm)
        self.graphics.draw(self.particles)
        time.sleep(0.5)
        self.update_weight(Robot.read_sensor())
        self.particles.resampling_genetic()
        self.graphics.draw(self.particles)

    def run_turn_left_raw(self, degrees):
        Robot.turn_left(degrees)
        self.particles.after_turning(degrees)
        self.graphics.draw(self.particles)
        time.sleep(0.5)
        self.update_weight(Robot.read_sensor())
        self.particles.resampling_genetic()
        self.graphics.draw(self.particles)

    def run_move_forward(self, cm):
        while cm > 0:
            dist = min(cm, 30.0)
            self.run_move_forward_raw(dist)
            cm -= dist
            time.sleep(0.1)

    def run_turn_left(self, degrees):
        while abs(degrees) > 0:
            angle = min(abs(degrees), 46.0) * sign(degrees)
            self.run_turn_left_raw(angle)
            degrees -= angle
            time.sleep(0.1)

    def run_navigate_waypoint(self, waypoint, pause_seconds=0.2):
        Wx, Wy = waypoint  # Should be in cm

        (x, y, theta) = self.particles.estimate_position()  # Should be in cm
        dx = Wx - x
        dy = Wy - y

        print("At", x, y, " and going to", Wx, Wy)
        # 1. Turn the robot to face the waypoint in a straight line
        absolute_angle_rad = math.atan2(dy, dx)
        absolute_angle_deg = (absolute_angle_rad * 180.0 / math.pi)

        turn_angle_deg = mymod(absolute_angle_deg - theta)

        print(f"1. run_turn_left {turn_angle_deg}")
        self.run_turn_left(turn_angle_deg)

        # 2. Move in a straight line
        cm = math.sqrt(dx ** 2 + dy ** 2)
        print(f"2. run_move_forward {cm}")
        self.run_move_forward(cm)

        time.sleep(pause_seconds)

    def run_navigate_all_waypoints(self, pause_seconds=0.5):
        for waypoint in self.waypoints:
            self.run_navigate_waypoint(waypoint, pause_seconds)

    def calculate_likelihood_watson(self, x, y, theta, z, doPrint=False):
        for A, B in self.walls:
            Ax, Ay = self.points[A]
            Bx, By = self.points[B]

            part_pos = np.array([x, y]).T
            part_dir = np.array([mycos(theta), mysin(theta)]).T

            wall_pos = np.array([Ax, Ay]).T
            wall_dir = np.array([Ax - Bx, Ay - By]).T

            _, _, rank = np.linalg.lstsq(np.array([part_dir, -wall_dir]).T, part_pos - wall_pos, rcond=None)[:3]
            if rank == 2:
                # particle is pointing to wall for A & B
                beta = math.acos((mycos(theta) * (Ay - By) + mysin(theta) * (Bx - Ax)) / (
                        ((Ay - By) ** 2) + ((Bx - Ax) ** 2)) ** 0.5)

                max_angle = math.pi / 4
                if beta > max_angle:
                    return 1

                m = ((By - Ay) * (Ax - x) - (Bx - Ax) * (Ay - y)) / (
                        (By - Ay) * mycos(theta) - (Bx - Ax) * mysin(theta))

                likelihood = math.exp(-((z - m) ** 2) / (2 * Config.STDDEV_SENSOR ** 2))

                if doPrint:
                    print(self.points[A])
                    print(self.points[B])

                return likelihood

    def calculate_likelihood(self, x, y, theta, z, doPrint=False):
        likelihoods = []
        for A, B in self.walls:
            Ax, Ay = self.points[A]
            Bx, By = self.points[B]

            m = ((By - Ay) * (Ax - x) - (Bx - Ax) * (Ay - y)) / ((By - Ay) * mycos(theta) - (Bx - Ax) * mysin(theta))
            if m < 0:
                m = float('inf')

            beta = math.acos(
                (mycos(theta) * (Ay - By) + mysin(theta) * (Bx - Ax)) / (((Ay - By) ** 2) + ((Bx - Ax) ** 2)) ** 0.5)
            if beta > 50.0 * math.pi / 180.0:
                likelihood = 1
            else:
                likelihood = math.exp(-((z - m) ** 2) / (2 * Config.STDDEV_SENSOR ** 2))
            likelihoods.append(likelihood)

        return max(likelihoods)

    def update_weight(self, z):
        for i, particle in enumerate(self.particles):
            x, y, theta = particle
            self.particles.weights[i] *= self.calculate_likelihood(x, y, theta, z, doPrint=(i % 10 == 0))


sim = Simulator()

try:
    sim.run_navigate_all_waypoints()

except KeyboardInterrupt:
    print("Terminated: Ctrl+C pressed")
    BP.reset_all()

