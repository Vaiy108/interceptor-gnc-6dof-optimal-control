import numpy as np

from src.dynamics.rotation_utils import euler_to_rotation_matrix


class Interceptor6DOF:

    def __init__(self):

        self.pos = np.array([0.0, 0.0, 0.0])

        self.vel = np.array([300.0, 0.0, 0.0])

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        self.mass = 100.0

        self.gravity = np.array([0.0, 0.0, -9.81])

    def step(self, acceleration_command, dt):

        max_acc = 120.0

        acc_norm = np.linalg.norm(acceleration_command)

        if acc_norm > max_acc:
            acceleration_command = (
                acceleration_command / acc_norm * max_acc
            )

        #total_acc = acceleration_command + self.gravity
        total_acc = acceleration_command

        self.vel += total_acc * dt

        self.pos += self.vel * dt

        speed = np.linalg.norm(self.vel)

        if speed > 1e-6:

            #self.pitch = np.arcsin(self.vel[2] / speed)
            self.pitch = np.arcsin(np.clip(self.vel[2] / speed, -1.0, 1.0))

            self.yaw = np.arctan2(self.vel[1], self.vel[0])

        self.q = self.pitch
        self.r = self.yaw

    def get_state(self):

        return {
            "position": self.pos.copy(),
            "velocity": self.vel.copy(),
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "p": self.p,
            "q": self.q,
            "r": self.r
        }