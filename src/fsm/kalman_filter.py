import numpy as np


class KalmanFilter:
    """
    State vector:
        x = [position, velocity, acceleration]
    """

    def __init__(self):
        self.x = np.zeros((3, 1))

        self.P = np.array([
            [1, 0, 0],
            [0, 0.001, 0],
            [0, 0, 0.0001]
        ], dtype=float)

        self.Q = np.array([
            [0.001, 0, 0],
            [0, 0.001, 0],
            [0, 0, 100]
        ], dtype=float)

        self.H = np.array([
            [1, 0, 0],  # baro (position)
            [0, 0, 1]   # accel (acceleration)
        ], dtype=float)

        self.R = np.array([
            [0.1, 0],
            [0, 0.01]
        ], dtype=float)

        self.F = np.eye(3)

    def predict(self, dt: float):
        self.F = np.array([
            [1, dt, 0.5 * dt * dt],
            [0, 1, dt],
            [0, 0, 1]
        ], dtype=float)

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z_baro: float, z_accel: float):
        z = np.array([[z_baro], [z_accel]])

        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        y = z - (self.H @ self.x)

        self.x = self.x + K @ y

        I = np.eye(3)
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ self.R @ K.T

    def get(self):
        return {
            "altitude": float(self.x[0, 0]),
            "velocity": float(self.x[1, 0]),
            "acceleration": float(self.x[2, 0]),
        }