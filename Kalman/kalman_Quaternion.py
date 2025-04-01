import numpy as np
import time
import math

class KalmanFilterWithQuaternion:
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure

        self.bias = np.zeros(3)  # Bias for gyro x, y, z
        self.P = np.identity(3) * 0.01

        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.last_time = time.time()

    def normalize_quaternion(self, q):
        return q / np.linalg.norm(q)

    def quat_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def update(self, gyro, accel):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Gyro i rad/s og korriger for bias
        gyro_vec = np.array([
            math.radians(gyro['x']) - self.bias[0],
            math.radians(gyro['y']) - self.bias[1],
            math.radians(gyro['z']) - self.bias[2]
        ])

        # Konverter gyro til quaternion-derivat
        omega = np.array([0.0, *gyro_vec])
        q_dot = 0.5 * self.quat_multiply(self.quaternion, omega)
        self.quaternion += q_dot * dt
        self.quaternion = self.normalize_quaternion(self.quaternion)

        # Akselerometer (normalisert)
        accel_vector = np.array([accel['x'], accel['y'], accel['z']])
        accel_mag = np.linalg.norm(accel_vector)
        if accel_mag < 0.7 or accel_mag > 1.3:
            return self._get_angles_from_quaternion()  # hopp korreksjon

        accel_vector /= accel_mag

        # Forventet gravitasjon fra quaternion
        q = self.quaternion
        gravity = np.array([
            2*(q[1]*q[3] - q[0]*q[2]),
            2*(q[0]*q[1] + q[2]*q[3]),
            q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2
        ])

        error = np.cross(gravity, accel_vector)
        S = self.P + self.R_measure * np.identity(3)
        K = self.P @ np.linalg.inv(S)
        correction = K @ error

        # Oppdater gyro-bias
        self.bias += correction
        self.P = (np.identity(3) - K) @ self.P + self.Q_angle * np.identity(3)

        return self._get_angles_from_quaternion()

    def _get_angles_from_quaternion(self):
        q = self.quaternion
        roll = math.degrees(math.atan2(2*(q[0]*q[1] + q[2]*q[3]),
                                       1 - 2*(q[1]**2 + q[2]**2)))
        pitch = math.degrees(math.asin(2*(q[0]*q[2] - q[3]*q[1])))

        return {
            'roll': (roll + 360) % 360,
            'pitch': (pitch + 360) % 360
        }


# --- EKSEMPEL ---
if __name__ == "__main__":
    kalman_q = KalmanFilterWithQuaternion()
    while True:
        gyro_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        accel_data = {'x': 0.0, 'y': 0.0, 'z': 1.0}

        orientation = kalman_q.update(gyro_data, accel_data)
        print("Roll: {:.2f}, Pitch: {:.2f}".format(orientation['roll'], orientation['pitch']))
        time.sleep(0.01)
