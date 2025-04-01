import numpy as np
import time
import math

class KalmanFilterWithRotation:
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure

        self.bias = np.zeros(2)  # Bias for roll og pitch
        self.P = np.identity(2) * 0.01
        self.angle = np.zeros(2)  # [roll, pitch]

        self.rotation_matrix = np.identity(3)
        self.last_time = time.time()

    def update(self, gyro, accel):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Gyrodata i rad/s
        gx = math.radians(gyro['x'])
        gy = math.radians(gyro['y'])
        gz = math.radians(gyro['z'])
        omega = np.array([gx, gy, gz])

        # Skap en antisymmetrisk matrise for rotasjon
        omega_mat = np.array([
            [0, -gz, gy],
            [gz, 0, -gx],
            [-gy, gx, 0]
        ])

        # Oppdater rotasjonsmatrisen
        self.rotation_matrix += omega_mat @ self.rotation_matrix * dt

        # Ortogonaliser (hold matrisen ren)
        u, _, v = np.linalg.svd(self.rotation_matrix)
        self.rotation_matrix = u @ v

        # Estimert gravitasjon fra rotasjonsmatrise
        est_gravity = self.rotation_matrix[:, 2]  # z-akse

        # Normaliser akselerometer
        accel_vector = np.array([accel['x'], accel['y'], accel['z']])
        accel_vector = accel_vector / np.linalg.norm(accel_vector)

        # Kalman filter korreksjon (kun X og Y aksene)
        y = accel_vector[:2] - est_gravity[:2]  # Innovasjon
        S = self.P + self.R_measure * np.identity(2)
        K = self.P @ np.linalg.inv(S)

        correction = K @ y

        # Lag liten rotasjon fra korreksjon (kun roll/pitch)
        delta_rot = np.array([
            [0, -correction[1], correction[0]],
            [correction[1], 0, 0],
            [-correction[0], 0, 0]
        ])

        self.rotation_matrix += delta_rot @ self.rotation_matrix
        u, _, v = np.linalg.svd(self.rotation_matrix)
        self.rotation_matrix = u @ v

        # Oppdater P
        self.P = (np.identity(2) - K) @ self.P + self.Q_angle * np.identity(2)

        # Hent roll og pitch fra rotasjonsmatrisen (0–360° støttet)
        roll_rad = math.atan2(self.rotation_matrix[2][1], self.rotation_matrix[2][2])
        pitch_rad = math.atan2(-self.rotation_matrix[2][0], 
                               math.sqrt(self.rotation_matrix[2][1] ** 2 + self.rotation_matrix[2][2] ** 2))

        roll = (math.degrees(roll_rad) + 360) % 360
        pitch = (math.degrees(pitch_rad) + 360) % 360

        return {'roll': roll, 'pitch': pitch}


# --- EKSEMPEL ---
if __name__ == "__main__":
    kalman_rot = KalmanFilterWithRotation()
    while True:
        gyro_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        accel_data = {'x': 0.0, 'y': 0.0, 'z': 1.0}

        orientation = kalman_rot.update(gyro_data, accel_data)
        print("Roll: {:.2f}, Pitch: {:.2f}".format(orientation['roll'], orientation['pitch']))
        time.sleep(0.01)
