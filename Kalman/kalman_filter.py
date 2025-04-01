import numpy as np
import time
import math

class KalmanFilter:
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure

        self.angle = 0.0  # Kalman-estimert vinkel
        self.bias = 0.0   # Kalman-estimert bias
        self.rate = 0.0   # Ufiltrert rate

        self.P = [[0.0, 0.0], [0.0, 0.0]]  # Feilkovariansmatrise

    def update(self, new_angle, new_rate, dt):
        # Predict
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate

        # Oppdater feilkovariansmatrise
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Beregn Kalman gain
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]

        # Oppdater vinkel og bias med måling
        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Oppdater feilkovariansmatrise igjen
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

class ComplementaryFilterWithKalman:
    def __init__(self, alpha=0.98, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.alpha = alpha  # Komplementær filter alpha-verdi
        self.kalman_filter = KalmanFilter(Q_angle, Q_bias, R_measure)

        # Initialiserte orienteringer
        self.roll = 0.0
        self.pitch = 0.0
        self.last_time = time.time()

    def complementary_filter(self, accel_data, gyro_data, dt):
        # Beregn akselerometervinkler for både pitch og roll
        accel_angle_x = math.atan2(accel_data['y'], accel_data['z']) * (180 / math.pi)
        accel_angle_y = math.atan2(-accel_data['x'], math.sqrt(accel_data['y']**2 + accel_data['z']**2)) * (180 / math.pi)

        # Integrer gyroskopdataene for bare den aksen som roteres
        self.roll += gyro_data['x'] * dt
        self.pitch += gyro_data['y'] * dt

        # Bruk komplementært filter for å kombinere gyro og akselerometer for roll og pitch
        # Hvis vi ikke roterer rundt roll, bruk akselerometeret til å korrigere roll
        if abs(accel_angle_x) < 90:
            self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_angle_x
        if abs(accel_angle_y) < 90:
            self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_angle_y

        return {'roll': self.roll, 'pitch': self.pitch}

    def update(self, accel_data, gyro_data):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0:
            dt = 0.01  # Fallback til 10ms hvis tid mellom målinger er for liten

        # Først bruk komplementært filter
        filtered_orientation = self.complementary_filter(accel_data, gyro_data, dt)

        # Deretter bruk Kalman-filteret for finjustering av de estimerte roll/pitch
        kalman_roll = self.kalman_filter.update(filtered_orientation['roll'], gyro_data['x'], dt)
        kalman_pitch = self.kalman_filter.update(filtered_orientation['pitch'], gyro_data['y'], dt)

        return {'roll': kalman_roll, 'pitch': kalman_pitch}

# --- EKSEMPEL ---
if __name__ == "__main__":
    complementary_kalman_filter = ComplementaryFilterWithKalman()

    while True:
        # Dummydata for testing
        accel_data = {'x': 0.0, 'y': 0.0, 'z': 1.0}  # Akselerometerdata
        gyro_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # Gyroskopdata

        orientation = complementary_kalman_filter.update(accel_data, gyro_data)
        print(f"Roll: {orientation['roll']:.2f}, Pitch: {orientation['pitch']:.2f}")
        time.sleep(0.01)
