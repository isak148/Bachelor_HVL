# -*- coding: utf-8 -*-
# Kode hentet fra:
# https://github.com/Hykudoru/MPU6050-Gyro-Motion-Tracking/blob/main/MPU6050-Gyro-Motion-Tracking/KalmanFilter.py
# Original forfatter: Hykudoru

import math
import time

class KalmanFilter:
    # KORRIGERT __init__ (fra forrige svar)
    def __init__(self):
        """
        Initialiserer 1D Kalman filter (versjon fra Hykudoru repo).
        Bruker hardkodede R/Q verdier.
        """
        self.angle = 0.0  # Reset angle
        self.bias = 0.0   # Reset bias
        self.P = [[0.0, 0.0], [0.0, 0.0]]
        self.Q_angle = 0.001 # Repo kalte denne R_angle
        self.Q_bias = 0.003  # Repo kalte denne R_bias
        self.R_measure = 0.03 # Repo kalte denne Q_measure
        self.dt = 0.0
        self.last_time = time.time() # Initialiseres, men dt beregnes på nytt i update

    def update(self, new_angle_acc, new_rate_gyro):
        """
        Oppdaterer filteret med ny måling og gyro-rate.
        Args:
            new_angle_acc: Vinkel beregnet fra akselerometer (grader).
            new_rate_gyro: Vinkelhastighet målt av gyro (grader/sekund).
        Returns:
            float: Det filtrerte vinkelestimatet (grader).
        """
        current_time = time.time()
        self.dt = current_time - self.last_time
        # Sikkerhet for dt
        if self.dt <= 0: self.dt = 0.01
        elif self.dt > 0.5: self.dt = 0.5 # Begrens stor dt
        self.last_time = current_time
        dt = self.dt

        # Prediction Step
        self.angle += dt * (new_rate_gyro - self.bias)
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Update Step (Correction)
        y = new_angle_acc - self.angle
        S = self.P[0][0] + self.R_measure
        K = [0.0, 0.0]
        if abs(S) > 1e-6 :
            K[0] = self.P[0][0] / S
            K[1] = self.P[1][0] / S

        self.angle += K[0] * y
        self.bias += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]
        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle