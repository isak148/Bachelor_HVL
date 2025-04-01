# -*- coding: utf-8 -*-
# Kode hentet fra:
# https://github.com/Hykudoru/MPU6050-Gyro-Motion-Tracking/blob/main/MPU6050-Gyro-Motion-Tracking/KalmanFilter.py
# Original forfatter: Hykudoru

import math
import time # Selv om time ikke brukes direkte i klassen her

class KalmanFilter:

    def __init__(self):
        """
        Initialiserer 1D Kalman filter (versjon fra Hykudoru repo).
        Bruker hardkodede R/Q verdier.
        """
        self.angle = 0.0  # Reset angle
        self.bias = 0.0   # Reset bias

        # Covariance matrix
        self.P = [[0.0, 0.0], [0.0, 0.0]]

        # Hardkodede verdier fra original repo
        # (Repo brukte R_ for prosess, Q_ for measure - motsatt av standard)
        # Vi bruker standard navn internt, men setter verdiene fra repoet.
        self.Q_angle = 0.001 # Repo kalte denne R_angle (prosess)
        self.Q_bias = 0.003  # Repo kalte denne R_bias (prosess)
        self.R_measure = 0.03 # Repo kalte denne Q_measure (måling)

        self.dt = 0.0
        self.last_time = time.time()

    def update(self, new_angle_acc, new_rate_gyro):
        """
        Oppdaterer filteret med ny måling og gyro-rate.
        Args:
            new_angle_acc: Vinkel beregnet fra akselerometer (grader).
            new_rate_gyro: Vinkelhastighet målt av gyro (grader/sekund).
        Returns:
            float: Det filtrerte vinkelestimatet (grader).
        """
        # Calculate dt
        current_time = time.time()
        self.dt = current_time - self.last_time
        if self.dt <= 0: self.dt = 0.01 # fallback dt
        self.last_time = current_time

        dt = self.dt

        # Prediction Step
        # Update angle estimate with gyro rate (and subtract bias)
        self.angle += dt * (new_rate_gyro - self.bias)

        # Update covariance matrix P - Prediction part
        # P = F * P * F^T + Q
        # F = [[1, -dt], [0, 1]]
        # Q = [[Q_angle, 0], [0, Q_bias]] * dt (repo skalerer ikke Q med dt, men det er vanlig)
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle) # Skalert med dt?
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt # Skalert med dt?

        # Update Step (Correction)
        # Calculate innovation (measurement residual)
        y = new_angle_acc - self.angle

        # Calculate innovation covariance S
        # S = H * P * H^T + R
        # H = [1 0]
        # R = R_measure
        S = self.P[0][0] + self.R_measure

        # Calculate Kalman gain K
        # K = P * H^T * S^-1
        K = [0.0, 0.0]
        if abs(S) > 1e-6 : # Avoid division by zero
            K[0] = self.P[0][0] / S
            K[1] = self.P[1][0] / S
        else:
             # Håndter S nær null (f.eks. sett gain til 0 eller en liten verdi)
            # print("Advarsel: S er nær null i Kalman filter.")
            pass


        # Update angle estimate x_new = x_pred + K * y
        self.angle += K[0] * y

        # Update bias estimate
        self.bias += K[1] * y

        # Update covariance matrix P - Update part
        # P_new = (I - K * H) * P_pred
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle