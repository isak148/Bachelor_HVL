# -*- coding: utf-8 -*-
# Kode hentet fra:
# https://github.com/Hykudoru/MPU6050-Gyro-Motion-Tracking/blob/main/MPU6050-Gyro-Motion-Tracking/KalmanFilter.py
# Original forfatter: Hykudoru

import math
import time # Selv om time ikke brukes direkte i klassen her

class KalmanFilter:
    def __init__(self, R_angle=0.001, R_bias=0.003, R_measure=0.03):
        """
        Initialiserer 1D Kalman filter for vinkelestimering.
        Args:
            R_angle: Usikkerhet i vinkelestimat fra modellen (gyro integrasjon).
            R_bias: Usikkerhet i bias estimat (hvor mye bias drifter).
            R_measure: Usikkerhet i målingen (akselerometer vinkel).
        """
        self.angle = 0.0  # Reset angle
        self.bias = 0.0   # Reset bias

        # Covariance matrix
        self.P = [[0.0, 0.0], [0.0, 0.0]]

        # Process noise covariance constants (R i repo, men Q i standard Kalman)
        self.Q_angle = R_angle # Endret navn for klarhet
        self.Q_bias = R_bias   # Endret navn for klarhet

        # Measurement noise covariance constant (Q i repo, men R i standard Kalman)
        self.R_measure = R_measure # Endret navn for klarhet

        self.dt = 0.0
        self.last_time = time.time() # Brukes ikke internt, men greit å ha

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