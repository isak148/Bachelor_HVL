# -*- coding: utf-8 -*-
# Kode hentet fra:
# https://github.com/Hykudoru/MPU6050-Gyro-Motion-Tracking/blob/main/MPU6050-Gyro-Motion-Tracking/KalmanFilter.py
# Original forfatter: Hykudoru
# MODIFISERT for å dynamisk justere R_measure

import math
import time

class KalmanFilter:
    def __init__(self):
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        # Lagre den *opprinnelige* R_measure
        self._R_measure_base = 0.03
        self.dt = 0.0
        self.last_time = time.time()

    # ENDRE SIGNATUREN: Legg til accel_z
    def update(self, new_angle_acc, new_rate_gyro, accel_z):
        """
        Oppdaterer filteret med ny måling og gyro-rate.
        Args:
            new_angle_acc: Vinkel beregnet fra akselerometer (grader).
            new_rate_gyro: Vinkelhastighet målt av gyro (grader/sekund).
            accel_z: Den kalibrerte Z-akselerasjonsverdien (i g).
        Returns:
            float: Det filtrerte vinkelestimatet (grader).
        """
        current_time = time.time()
        self.dt = current_time - self.last_time
        if self.dt <= 0: self.dt = 0.01
        elif self.dt > 0.5: self.dt = 0.5
        self.last_time = current_time
        dt = self.dt

        # --- Dynamisk R_measure ---
        # Sett en terskel for når z er "for liten" (f.eks. 0.15g ~ 8.6 grader fra 90)
        z_threshold = 0.15
        # Faktor for å øke R_measure (f.eks. 100x mindre tillit)
        r_increase_factor = 100

        # Bruk høyere R_measure hvis z er nær 0, ellers bruk baseverdien
        if abs(accel_z) < z_threshold:
            current_R_measure = self._R_measure_base * r_increase_factor
        else:
            current_R_measure = self._R_measure_base
        # --------------------------

        # Prediction Step
        self.angle += dt * (new_rate_gyro - self.bias)
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Update Step (Correction)
        y = new_angle_acc - self.angle
        # Bruk den dynamisk justerte R_measure her:
        S = self.P[0][0] + current_R_measure
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