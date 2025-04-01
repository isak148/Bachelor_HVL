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
