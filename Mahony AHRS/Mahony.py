import numpy as np
import math
import time
# Du må fortsatt ha mpu6050-klassen din et sted
# from mpu6050_orientation import MPU6050_Orientation # Eksempel

class MahonyAHRS:
    def __init__(self, sample_freq=100.0, kp=1.0, ki=0.1):
        """
        Initialiserer Mahony AHRS filter.
        Args:
            sample_freq: Samplingsfrekvens i Hz.
            kp: Proposjonal gain for akselerometer/magnetometer fusjon.
            ki: Integral gain for gyro bias korreksjon.
        """
        self.sample_freq = sample_freq
        self.kp = kp
        self.ki = ki
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # Initialkvaternion [w, x, y, z]
        self.integral_fb = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # Integral av feil (gyro bias)
        self.last_time = time.time()
        print(f"MahonyAHRS initialisert med sample_freq={sample_freq}, Kp={kp}, Ki={ki}")

    def update(self, gx, gy, gz, ax, ay, az):
        """
        Oppdaterer filterets tilstand med nye sensoravlesninger.
        Args:
            gx, gy, gz: Gyroskopdata i grader/sekund (kalibrert!).
            ax, ay, az: Akselerometerdata i g (kalibrert!).
        """
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
             # Bruk nominell dt hvis tiden står stille eller går bakover
             dt = 1.0 / self.sample_freq
        elif dt > 0.5: # Hvis dt er urimelig stor (f.eks. etter pause), begrens
            print(f"Advarsel: Stor dt detektert ({dt:.2f}s), begrenser til 0.5s")
            dt = 0.5
        self.last_time = current_time


        q0, q1, q2, q3 = self.q  # Hent ut elementer for lesbarhet

        # Konverter gyro til rad/s
        gx_rad = math.radians(gx)
        gy_rad = math.radians(gy)
        gz_rad = math.radians(gz)

        # Normaliser akselerometermåling
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:  # Unngå deling på null ved fritt fall e.l.
            # print("Advarsel: Akselerometermagnitude nær null.")
            # Hopp over akselerometer-korreksjon hvis magnituden er for liten
            ax = 0; ay = 0; az = 0
        else:
            ax /= norm
            ay /= norm
            az /= norm

        # --- Estimert retning av gravitasjon basert på kvaternion ---
        # Dette er Z-aksen til verdensrammen transformert til sensorrammen
        # via kvaternionen q = [q0, q1, q2, q3]
        # v_sensor = q * [0,0,1] * q_conjugate
        # Forenklet blir Z-komponentene:
        vx = 2.0 * (q1 * q3 - q0 * q2)
        vy = 2.0 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        # --- Feilberegning ---
        # Feilen er kryssproduktet mellom målt retning (akselerometer)
        # og estimert retning (fra kvaternion). Dette gir en rotasjonsfeil.
        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)

        # --- Integral av feil (for gyro bias korreksjon) ---
        # Hvis ki > 0, akkumuler feilen for å estimere bias
        if self.ki > 0.0:
            self.integral_fb[0] += ex * self.ki * dt
            self.integral_fb[1] += ey * self.ki * dt
            self.integral_fb[2] += ez * self.ki * dt
            # Legg til den estimerte biasen til gyro-målingen
            gx_rad += self.integral_fb[0]
            gy_rad += self.integral_fb[1]
            gz_rad += self.integral_fb[2]
        # else: # Hvis ki = 0, ingen bias-korreksjon

        # --- Proposjonal korreksjon ---
        # Legg til den proporsjonale feilen (kp*e) til gyro-målingen
        gx_rad += ex * self.kp
        gy_rad += ey * self.kp
        gz_rad += ez * self.kp

        # --- Integrer gyro-raten for å oppdatere kvaternionen ---
        # Bruker førsteordens approksimasjon q_dot = 0.5 * q * omega
        # omega = [0, gx, gy, gz] (ren kvaternion for vinkelhastighet)
        # Multiplikasjon q * omega:
        # w = -0.5 * (q1*gx + q2*gy + q3*gz)
        # x = 0.5 * (q0*gx + q2*gz - q3*gy)
        # y = 0.5 * (q0*gy - q1*gz + q3*gx)
        # z = 0.5 * (q0*gz + q1*gy - q2*gx)

        delta_q0 = -0.5 * (q1 * gx_rad + q2 * gy_rad + q3 * gz_rad) * dt
        delta_q1 =  0.5 * (q0 * gx_rad + q2 * gz_rad - q3 * gy_rad) * dt
        delta_q2 =  0.5 * (q0 * gy_rad - q1 * gz_rad + q3 * gx_rad) * dt
        delta_q3 =  0.5 * (q0 * gz_rad + q1 * gy_rad - q2 * gx_rad) * dt

        q0 += delta_q0
        q1 += delta_q1
        q2 += delta_q2
        q3 += delta_q3

        # --- Normaliser kvaternionen ---
        # Viktig for å unngå at den "vokser" eller "krymper" pga. numeriske feil
        norm_q = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm_q < 1e-6:
             print("Advarsel: Kvaternionsnorm nær null. Reinitialiserer.")
             self.q = np.array([1.0, 0.0, 0.0, 0.0]) # Reinitialiser til identitet
        else:
            self.q = np.array([q0, q1, q2, q3]) / norm_q

    def get_angles(self):
        """
        Beregner Euler-vinkler (Roll, Pitch, Yaw) fra kvaternionen.
        Returns:
            dict: Dictionary med 'roll', 'pitch', 'yaw' i grader (0-360 range).
                  ADVARSEL: Pitch vil være begrenset til +/- 90 pga. asin().
                  Yaw vil drifte uten magnetometer.
        """
        q0, q1, q2, q3 = self.q

        # Roll (x-axis rotation)
        # atan2(2(q0q1 + q2q3), 1 - 2(q1^2 + q2^2))
        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll_rad = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        # asin(2(q0q2 - q3q1))
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        # Begrens pitch til +/- 90 grader for å unngå feil i asin
        if abs(sinp) >= 1:
            # pitch = math.copysign(math.pi / 2, sinp) # 90/-90 grader
             pitch_rad = math.copysign(math.pi / 2 - 1e-6, sinp) # Litt innenfor for å unngå atan2 feil
        else:
            pitch_rad = math.asin(sinp)

        # Yaw (z-axis rotation)
        # atan2(2(q0q3 + q1q2), 1 - 2(q2^2 + q3^2))
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        # Konverter til grader og map til 0-360
        # Husk at pitch her er fra asin(), så den er i praksis +/- 90
        roll = (math.degrees(roll_rad) + 360) % 360
        pitch = (math.degrees(pitch_rad) + 360) % 360 # Vil kun være i [0,90] U [270, 360]
        yaw = (math.degrees(yaw_rad) + 360) % 360 # Vil drifte!

        return {'roll': roll, 'pitch': pitch, 'yaw': yaw}

# --- EKSEMPEL BRUK ---
if __name__ == "__main__":
    try:
        # 1. Initialiser MPU6050-klassen din (som har kalibrering)
        # Anta at du har en klasse MPU6050_Orientation som før
        from mpu6050_orientation import MPU6050_Orientation # BYTT TIL RIKTIG IMPORT
        mpu = MPU6050_Orientation(0x68) # Kalibrering skjer inni __init__

        # 2. Initialiser MahonyAHRS
        # Juster kp og ki etter behov. Startverdier er ofte OK.
        # kp høyere = stoler mer på akselerometer (raskere korreksjon, mer støy)
        # ki høyere = korrigerer bias raskere (kan bli ustabilt hvis for høyt)
        mahony = MahonyAHRS(sample_freq=100.0, kp=2.0, ki=0.2)

        print("Starter Mahony AHRS loop...")
        while True:
            # 3. Hent RÅ sensor data
            accel_data_raw = mpu.get_accel_data(g=True) # Hent i g enheter
            gyro_data_raw = mpu.get_gyro_data()       # Hent i grader/sekund

            # 4. ANVEND KALIBRERING (VIKTIG!)
            # Bruk offsettene som ble beregnet i mpu.__init__
            accel_cal = {
                'x': accel_data_raw['x'] - mpu.accel_offset['x'],
                'y': accel_data_raw['y'] - mpu.accel_offset['y'],
                'z': accel_data_raw['z'] - mpu.accel_offset['z']
            }
            gyro_cal = {
                'x': gyro_data_raw['x'] - mpu.gyro_offset['x'],
                'y': gyro_data_raw['y'] - mpu.gyro_offset['y'],
                'z': gyro_data_raw['z'] - mpu.gyro_offset['z']
            }

            # 5. Oppdater Mahony filteret med KALIBRERTE data
            mahony.update(gyro_cal['x'], gyro_cal['y'], gyro_cal['z'],
                          accel_cal['x'], accel_cal['y'], accel_cal['z'])

            # 6. Hent ut vinkler
            angles = mahony.get_angles()

            # 7. Skriv ut resultater
            # Husk: Pitch er effektivt +/- 90, Yaw drifter.
            print(f"Roll: {angles['roll']:6.1f}, Pitch: {angles['pitch']:6.1f}, Yaw: {angles['yaw']:6.1f}  | Bias: {mahony.integral_fb[0]:6.2f}, {mahony.integral_fb[1]:6.2f}, {mahony.integral_fb[2]:6.2f}")

            # Vent litt for å prøve å holde samplingsraten
            time.sleep(max(0, (1.0 / mahony.sample_freq) - (time.time() - mahony.last_time)))

    except ImportError:
        print("Kunne ikke importere MPU6050 klassen.")
    except Exception as e:
        print(f"En feil oppstod: {e}")
        import traceback
        traceback.print_exc()