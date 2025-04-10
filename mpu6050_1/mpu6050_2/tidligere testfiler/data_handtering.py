import time
import csv
import os
from mpu6050 import mpu6050
import math
import numpy as np
from scipy.signal import butter, lfilter
from scipy.fft import fft
from Bachelor_test1 import MPU6050_Orientation
from time import sleep



def butter_lowpass_filter(data, cutoff, fs, order=5):
        """
        Bruker et Butterworth lavpassfilter.

        data: Dataene som skal filtreres.
        cutoff: Avskjæringsfrekvensen i Hz.
        fs: Samplingsfrekvensen i Hz.
        order: Filterets orden.
        """
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = lfilter(b, a, data)
        return y

class MPU6050_DataLogger(mpu6050):
    def __init__(self, address, bus=1, filename="mpu6050_data.csv", sample_rate=100.0, window_size=50):
        super().__init__(address, bus)
        self.filename = filename
        self.log_file = None
        self.csv_writer = None
        self.is_logging = False
        self.sample_rate = sample_rate  # Lagre samplingsfrekvensen
        self.window_size = window_size  # Størrelse på vindu for funksjonsberegning
        self.accel_buffer_x
        self.accel_buffer_y
        self.accel_buffer_z
        self.gyro_buffer_x 
        self.gyro_buffer_y 
        self.gyro_buffer_z 

    def start_logging(self):
        """Starter datalogging til en CSV-fil."""
        self.log_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            "Timestamp", "Accel_X", "Accel_Y", "Accel_Z",
            "Gyro_X", "Gyro_Y", "Gyro_Z"
        ])  # Skriv overskriftsrad
        self.is_logging = True
        print("Datalogging startet.")

    def stop_logging(self):
        """Stopper datalogging og lukker filen."""
        if self.is_logging:
            self.log_file.close()
            self.is_logging = False
            print("Datalogging stoppet.")

    def log_data(self):
        """Leser sensordata og skriver dem til CSV-filen."""
        if self.is_logging:
            accel_data = self.get_accel_data()
            gyro_data = self.get_gyro_data()
            timestamp = time.time()
            self.csv_writer.writerow([
                timestamp,
                accel_data['x'], accel_data['y'], accel_data['z'],
                gyro_data['x'], gyro_data['y'], gyro_data['z']
            ])

    def run(self, duration):
        """Kjører datalogging for en gitt varighet."""
        self.start_logging()
        start_time = time.time()
        while time.time() - start_time < duration:
            self.log_data()
            time.sleep(0.01)  # Sampler med 100Hz
        self.stop_logging()

       
    def log_data(self):
        """Leser sensordata, filtrerer og skriver dem til CSV-filen."""
        if self.is_logging:
            accel_data = self.get_accel_data()
            gyro_data = self.get_gyro_data()
            

            # Filtrer dataene
            accel_x_filtered = butter_lowpass_filter(
                [accel_data['x']], cutoff=5, fs=self.sample_rate, order=4)[0]
            accel_y_filtered = butter_lowpass_filter(
                [accel_data['y']], cutoff=5, fs=self.sample_rate, order=4)[0]
            accel_z_filtered = butter_lowpass_filter(
                [accel_data['z']], cutoff=5, fs=self.sample_rate, order=4)[0]

            # Beregn total akselerasjon
            total_accel = math.sqrt(
                accel_x_filtered**2 + accel_y_filtered**2 + accel_z_filtered**2)

            timestamp = time.time()
            self.csv_writer.writerow([
                timestamp,
                accel_x_filtered, accel_y_filtered, accel_z_filtered,
                gyro_data['x'], gyro_data['y'], gyro_data['z'], total_accel
            ])


    def calculate_features(self):
        """Beregner funksjoner fra dataene i bufferen."""
        if len(self.accel_buffer_x) < self.window_size:
            return None  # Ikke nok data

        # Akselerasjonsfunksjoner
        accel_mean_x = np.mean(self.accel_buffer_x)
        accel_var_x = np.var(self.accel_buffer_x)
        accel_mean_y = np.mean(self.accel_buffer_y)
        accel_var_y = np.var(self.accel_buffer_y)
        accel_mean_z = np.mean(self.accel_buffer_z)
        accel_var_z = np.var(self.accel_buffer_z)

        # Gyrofunksjoner
        gyro_mean_x = np.mean(self.gyro_buffer_x)
        gyro_var_x = np.var(self.gyro_buffer_x)
        gyro_mean_y = np.mean(self.gyro_buffer_y)
        gyro_var_y = np.var(self.gyro_buffer_y)
        gyro_mean_z = np.mean(self.gyro_buffer_z)
        gyro_var_z = np.var(self.gyro_buffer_z)

        # Frekvensdomene (eksempel for accel_x)
        fft_accel_x = fft(self.accel_buffer_x)
        dominant_freq_accel_x = np.abs(fft_accel_x[1])  # Grunnfrekvens

        return {
            "accel_mean_x": accel_mean_x, "accel_var_x": accel_var_x,
            "accel_mean_y": accel_mean_y, "accel_var_y": accel_var_y,
            "accel_mean_z": accel_mean_z, "accel_var_z": accel_var_z,
            "gyro_mean_x": gyro_mean_x, "gyro_var_x": gyro_var_x,
            "gyro_mean_y": gyro_mean_y, "gyro_var_y": gyro_var_y,
            "gyro_mean_z": gyro_mean_z, "gyro_var_z": gyro_var_z,
            "dominant_freq_accel_x": dominant_freq_accel_x
        }

    def log_data(self):
        """Leser sensordata, filtrerer, legger til buffere og skriver dem til CSV-filen."""
        if self.is_logging:
            accel_data = self.get_accel_data()
            gyro_data = self.get_gyro_data()

            # Filtrer dataene
            accel_x_filtered = butter_lowpass_filter(
                [accel_data['x']], cutoff=5, fs=self.sample_rate, order=4
            )[0]
            accel_y_filtered = butter_lowpass_filter(
                [accel_data['y']], cutoff=5, fs=self.sample_rate, order=4
            )[0]
            accel_z_filtered = butter_lowpass_filter(
                [accel_data['z']], cutoff=5, fs=self.sample_rate, order=4
            )[0]

            # Legg til data i buffere
            self.accel_buffer_x.append(accel_x_filtered)
            self.accel_buffer_y.append(accel_y_filtered)
            self.accel_buffer_z.append(accel_z_filtered)
            self.gyro_buffer_x.append(gyro_data['x'])
            self.gyro_buffer_y.append(gyro_data['y'])
            self.gyro_buffer_z.append(gyro_data['z'])

            # Begrens bufferstørrelsen
            if len(self.accel_buffer_x) > self.window_size:
                self.accel_buffer_x.pop(0)
                self.accel_buffer_y.pop(0)
                self.accel_buffer_z.pop(0)
                self.gyro_buffer_x.pop(0)
                self.gyro_buffer_y.pop(0)
                self.gyro_buffer_z.pop(0)

            timestamp = time.time()
            features = self.calculate_features()

            if features:
                self.csv_writer.writerow(
                    [timestamp] +
                    [round(value, 4) for value in features.values()]
                    
                )


def gjenkjenn_aktivitet(orientation, accel_data, gyro_data, threshold_orientering=20.0, threshold_accel_variasjon=0.2):
    """
    Gjenkjenner svømmeaktivitet basert på orientering og akselerasjonsdata.

    Args:
        orientation (dict): En ordbok med 'roll' og 'pitch' orienteringsverdier.
        accel_data (dict): En ordbok med akselerometerdata ('x', 'y', 'z').
        gyro_data (dict): En ordbok med gyroskopdata ('x', 'y', 'z').
        threshold_orientering (float): Terskelverdi for orienteringsendring for å vurdere bevegelse.
        threshold_accel_variasjon (float): Terskelverdi for akselerasjonsvariasjon for å vurdere bevegelse.

    Returns:
        str: Den gjenkjente aktiviteten ('Flyting', 'Svømming', 'Svømme nedover', 'Svømme oppover', eller 'Ukjent').
    """

    pitch = orientation['pitch']
    roll = orientation['roll']
    total_accel = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
    
    # Sjekk for flyting
    if abs(pitch) < threshold_orientering and abs(roll) < threshold_orientering and abs(total_accel - 1) < 0.1:
        return 'Flyting'
    
    # Sjekk for svømming
    elif abs(pitch) > threshold_orientering or abs(roll) > threshold_orientering:
        return 'Svømming'
    
    # Sjekk for svømme nedover/oppover (foreløpig enkel sjekk, kan forbedres med mer avansert analyse av akselerasjonsendring)
    elif pitch > 0 and accel_data['z'] < -threshold_accel_variasjon:  # Forenklet sjekk for "nedover"
        return 'Svømme nedover'
    elif pitch < 0 and accel_data['z'] > threshold_accel_variasjon:  # Forenklet sjekk for "oppover"
        return 'Svømme oppover'
    
    else:
        return 'Ukjent'
    

if __name__ == "__main__":
    mpu = MPU6050_Orientation(0x68)

    while True:
        accel_data = mpu.get_accel_data(g=True)
        gyro_data = mpu.get_gyro_data()
        orientation = mpu.get_orientation()

        # Gjenkjenn aktivitet
        aktivitet = gjenkjenn_aktivitet(orientation, accel_data, gyro_data)
        print(f"Aktivitet: {aktivitet}")
        '''
        # Skriv data til CSV-fil (med aktivitet)
        file_path = "sensor_data.csv"
        file_exists = os.path.exists(file_path)

        with open(file_path, "a", newline='') as file:  # Bruk "a" for å legge til data
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(["Timestamp", "Accelerometer x", "Accelerometer y", "Accelerometer z",
                                "Gyroscope x", "Gyroscope y", "Gyroscope z", "Roll", "Pitch", "Aktivitet"])  # Inkluder "Aktivitet"
            writer.writerow([time.time(), accel_data['x'], accel_data['y'], accel_data['z'],
                            gyro_data['x'], gyro_data['y'], gyro_data['z'], orientation['roll'], orientation['pitch'], aktivitet])  # Inkluder aktivitet

'''
        sleep(0.01)