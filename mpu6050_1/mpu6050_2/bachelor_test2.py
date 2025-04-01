from mpu6050 import mpu6050
from time import sleep 
import math
import time

import os
import csv
from collections import deque
import numpy as np
from scipy.signal import butter, lfilter
from Kalman.Kalman_rotasjon_matrise import KalmanFilterWithRotation
from Kalman.kalman_Quaternion import KalmanFilterWithQuaternion
#import smbus2

class MPU6050_Orientation(mpu6050):
    def __init__(self, address, bus=1):
        super(). __init__(address, bus)

        #Justere følsomhet skala(grader/s) for gyro
        self.set_gyro_range(self.GYRO_RANGE_250DEG)
        #Juster akselrometer følsomhet 
        self.set_accel_range(self.ACCEL_RANGE_2G)
        #Velg filtrerng for mpu mer filtrering treigere respons 
        self.set_filter_range(self.FILTER_BW_188)

        self.gyro_offset = self.calibrate_gyro(50)
        self.accel_offset = self.calibrate_accel(50)

        self.gyro_angle_x = 0.0
        self.gyro_angle_y = 0.0
        self.alpha = 0.95 #Komplementær filtervekt
        self.last_time = time.time()

        # variabler for automatiskkalibrering av roll pitch 
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        #variabler for å telle stabile målinger
        self.stable_count_x = 0
        self.stable_count_y = 0
        self.stable_count_z = 0
        self.required_stable_count = 20
        self.threshold = 0.05

        # variabler for FFS 
        self.sample_rate = 100  # Hz
        self.window_size = self.sample_rate * 5  # 5 sekunder med data    
        self.data_buffer = deque(maxlen=self.window_size)
        self.raw_data_buffer = deque(maxlen=self.window_size)
        self.last_periodicity_status = None
        
        #Initialiserer kalman  filter
        self.kalman_filter = KalmanFilterWithQuaternion()






    def calibrate_accel(self, samples=100):
        print("starter kalibrering accelrometer")
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        for _ in range(samples):
            accel_data = self.get_accel_data(g=True)
            offset['x'] += accel_data['x']
            offset['y'] += accel_data['y']
            offset['z'] += accel_data['z']
            sleep(0.05)
        
        print("Ferdig kalibrert accelrometer")

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] = offset['z'] / samples - 1.0 # juster for gravitasjon 
        
        return offset  

    def calibrate_gyro(self, samples=100):
        print("starter kalibrering gyro")
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        for _ in range(samples):
            gyro_data = self.get_gyro_data() 
            offset['x'] += gyro_data['x']
            offset['y'] += gyro_data['y']
            offset['z'] += gyro_data['z']
            sleep(0.05)
       
        print("Ferdig kalibrert gyro")

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples
        
        return offset  



    def get_orientation(self):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 1 / self.sample_rate
        self.last_time = current_time

        # --- Hent og kalibrer rådata ---
        accel_data_raw = self.get_accel_data(g=True)
        gyro_data_raw = self.get_gyro_data()

        accel_data_cal = {
            'x': accel_data_raw['x'] - self.accel_offset['x'],
            'y': accel_data_raw['y'] - self.accel_offset['y'],
            'z': accel_data_raw['z'] - self.accel_offset['z']
        }

        gyro_data_cal = {
            'x': gyro_data_raw['x'] - self.gyro_offset['x'],
            'y': gyro_data_raw['y'] - self.gyro_offset['y'],
            'z': gyro_data_raw['z'] - self.gyro_offset['z']
        }

        # --- Bruk Kalman med rotasjonsmatrise ---
        
        orientation = self.kalman_filter.update(gyro_data_cal, accel_data_cal)

        return orientation  # {'roll': ..., 'pitch': ...}


    
    def butter_highpass(self, cutoff, fs, order=5): # butter høypass filter
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

   
    def highpass_filter(self, data, cutoff, fs, order=5): # kaller butter_highpass filteret og gir 1 returverdi. 
        b, a = self.butter_highpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y
    

    def is_periodic(self, signal, threshold_ratio=0.1, min_significant_freqs=1):
        """Vurderer om et signal inneholder et periodisk mønster basert på FFT-analyse."""
        N = len(signal)
        fft_values = np.fft.rfft(signal)
        fft_magnitude = np.abs(fft_values) / N
        threshold = threshold_ratio * np.max(fft_magnitude)
        significant_freqs = np.sum(fft_magnitude > threshold)
        return significant_freqs >= min_significant_freqs


    def gi_status_aks(self):
        # Hent akselerasjonsdata
        accel_data = self.get_accel_data(g=True)
        tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)

        # Legg til den nyeste målingen i en buffer for filtrering
        self.raw_data_buffer.append(tot_G)

        # Når bufferen har nok data, filtrer og vurder periodisiteten
        if len(self.raw_data_buffer) == self.window_size:
            # Filtrer dataene
            filtered_data = self.highpass_filter(list(self.raw_data_buffer), cutoff=0.5, fs=self.sample_rate, order=5)
            self.data_buffer.extend(filtered_data)

            # Vurder periodisiteten basert på de filtrerte dataene
            is_periodic = self.is_periodic(list(self.data_buffer))
            self.last_periodicity_status = is_periodic

            # Tøm bufferen for neste vindu
            self.raw_data_buffer.clear()
            self.data_buffer.clear()

        # Hent orienteringsdata
        orientation = self.get_orientation()
        roll = orientation['roll']
        pitch = orientation['pitch']

        # Returner resultatene
        return {
            'total_G': tot_G,
            'roll': roll,
            'pitch': pitch,
            'is_periodic': self.last_periodicity_status
        }

        

#vi trenger ikkje denne da vi har en egen funksjon for å hente data

if __name__ == "__main__":
    #sensor = mpu6050(0x68)
    mpu = MPU6050_Orientation(0x68)
    
    while True:
        status = mpu.gi_status_aks()
        print(f"Total G: {status['total_G']}")
        print(f"Roll: {status['roll']}")
        print(f"Pitch: {status['pitch']}")
        print(f"Periodisitet: {'Jevn' if status['is_periodic'] else 'Ujevn'}")

    
   
    
    
''' 
    

    #sensor = mpu6050(0x68)
    mpu = MPU6050_Orientation(0x68)
    



    while True:
        accel_data = mpu.get_accel_data(g=True)
        gyro_data = mpu.get_gyro_data()
        #temp = sensor.get_temp()
        orientation = mpu.get_orientation()

        print("Accelerometer data")
        print("x: " + str(accel_data['x']-mpu.accel_offset['x']))
        print("y: " + str(accel_data['y']-mpu.accel_offset['y']))
        print("z: " + str(accel_data['z']-mpu.accel_offset['z']))

        print("Gyroscope data")
        print("x: " + str(gyro_data['x']- mpu.gyro_offset['x']))
        print("y: " + str(gyro_data['y']- mpu.gyro_offset['y']))
        print("z: " + str(gyro_data['z']- mpu.gyro_offset['z']))

        #print("Temp: " + str(temp) + " C")
    
    
        print(f"Roll: {orientation['roll']:.2f}, Pitch: {orientation['pitch']:.2f}")

        if abs(orientation['pitch']) > 45.0:
            print("svømmer")
        else: 
            print("flyter")

        file_path = "sensor_data.csv"
        file_exists = os.path.exists(file_path)

        with open(file_path, "w", newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(["Accelerometer x", "Accelerometer y", "Accelerometer z", 
                                "Gyroscope x", "Gyroscope y", "Gyroscope z", "role", "pitch"])

            writer.writerow([accel_data['x'], accel_data['y'], accel_data['z'], 
                            gyro_data['x'], gyro_data['y'], gyro_data['z'],orientation['roll'],orientation['pitch']])
        sleep(0.01)
'''