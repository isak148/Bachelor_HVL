from mpu6050 import mpu6050
from time import sleep 
import math
import time

import os
import csv
from collections import deque
import numpy as np
from scipy.signal import butter, lfilter
from Kalman.kalman_filter import KalmanFilter

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

        self.kalman_x = KalmanFilter()
        self.kalman_y = KalmanFilter()


    def calibrate_accel(self, samples=100): # Økt antall samples
        print("Starter kalibrering av akselerometer.")
        print("VIKTIG: Plasser sensoren HELT I RO og HORISONTALT (flatt)!")
        sleep(2) # Gi brukeren tid til å plassere sensoren

        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        sum_sq = {'x': 0.0, 'y': 0.0, 'z': 0.0} # For standardavvik

        for i in range(samples):
            try:
                accel_data = self.get_accel_data(g=True)
                offset['x'] += accel_data['x']
                offset['y'] += accel_data['y']
                offset['z'] += accel_data['z']
                sum_sq['x'] += accel_data['x']**2
                sum_sq['y'] += accel_data['y']**2
                sum_sq['z'] += accel_data['z']**2
                sleep(0.02) # Kortere sleep, flere samples
            except IOError:
                 print(f"IOError under aksel.kalibrering sample {i+1}/{samples}. Prøver igjen...")
                 sleep(0.1) # Vent litt før nytt forsøk
                 # Vurder å hoppe over/prøve på nytt her
                 # For enkelhets skyld, la oss bare fortsette, men dette kan påvirke snittet


        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples

        # Beregn standardavvik for å sjekke stabilitet
        std_dev_x = math.sqrt(max(0, sum_sq['x'] / samples - offset['x']**2)) # max(0,...) for numerisk stabilitet
        std_dev_y = math.sqrt(max(0, sum_sq['y'] / samples - offset['y']**2))
        std_dev_z = math.sqrt(max(0, sum_sq['z'] / samples - offset['z']**2))

        print(f"Aksel. Kalibrering Std Dev: x={std_dev_x:.4f}, y={std_dev_y:.4f}, z={std_dev_z:.4f}")
        # Sett en fornuftig terskel for stabilitet (f.eks. 0.05g)
        if max(std_dev_x, std_dev_y, std_dev_z) > 0.05:
            print("ADVARSEL: Akselerometeret var ustabilt under kalibrering! Resultatet kan være upålitelig.")

        # --- Forbedret Offset Beregning ---
        # Anta at sensoren var noenlunde flat, X og Y skal være nær 0g
        accel_offset_cal = {'x': offset['x'], 'y': offset['y']}

        # For Z, finn ut om den er nærmest +1g eller -1g
        g_magnitude_z = abs(offset['z'])
        if g_magnitude_z < 0.5: # Bør være nær 1g hvis den ligger flatt
             print(f"ADVARSEL: Z-aksen ({offset['z']:.2f}g) er ikke nær +/-1g. Er sensoren plassert flatt?")
             # Bruk den målte verdien som offset hvis vi ikke er sikre på orienteringen
             accel_offset_cal['z'] = offset['z']
        else:
            # Bestem om det er +1g eller -1g
            expected_g_z = np.sign(offset['z']) # Gir +1.0 eller -1.0
            # Offset er forskjellen mellom målt verdi og forventet G-verdi
            accel_offset_cal['z'] = offset['z'] - expected_g_z

        print(f"Ferdig kalibrert akselerometer. Offset: x={accel_offset_cal['x']:.3f}, y={accel_offset_cal['y']:.3f}, z={accel_offset_cal['z']:.3f}")
        return accel_offset_cal

    def calibrate_gyro(self, samples=100): # Økt antall samples
        print("Starter kalibrering av gyroskop.")
        print("VIKTIG: Plasser sensoren HELT I RO!")
        sleep(2) # Gi brukeren tid

        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        sum_sq = {'x': 0.0, 'y': 0.0, 'z': 0.0} # For standardavvik

        for i in range(samples):
            try:
                gyro_data = self.get_gyro_data()
                offset['x'] += gyro_data['x']
                offset['y'] += gyro_data['y']
                offset['z'] += gyro_data['z']
                sum_sq['x'] += gyro_data['x']**2
                sum_sq['y'] += gyro_data['y']**2
                sum_sq['z'] += gyro_data['z']**2
                sleep(0.02) # Kortere sleep
            except IOError:
                 print(f"IOError under gyro.kalibrering sample {i+1}/{samples}. Prøver igjen...")
                 sleep(0.1)

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples

         # Beregn standardavvik for å sjekke stabilitet
        std_dev_x = math.sqrt(max(0, sum_sq['x'] / samples - offset['x']**2))
        std_dev_y = math.sqrt(max(0, sum_sq['y'] / samples - offset['y']**2))
        std_dev_z = math.sqrt(max(0, sum_sq['z'] / samples - offset['z']**2))

        print(f"Gyro Kalibrering Std Dev: x={std_dev_x:.4f}, y={std_dev_y:.4f}, z={std_dev_z:.4f}")
        # Sett en fornuftig terskel (f.eks. 0.5 deg/s)
        if max(std_dev_x, std_dev_y, std_dev_z) > 0.5:
            print("ADVARSEL: Gyroskopet var ustabilt under kalibrering! Resultatet kan være upålitelig.")


        print(f"Ferdig kalibrert gyro. Offset: x={offset['x']:.3f}, y={offset['y']:.3f}, z={offset['z']:.3f}")
        return offset


    def get_orientation(self, dt= 0.01):

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        #Beregner vinkelorientering basert på akselerometer og gyroskop
        accel_data = self.get_accel_data(g=True)
        gyro_data = self.get_gyro_data()
        
        #Juster for kalibrert bias
        gyro_data['x'] -= self.gyro_offset['x']
        gyro_data['y'] -= self.gyro_offset['y']
        gyro_data['z'] -= self.gyro_offset['z']

        accel_data['x'] -= self.accel_offset['x']
        accel_data['y'] -= self.accel_offset['y']
        accel_data['z'] -= self.accel_offset['z'] 
      
        
        # Beregn vinkel fra akselerometer i grader
        accel_angle_x = math.atan2(accel_data['y'], accel_data['x']) * (180/math.pi)
        accel_angle_y = math.atan2(-accel_data['x'], accel_data['z']) * (180/math.pi)
        
        
        
       
        #sjekker om en av aksene er stabil på en 1G
        if abs(accel_data['x']) > (1.0 - self.threshold) and abs(accel_data['x']) < (1.0 + self.threshold):
            self.stable_count_x += 1
        else:
            self.stable_count_x = 0

        if abs(accel_data['y']) > (1.0 - self.threshold) and abs(accel_data['y']) < (1.0 + self.threshold):
            self.stable_count_y += 1
        else:
            self.stable_count_y
        
        if abs(accel_data['z']) > (1.0 - self.threshold) and abs(accel_data['z']) < (1.0 + self.threshold):
            self.stable_count_z += 1
        else:
            self.stable_count_z = 0
        
        if self.stable_count_z >= self.required_stable_count:
            self.gyro_angle_x = 0.0 #Nullstil roll
            self.gyro_angle_y = 0.0 #Nullstill pitch 
            if accel_data['z'] > 0: # Hvis +1 g på z-aksen
                self.roll_offset = 0.0
                self.pitch_offset = 0.0
            else: #Hvis -1g på z aksen 
                self.roll_offset = 180.0 # sensoren er opp-ned 
                self.pitch_offset = 180.0
                self.gyro_angle_x = accel_angle_x +180
                self.gyro_angle_y = accel_angle_y +180                   
            self.stable_count_z = 0 # Resetter teller
        elif self.stable_count_y >= self.required_stable_count:
            self.gyro_angle_x = accel_angle_x # Nullstill akkumulert roll 
            if accel_data['y'] > 0: # hvis +1g på Y-aksen
                self.roll_offset = 90.0
                self.pitch_offset = 0.0
            else: # -1G på y-aksen 
                self.roll_offset = -90.0
                self.pitch_offset = 0.0
            self.stable_count_y = 0 # Resetter teller
        elif self.stable_count_x >= self.required_stable_count:
            self.gyro_angle_y = accel_angle_y # Nullstill akkumulert pitch 
            if accel_data['x'] < 0: # hvis +1g på Y-aksen 
                self.pitch_offset = -90.0
                self.roll_offset = 0.0
            else: #Hvis -1g på x-aksen
                self.pitch_offset = 90.0
                self.roll_offset = 0.0
            self.stable_count_x = 0 # Resetter teller 
        else:
            pass 


        # Integrer gyro-data for å beregne vinkelendring
        self.gyro_angle_x += gyro_data['x'] * dt
        self.gyro_angle_y += gyro_data['y'] * dt

        # Bruk Kalman-filter for å kombinere akselerometer og gyro
        angle_x = self.kalman_x.update(accel_angle_x - self.roll_offset, gyro_data['x'], dt)
        angle_y = self.kalman_y.update(accel_angle_y - self.pitch_offset, gyro_data['y'], dt)

        return {'roll': angle_x, 'pitch': angle_y}




    
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