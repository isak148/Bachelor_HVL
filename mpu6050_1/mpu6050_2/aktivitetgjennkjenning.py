from mpu6050 import mpu6050
from time import sleep 
import math
import time
import numpy as np
from scipy.fft import fft
import os
import csv

#import smbus2

class MPU6050_Orientation(mpu6050):
    def __init__(self, address, bus=1, window_size=32):  # Legg til window_size
        super().__init__(address, bus)

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

        self.window_size = window_size
        self.accel_z_buffer # Buffer for z-akselerasjon for analyse
        self.time_buffer # Buffer for tidsstempel
        self.sample_rate = 100  # Hz (tilnærmet)





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

        #Integrer gyro-data for å beregne vinkelendring
        self.gyro_angle_x += gyro_data['x'] * dt
        self.gyro_angle_y += gyro_data['y'] * dt

        #bruk komplementært filter: kombiner gyro og akselerometer
        angle_x =self.alpha * self.gyro_angle_x + (1- self.alpha) * (accel_angle_x - self.roll_offset)
        angle_y =self.alpha * self.gyro_angle_y + (1- self.alpha) * (accel_angle_y - self.pitch_offset)


        return {'roll': angle_x, 'pitch': angle_y}

    def gi_status_aks():

        mpu = MPU6050_Orientation(0x68)
        accel_data = mpu.get_accel_data(g=True)
        tot_G = accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2
        tot_G = math.sqrt(tot_G)
        
        orientation = mpu.get_orientation()
        roll = orientation['roll']
        pitch = orientation['pitch']


        
        

    def get_accel_z_change(self):
            """Beregn endring i akselerasjon i z-retningen."""
            if len(self.accel_z_buffer) < 2:
                return 0  # Ikke nok data
            return (self.accel_z_buffer[-1] - self.accel_z_buffer[-2]) / (self.time_buffer[-1] - self.time_buffer[-2])

    def get_dominant_frequency(self, data):
        """Finn den dominerende frekvensen i dataene."""
        if len(data) < self.window_size:
            return 0  # Ikke nok data
        
        # Filtrer ut NaN-verdier
        data = np.nan_to_num(data)
        
        # Utfør FFT
        fft_data = fft(data)
        
        # Finn den dominerende frekvensen
        frequencies = np.fft.fftfreq(len(data), 1/self.sample_rate)
        dominant_frequency_index = np.argmax(np.abs(fft_data[1:len(data)//2])) + 1  # Ignorer DC-komponenten
        dominant_frequency = frequencies[dominant_frequency_index]
        return abs(dominant_frequency)

def gjenkjenn_aktivitet(orientation, accel_data, gyro_data, accel_z_change, dominant_frequency,
                       threshold_orientering=20.0, threshold_accel_variasjon=0.2,
                       threshold_dominant_frequency=0.5):
    """
    Gjenkjenner svømmeaktivitet basert på orientering, akselerasjonsdata, og frekvensanalyse.

    Args:
        orientation (dict): En ordbok med 'roll' og 'pitch' orienteringsverdier.
        accel_data (dict): En ordbok med akselerometerdata ('x', 'y', 'z').
        gyro_data (dict): En ordbok med gyroskopdata ('x', 'y', 'z').
        accel_z_change (float): Endring i akselerasjon i z-retningen.
        dominant_frequency (float): Den dominerende frekvensen i bevegelsen.
        threshold_orientering (float): Terskelverdi for orienteringsendring for å vurdere bevegelse.
        threshold_accel_variasjon (float): Terskelverdi for akselerasjonsvariasjon for å vurdere bevegelse.
        threshold_dominant_frequency (float): Terskelverdi for dominerende frekvens.

    Returns:
        str: Den gjenkjente aktiviteten ('Flyting', 'Svømming', 'Svømme nedover', 'Svømme oppover', eller 'Ukjent').
    """

    pitch = orientation['pitch']
    roll = orientation['roll']
    total_accel = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)

    # Sjekk for flyting
    if abs(pitch) < threshold_orientering and abs(roll) < threshold_orientering and abs(total_accel - 1) < 0.1:
        return 'Flyting'

    # Sjekk for svømming (inkluderer frekvensanalyse)
    elif (abs(pitch) > threshold_orientering or abs(roll) > threshold_orientering) and dominant_frequency > threshold_dominant_frequency:
        return 'Svømming'

    # Sjekk for svømme nedover/oppover (inkluderer akselerasjonsendring)
    elif pitch > 0 and accel_z_change < -threshold_accel_variasjon:  # Forenklet sjekk for "nedover"
        return 'Svømme nedover'
    elif pitch < 0 and accel_z_change > threshold_accel_variasjon:  # Forenklet sjekk for "oppover"
        return 'Svømme oppover'

    else:
        return 'Ukjent'

if __name__ == "__main__":
    mpu = MPU6050_Orientation(0x68)

    while True:
        accel_data = mpu.get_accel_data(g=True)
        gyro_data = mpu.get_gyro_data()
        orientation = mpu.get_orientation()
        timestamp = time.time()

        # Oppdater buffere for frekvensanalyse og akselerasjonsendring
        mpu.accel_z_buffer.append(accel_data['z'])
        mpu.time_buffer.append(timestamp)

        if len(mpu.accel_z_buffer) > mpu.window_size:
            mpu.accel_z_buffer.pop(0)
            mpu.time_buffer.pop(0)

        # Beregn akselerasjonsendring og dominerende frekvens
        accel_z_change = mpu.get_accel_z_change()
        dominant_frequency = mpu.get_dominant_frequency(mpu.accel_z_buffer)

        # Gjenkjenn aktivitet
        aktivitet = gjenkjenn_aktivitet(orientation, accel_data, gyro_data, accel_z_change, dominant_frequency)
        print(f"Aktivitet: {aktivitet}, Roll: {orientation['roll']:.2f}, Pitch: {orientation['pitch']:.2f}, Dominant Freq: {dominant_frequency:.2f}, Accel Z Change: {accel_z_change:.2f}")

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
