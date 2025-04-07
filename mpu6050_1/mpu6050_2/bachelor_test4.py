from mpu6050 import mpu6050
from time import sleep 
import math
import time

import os
import csv
from collections import deque
import numpy as np
from scipy.signal import butter, lfilter
from collections import deque # For effektiv bufferhåndtering (valgfritt)


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

        # variabler for FFS 
        self.sample_rate = 100  # Hz
        self.window_size = self.sample_rate * 2  # 5 sekunder med data    
        self.data_buffer = deque(maxlen=self.window_size)
        self.raw_data_buffer = deque(maxlen=self.window_size)
        self.last_periodicity_status = "ubestemt"
       

       

        #import smbus2

      

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



    def get_data(self, dt= 0.01):

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
      
        return accel_data, gyro_data
        
       

    
    def butter_highpass(self, cutoff, fs, order=5): # butter høypass filter
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

   
    def highpass_filter(self, data, cutoff, fs, order=5): # kaller butter_highpass filteret og gir 1 returverdi. 
        b, a = self.butter_highpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y
    


    def vurder_stabilitet_G(self, tot_G_values, stabil_grense=1, toleranse=0.1, høy_grense=1.2, stille_grense=0.2):
         # --- Innstillinger ---
        # MPU6050 & Datainnsamling
        sensor_address = 0x68      # Standard I2C adresse
        target_sample_rate_hz = 100 # Ønsket samplingsfrekvens (Hz)

        # Analysevindu og Klassifisering
        window_duration_s = 0.5 # Tid per analysevindu (sekunder)
        window_size = int(target_sample_rate_hz * window_duration_s) # Samples per vindu (50)

        # Faste grenser for klassifisering (basert på MAKS Total G i vinduet)
        # Disse grensene er hentet fra MATLAB-logikken din:
        # Nivå 1 (Lav): Maks G er mellom 0.95 og 1.05
        # Nivå 3 (Høy): Maks G er over 1.65
        # Nivå 2 (Middels): Alle andre tilfeller
        LOW_G_MIN = 0.95
        LOW_G_MAX = 1.05
        HIGH_G_MAX = 1.65 # Grensen for å gå til Høy
        sensor = mpu6050(sensor_address)
        # Gjennomsnittsberegning
        avg_group_size = 10 # Antall klassifiseringer å ta gjennomsnitt av
        current_window_samples = [] # Liste for å samle samples for ett vindu
        classification_history = [] # Liste for å lagre nivå (1, 2, 3)
        try:
            # Les akselerometerdata (som G-krefter)
            accel_data = sensor.get_accel_data(g=True)
            # Beregn Total G
            tot_g = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)

            # Legg til i nåværende vindu-buffer
            current_window_samples.append(tot_g)

            # --- Sjekk om vinduet er fullt ---
            if len(current_window_samples) >= window_size:
                # Ta ut nøyaktig window_size samples for analyse
                window_to_analyze = current_window_samples[:window_size]
                # Fjern de analyserte samples fra starten av bufferet
                current_window_samples = current_window_samples[window_size:]

                # Finn maksimum Total G i vinduet
                max_value_in_window = np.max(window_to_analyze)

                # --- Klassifiser vinduet ---
                level = 0 # Default ukjent
                if max_value_in_window > HIGH_G_MAX:
                    level = 3 # Høy aktivitet
                elif max_value_in_window >= LOW_G_MIN and max_value_in_window <= LOW_G_MAX:
                    level = 1 # Lav aktivitet
                else:
                    level = 2 # Middels aktivitet (verken Høy eller Lav)

                # Lagre klassifiseringen
                classification_history.append(level)
                print(f"Vindu klassifisert: Nivå {level} (Maks G: {max_value_in_window:.3f})")

                # --- Sjekk om gjennomsnitt skal beregnes ---
                if len(classification_history) > 0 and len(classification_history) % avg_group_size == 0:
                    # Ta ut de siste 'avg_group_size' klassifiseringene
                    last_group = classification_history[-avg_group_size:]
                    # Beregn gjennomsnitt og rund av
                    avg_level = np.mean(last_group)
                    rounded_avg_level = round(avg_level)

                    print(f"-----------------------------------------------------")
                    print(f"Gj.snitt siste {avg_group_size} vinduer ({avg_group_size * window_duration_s:.1f}s): {avg_level:.2f}")
                    print(f"Avrundet nivå: {rounded_avg_level} (1=Lav, 2=Middels, 3=Høy)")
                    print(f"-----------------------------------------------------")


        except Exception as e:
            print(f"Feil under lesing/prosessering: {e}")
            # Vurder å nullstille current_window_samples her hvis feil skjer?
            # current_window_samples = []
            time.sleep(0.1) # Kort pause ved feil

    
    
    def vurder_stabilitet_Gyro(self, tot_G_values, stabil_grense=1, toleranse=0.1, høy_grense=1.2, stille_grense=0.2):
        """
        Funksjon som vurderer stabilitet basert på gjennomsnitt av tot_G i korte perioder.
        
        :param tot_G_values: Liste eller array med tot_G (total akselerasjon)
        :return: String "Stabil", "Høy", eller "Stille"
        """
        # Beregn gjennomsnittet av de siste tot_G-verdiene (tidsvindu på 0.5 sekunder)
        mean_tot_Gyro = np.mean(tot_G_values)
        
        # Vurder stabilitet basert på tot_G og grensene
        if mean_tot_Gyro > høy_grense:
            return "Høy"  # Høy aktivitet (f.eks., intens akselerasjon)
        elif abs(mean_tot_Gyro - stabil_grense) <= toleranse:
            return "Stabil"  # Stabilt (normal aktivitet rundt stabil_grense)
        elif mean_tot_Gyro < stille_grense:
            return "Stille"  # Stille (ro eller liten bevegelse)
        else:
            return "Ustabil"  # Ustabilt (signifikant avvik fra stabil grense)
    



    def gi_status_aks(self):
        # Hent akselerasjonsdata
        data = self.get_data()
        accel_data = data[0]
        gyro_data = data[1]
        tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
        tot_Gyro = math.sqrt(gyro_data['x']**2 + gyro_data['y']**2 + gyro_data['z']**2)
        # Legg til den nyeste målingen i en buffer for filtrering
        self.raw_data_buffer.append(tot_G)

        # Når bufferen har nok data, filtrer og vurder periodisiteten
        if len(self.raw_data_buffer) == self.window_size:
            # Filtrer dataene
            filtered_data = self.raw_data_buffer #self.highpass_filter(list(self.raw_data_buffer), cutoff=0.1, fs=self.sample_rate, order=5)
            self.data_buffer.extend(filtered_data)

            # Vurder periodisiteten basert på de filtrerte dataene
            is_periodic = self.vurder_stabilitet_G(list(self.data_buffer))
            self.last_periodicity_status = is_periodic

            # Tøm bufferen for neste vindu
            self.raw_data_buffer.clear()
            self.data_buffer.clear()

        # Returner resultatene
        return {
            'total_G': tot_G,
            'is_periodic': self.last_periodicity_status
        }

        

#vi trenger ikkje denne da vi har en egen funksjon for å hente data

if __name__ == "__main__":
    #sensor = mpu6050(0x68)
    mpu = MPU6050_Orientation(0x68)
    
    while True:
        status = mpu.gi_status_aks()
        print(f"Total G: {status['total_G']}")
        print(status['is_periodic'])

        
        file_path = "sensor_data_tot_G.csv"
        file_exists = os.path.exists(file_path)
        
        with open(file_path, "a", newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                    writer.writerow(["G"])
            writer.writerow([status['total_G']])
        

        time.sleep(0.01)

    