from mpu6050_1.mpu6050_2.mpu6050 import mpu6050
from time import sleep
import math
import time

import os
import csv
from collections import deque
import numpy as np
from scipy.signal import butter, lfilter

#import smbus2

class MPU6050_Orientation(mpu6050):
    def __init__(self, address, bus=1):
        super().__init__(address, bus)

        # Sensor-innstillinger
        self.set_gyro_range(self.GYRO_RANGE_250DEG) # +/- 250 dps
        self.set_accel_range(self.ACCEL_RANGE_2G)    # +/- 2g
        self.set_filter_range(self.FILTER_BW_188)    # Digital Low Pass Filter

        # Kalibrering
        #print("Starter kalibrering...")
        self.gyro_offset = self.calibrate_gyro(100)
        self.accel_offset = self.calibrate_accel(100)
        #print("Kalibrering fullført.")

        # Samme vindusstørrelse brukes for både akselerometer og gyro analyse
        self.window_size = 50 # 0.5 sekunds vindu (50 samples)

        # Buffere for rådata (magnitude)
        self.raw_accel_buffer = deque(maxlen=self.window_size)
        self.raw_gyro_buffer = deque(maxlen=self.window_size)

        # Variabler for Akselerometer (G) stabilitetsvurdering
        self.accel_stabilitet_historikk = []
        self.last_computed_accel_status = "Initialiserer..."

        # Variabler for Gyro stabilitetsvurdering (NY)
        self.gyro_stabilitet_historikk = []
        self.last_computed_gyro_status = "Initialiserer..."

    # --- Kalibreringsfunksjoner ---
    def calibrate_accel(self, samples=100):
        print(f"  Kalibrerer akselerometer ({samples} samples)...", end='')
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        for _ in range(samples):
            try:
                accel_data = self.get_accel_data(g=True)
                offset['x'] += accel_data['x']
                offset['y'] += accel_data['y']
                offset['z'] += accel_data['z']
                sleep(0.02)
            except Exception as e:
                print(f"\nFeil under aksel-kalibrering: {e}")
                sleep(0.1) # Vent litt før neste forsøk
        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] = (offset['z'] / samples) - 1.0 # Juster for gravitasjon
        print(f" Ferdig. Offset: x={offset['x']:.3f}, y={offset['y']:.3f}, z={offset['z']:.3f}")
        return offset

    def calibrate_gyro(self, samples=100):
        print(f"  Kalibrerer gyroskop ({samples} samples)...", end='')
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        for _ in range(samples):
            try:
                gyro_data = self.get_gyro_data()
                offset['x'] += gyro_data['x']
                offset['y'] += gyro_data['y']
                offset['z'] += gyro_data['z']
                sleep(0.02)
            except Exception as e:
                 print(f"\nFeil under gyro-kalibrering: {e}")
                 sleep(0.1) # Vent litt før neste forsøk
        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples
        print(f" Ferdig. Offset: x={offset['x']:.3f}, y={offset['y']:.3f}, z={offset['z']:.3f}")
        return offset

    # --- Hent data (uendret) ---
    def get_data(self):
        
        try:
            accel_data_raw = self.get_accel_data(g=True)
            gyro_data_raw = self.get_gyro_data()
        except Exception as e:
            print(f"Feil ved lesing fra MPU6050: {e}")
            return {'x': 0, 'y': 0, 'z': 1}, {'x': 0, 'y': 0, 'z': 0} # Return dummy
        accel_data = accel_data_raw.copy()
        gyro_data = gyro_data_raw.copy()
        accel_data['x'] -= self.accel_offset['x']
        accel_data['y'] -= self.accel_offset['y']
        accel_data['z'] -= self.accel_offset['z']
        gyro_data['x'] -= self.gyro_offset['x']
        gyro_data['y'] -= self.gyro_offset['y']
        gyro_data['z'] -= self.gyro_offset['z']
        return accel_data, gyro_data
    
    '''
    # --- Filterfunksjoner (uendret, ikke i bruk) ---
    def butter_highpass(self, cutoff, fs, order=5):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

    def highpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_highpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y
    '''
    # --- Vurderingsfunksjoner ---
    def vurder_stabilitet_G(self, tot_G_values, ro_grense=1.0, høy_aktivitet_øvre_grense=1.65,høy_aktivitet_nedre_grense=0.65 , toleranse=0.05):
            """
            Vurderer akselerometer-aktivitet basert på terskler INNENFOR vinduet,
            UTEN å beregne gjennomsnitt av tot_G_values.
            Klassifiserer vinduet basert på om noen verdier er høye, eller om alle er lave.
            Den videre logikken med å samle 10 nivåer og ta snittet av DEM er beholdt.
            """
            if not tot_G_values: return self.last_computed_accel_status

            # Konverter til numpy array for enklere testing
            g_array = np.array(tot_G_values)

            # Definer grenser for klassifisering
            lav_øvre_grense = ro_grense + toleranse
            lav_nedre_grense = ro_grense - toleranse
            
            nivå = 0
            # 1. Sjekk for Høy aktivitet (Nivå 3): Holder det at MINST ÉN verdi er over grensen?
            if np.any((g_array > høy_aktivitet_øvre_grense)|(g_array < høy_aktivitet_nedre_grense)):
                nivå = 3 # Høy
            # 2. Sjekk for Lav aktivitet (Nivå 1): Må ALLE verdier være innenfor ro-intervallet?
            elif np.all((g_array >= lav_nedre_grense) & (g_array <= lav_øvre_grense)):
                nivå = 1 # Lav (I ro)
            # 3. Ellers er det Middels aktivitet (Nivå 2)
            else:
                nivå = 2 # Middels

            # --- Resten av logikken er uendret: legg til nivå i historikk, sjekk om full, beregn snitt av nivåer ---
            self.accel_stabilitet_historikk.append(nivå)

            if len(self.accel_stabilitet_historikk) >= 10:
                snitt_nivå = np.mean(self.accel_stabilitet_historikk)
                rundet_snitt = int(round(snitt_nivå))
                if rundet_snitt == 1: self.last_computed_accel_status = "Stille"
                elif rundet_snitt == 2: self.last_computed_accel_status = "Moderat"
                elif rundet_snitt == 3: self.last_computed_accel_status = "Høy"
                else: self.last_computed_accel_status = f"Ukjent G ({rundet_snitt})"
                self.accel_stabilitet_historikk.clear()

            return self.last_computed_accel_status

    # Modifisert implementasjon for Gyro uten snitt av rådata
    def vurder_stabilitet_Gyro(self, tot_Gyro_values, stille_terskel=5.0, hoy_terskel=50.0):
        """
        Vurderer rotasjonsaktivitet basert på terskler INNENFOR vinduet,
        UTEN å beregne gjennomsnitt av tot_Gyro_values.
        Klassifiserer vinduet basert på om noen verdier er høye, eller om alle er lave.
        Den videre logikken med å samle 10 nivåer og ta snittet av DEM er beholdt.
        Terskler er i grader per sekund (dps).
        """
        if not tot_Gyro_values: return self.last_computed_gyro_status

        # Konverter til numpy array
        gyro_array = np.array(tot_Gyro_values)

        nivå = 0
        # 1. Sjekk for Høy bevegelse (Nivå 3): MINST ÉN verdi over høy terskel?
        if np.any(gyro_array > hoy_terskel):
            nivå = 3 # Høy bevegelse
        # 2. Sjekk for Stille (Nivå 1): ALLE verdier under/lik stille terskel?
        elif np.all(gyro_array <= stille_terskel):
            nivå = 1 # Stille
        # 3. Ellers er det Moderat bevegelse (Nivå 2)
        else:
            nivå = 2 # Moderat bevegelse

        # --- Resten av logikken er uendret: legg til nivå i historikk, sjekk om full, beregn snitt av nivåer ---
        self.gyro_stabilitet_historikk.append(nivå)

        if len(self.gyro_stabilitet_historikk) >= 10:
            snitt_nivå = np.mean(self.gyro_stabilitet_historikk)
            rundet_snitt = int(round(snitt_nivå))
            if rundet_snitt == 1: self.last_computed_gyro_status = "Stille"
            elif rundet_snitt == 2: self.last_computed_gyro_status = "Moderat"
            elif rundet_snitt == 3: self.last_computed_gyro_status = "Høy"
            else: self.last_computed_gyro_status = f"Ukjent Gyro ({rundet_snitt})"
            self.gyro_stabilitet_historikk.clear()

        return self.last_computed_gyro_status

    # --- oppdater_og_vurder_status (uendret fra forrige versjon) ---
    def oppdater_og_vurder_status(self, Lagre = False, Filnavn = "" ):
        """
        Henter nye data, beregner magnituder, legger i buffere,
        og kaller vurderingsfunksjoner når bufferne er fulle.
        Returnerer øyeblikkelige verdier og de sist beregnede statusene.
        """
        
        accel_data, gyro_data = self.get_data()
        tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
        tot_Gyro = math.sqrt(gyro_data['x']**2 + gyro_data['y']**2 + gyro_data['z']**2)

        self.raw_accel_buffer.append(tot_G)
        self.raw_gyro_buffer.append(tot_Gyro)

        status_fra_G = self.last_computed_accel_status
        status_fra_Gyro = self.last_computed_gyro_status

        if len(self.raw_accel_buffer) == self.window_size:
            # Kall de (nå modifiserte) vurderingsfunksjonene
            status_fra_G = self.vurder_stabilitet_G(list(self.raw_accel_buffer))
            status_fra_Gyro = self.vurder_stabilitet_Gyro(list(self.raw_gyro_buffer))

            self.raw_accel_buffer.clear()
            self.raw_gyro_buffer.clear()

        if accel_data['z'] <=-0.25:
            retning = ("Ned")
        elif accel_data['z']>=0.1:
            retning = ("Opp")
        else:
            retning = ("Plan")

        if(Lagre == True):
            self.skriv_til_fil(Filnavn, tot_G, tot_Gyro, accel_data['z'], retning)


        return {
            'Total_G': tot_G,   
            'Total_Gyro': tot_Gyro,
            'Aks_Status': status_fra_G,
            'Gyro_Status': status_fra_Gyro,
            'Retning': retning
        }
    
    
    def skriv_til_fil(self, filnavn, verdi1, verdi2, verdi3, verdi4):
        if not filnavn.endswith(".csv"):
            filnavn += ".csv"  # Legger til .csv hvis det mangler

        with open(filnavn, mode='a', newline='', encoding='utf-8') as fil:
            writer = csv.writer(fil)
            writer.writerow([verdi1, verdi2, verdi3, verdi4])  # Skriver begge verdiene på én linje

# --- Hovedprogram (uendret fra forrige versjon) ---
if __name__ == "__main__":
    try:
        mpu = MPU6050_Orientation(0x68)
        print("\nMPU6050 initialisert og klar.")
        print("Starter datainnsamling og statusvurdering (uten snitt av rådata i vindu)...") # Oppdatert print

        output_interval = 1.0
        last_print_time = time.time()

        file_path = "sensor_status_log_no_avg.csv" # Endret filnavn
        file_exists = os.path.exists(file_path)
        print(f"Logger data til: {file_path}")
        '''
        with open(file_path, "a", newline='') as file:
            writer = csv.writer(file)
            if not file_exists or os.path.getsize(file_path) == 0:
                writer.writerow([
                    "Timestamp", "Total_G", "Aks_Status",
                    "Total_Gyro_DPS", "Gyro_Status"
                ])
        '''
        while True:
            current_loop_time = time.time()
            status_data = mpu.oppdater_og_vurder_status()
            '''
            writer.writerow([
                current_loop_time, status_data['total_G'], status_data['aks_status'],
                status_data['total_Gyro'], status_data['gyro_status']
            ])
            file.flush()
            '''
            if current_loop_time - last_print_time >= output_interval:
                print(f"Tid: {current_loop_time:.1f} | "
                        f"G: {status_data['Total_G']:.2f}g [{status_data['Aks_Status']}] | "
                        f"Gyro: {status_data['Total_Gyro']:.1f}dps [{status_data['Gyro_Status']}]")
                last_print_time = current_loop_time

            sleep(0.01)

    except KeyboardInterrupt:
        print("\nAvslutter programmet.")
    except Exception as e:
        print(f"\nEn uventet feil oppstod: {e}")
