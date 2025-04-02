from mpu6050 import mpu6050
from time import sleep
import math
import time
import os
import csv
from collections import deque
import numpy as np
from scipy.signal import butter, lfilter
# import smbus2 # Kommentert ut, ser ikke ut til å være i bruk

class MPU6050_Orientation(mpu6050):
    def __init__(self, address, bus=1):
        super().__init__(address, bus)

        # Justere følsomhet skala(grader/s) for gyro
        self.set_gyro_range(self.GYRO_RANGE_250DEG)
        # Juster akselerometer følsomhet
        self.set_accel_range(self.ACCEL_RANGE_2G)
        # Velg filtrering for mpu mer filtrering treigere respons
        self.set_filter_range(self.FILTER_BW_188) # Vurder om dette filteret påvirker FFT

        # self.gyro_offset = self.calibrate_gyro(50) # Kalibrering er bra, men ikke strengt nødvendig for *denne* periodisitetsanalysen av tot_G
        # self.accel_offset = self.calibrate_accel(50) # Kalibrering av akselerometer kan fjernes hvis vi bare ser på tot_G variasjon

        # Variabler for FFT analyse med glidende vindu
        self.sample_rate = 100  # Hz (må matche sleep i main loop)
        self.window_duration = 5 # sekunder
        self.window_size = int(self.sample_rate * self.window_duration) # Antall målinger i vinduet
        # Bruker kun én buffer som et glidende vindu
        self.tot_G_buffer = deque(maxlen=self.window_size)
        self.last_periodicity_status = None # Starter som ukjent

    # --- Kalibreringsfunksjoner (kan beholdes eller fjernes hvis unødvendig) ---
    def calibrate_accel(self, samples=100):
        print("starter kalibrering akselerometer")
        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        # Les inn data før kalibrering for å unngå initialiseringsverdier
        for _ in range(10):
            self.get_accel_data(g=True)
            sleep(0.02)

        for i in range(samples):
            try:
                accel_data = self.get_accel_data(g=True)
                offset['x'] += accel_data['x']
                offset['y'] += accel_data['y']
                offset['z'] += accel_data['z']
                print(f"Kalibrering aksel sample {i+1}/{samples}") # Feedback
            except Exception as e:
                print(f"Feil under aksel kalibrering sample {i+1}: {e}")
                samples -=1 # Juster antall samples hvis en feiler
            sleep(1.0 / self.sample_rate) # Bruk sample rate for sleep

        print("Ferdig kalibrert akselerometer")

        if samples > 0:
            offset['x'] /= samples
            offset['y'] /= samples
            # Juster Z for gravitasjon KUN hvis sensoren er i ro og orientert slik at Z peker opp/ned
            # Forutsetter at Z-aksen er vertikal under kalibrering
            offset['z'] = (offset['z'] / samples) - 1.0
        else:
             print("Kalibrering aksel feilet - ingen samples.")
             offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        return offset

    def calibrate_gyro(self, samples=100):
        print("starter kalibrering gyro")
        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        # Les inn data før kalibrering
        for _ in range(10):
             self.get_gyro_data()
             sleep(0.02)

        for i in range(samples):
            try:
                gyro_data = self.get_gyro_data()
                offset['x'] += gyro_data['x']
                offset['y'] += gyro_data['y']
                offset['z'] += gyro_data['z']
                print(f"Kalibrering gyro sample {i+1}/{samples}") # Feedback
            except Exception as e:
                print(f"Feil under gyro kalibrering sample {i+1}: {e}")
                samples -= 1
            sleep(1.0 / self.sample_rate) # Bruk sample rate for sleep

        print("Ferdig kalibrert gyro")

        if samples > 0:
            offset['x'] /= samples
            offset['y'] /= samples
            offset['z'] /= samples
        else:
            print("Kalibrering gyro feilet - ingen samples.")
            offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        return offset

    # --- Filtreringsfunksjoner ---
    def butter_highpass(self, cutoff, fs, order=5):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

    def highpass_filter(self, data, cutoff, fs, order=5):
        # Sikrer at data er et numpy array for filtrering
        data_np = np.array(data)
        # Sjekk for tilstrekkelig data lengde for filteret
        if len(data_np) <= order * 3: # En tommelfingerregel for å unngå ustabilitet
             print("Advarsel: Ikke nok data for stabilt filter. Returnerer ufiltrert data.")
             return data_np
        b, a = self.butter_highpass(cutoff, fs, order=order)
        y = lfilter(b, a, data_np)
        return y

    # --- Periodisitetsfunksjon (FFT-basert) ---
    def is_periodic(self, signal, threshold_ratio=0.15, min_significant_freqs=2, freq_range=(1.0, 10.0)):
        """
        Vurderer om et signal inneholder et periodisk mønster basert på FFT-analyse.
        Args:
            signal (list or np.array): Tidsseriedata som skal analyseres.
            threshold_ratio (float): Andel av maks magnitude for å telle en frekvens som signifikant.
                                     Høyere verdi krever tydeligere topper.
            min_significant_freqs (int): Minimum antall signifikante frekvenstopper for å kalle signalet periodisk.
                                         Minst 2 er vanligvis lurt (grunnfrekvens + harmonisk, eller flere topper).
            freq_range (tuple): (min_freq, max_freq) Hz. Frekvensområde å se etter topper i. Ignorerer DC (0 Hz) og veldig høye frekvenser.
        Returns:
            bool: True hvis signalet vurderes som periodisk, False ellers.
        """
        N = len(signal)
        if N < 2: # Trenger minst to punkter for FFT
            return False

        # Bruk rfft for reelle signaler (mer effektivt)
        fft_values = np.fft.rfft(signal)
        fft_freqs = np.fft.rfftfreq(N, d=1.0/self.sample_rate) # Beregn frekvensene
        fft_magnitude = np.abs(fft_values) / N # Normaliser

        # Finn magnitude innenfor ønsket frekvensområde
        min_freq, max_freq = freq_range
        freq_indices = np.where((fft_freqs >= min_freq) & (fft_freqs <= max_freq))[0]

        if len(freq_indices) == 0:
            #print("Ingen frekvenser i det gitte området.")
            return False # Ingen frekvenser i området å vurdere

        # Finn maks magnitude *kun* innenfor frekvensområdet
        max_magnitude_in_range = np.max(fft_magnitude[freq_indices]) if len(freq_indices) > 0 else 0

        if max_magnitude_in_range == 0:
             #print("Ingen signifikant energi i frekvensområdet.")
             return False

        # Sett terskel basert på maks magnitude i *området*
        threshold = threshold_ratio * max_magnitude_in_range

        # Tell antall topper over terskelen *kun* i frekvensområdet
        significant_freqs_count = np.sum(fft_magnitude[freq_indices] > threshold)

        # Debug print (kan fjernes senere)
        # print(f"FFT analyse: Freqs in range: {len(freq_indices)}, Max mag in range: {max_magnitude_in_range:.4f}, Threshold: {threshold:.4f}, Peaks found: {significant_freqs_count}")

        return significant_freqs_count >= min_significant_freqs

    # --- Hovedfunksjon for statusoppdatering ---
    def gi_status_aks(self):
        """Henter data, oppdaterer buffer, kjører analyse og returnerer status."""
        try:
            accel_data = self.get_accel_data(g=True)
            # Beregn total G-kraft (magnitude)
            # Merk: Offset-korreksjon brukes vanligvis *før* magnitude beregnes,
            # men for periodisitetsanalyse av *variasjonen* er det kanskje ikke kritisk.
            # Hvis du trenger nøyaktig G-verdi:
            # x = accel_data['x'] - self.accel_offset['x']
            # y = accel_data['y'] - self.accel_offset['y']
            # z = accel_data['z'] - self.accel_offset['z']
            # tot_G = math.sqrt(x**2 + y**2 + z**2)
            # Men for enkelhetens skyld, bruker vi rådata her for variasjonsanalyse:
            tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)

        except Exception as e:
            print(f"Klarte ikke å hente data fra MPU6050: {e}")
            # Returner en feilstatus eller forrige status?
            return {
                'total_G': -1, # Indikerer feil
                'is_periodic': self.last_periodicity_status, # Returner forrige kjente status
                'retning': "Ukjent"
            }

        # Legg til nyeste måling i det glidende vinduet
        self.tot_G_buffer.append(tot_G)

        # Bestem retning basert på Z-aksel
        # Bruk rå Z-verdi, eller offset-korrigert hvis kalibrering er aktiv
        # z_val = accel_data['z'] - self.accel_offset['z'] # Hvis kalibrert
        z_val = accel_data['z'] # Rå verdi
        if z_val <= -0.25:   # Terskelverdier kan trenge justering
            retning = "Ned"
        elif z_val >= 0.1:  # Terskelverdier kan trenge justering
            retning = "Opp"
        else:
            retning = "Plan"


        # Sjekk periodisitet KUN HVIS bufferen er full
        # Dette gir en stabil analyse basert på et fullt vindu
        current_status = self.last_periodicity_status # Behold forrige status som default
        if len(self.tot_G_buffer) == self.window_size:
            # Hent data fra det glidende vinduet
            window_data = list(self.tot_G_buffer)

            # 1. Filtrer data (valgfritt, men kan hjelpe)
            #    Høypassfilter fjerner DC-komponent (gjennomsnittlig G)
            #    og fokuserer på *endringer*. Cutoff bør være lavere enn forventet bevegelsesfrekvens.
            cutoff_freq = 0.5 # Hz (Fjern variasjoner saktere enn 2 sekunder)
            filtered_window = self.highpass_filter(window_data, cutoff=cutoff_freq, fs=self.sample_rate, order=5)

            # 2. Analyser periodisitet på det filtrerte vinduet
            #    Juster parametre i is_periodic() etter behov
            is_now_periodic = self.is_periodic(filtered_window, threshold_ratio=0.2, min_significant_freqs=2, freq_range=(0.8, 8.0)) # Eksempel: ser etter 0.8-8 Hz bevegelse

            # Oppdater statusen som lagres i klassen
            self.last_periodicity_status = is_now_periodic
            current_status = is_now_periodic # Bruk den nylig beregnede statusen

        # Returner resultatene
        return {
            'total_G': tot_G,
            'is_periodic': current_status, # Returnerer siste *beregnede* status (eller None i starten)
            'retning': retning
        }

# --- Hovedprogram ---
if __name__ == "__main__":
    try:
        # sensor = mpu6050(0x68) # Gammel måte
        mpu = MPU6050_Orientation(0x68, bus=1) # Sørg for korrekt adresse og buss
        print("MPU6050 initialisert.")
        # Optional: Kjør kalibrering her hvis ønskelig
        # mpu.calibrate_gyro(100)
        # mpu.calibrate_accel(100)
        print(f"Starter datainnsamling med {mpu.sample_rate} Hz sample rate og {mpu.window_duration}s vindu...")

    except Exception as e:
        print(f"Kunne ikke initialisere MPU6050: {e}")
        exit()

    while True:
        try:
            start_time = time.time() # For å måle loop-tid

            status = mpu.gi_status_aks()

            if status['total_G'] != -1: # Sjekk om datahenting var vellykket
                 # Formater output for lesbarhet
                g_str = f"{status['total_G']:.2f}"

                # Håndter None-status i starten
                if status['is_periodic'] is None:
                    period_str = "Venter på fullt vindu..."
                elif status['is_periodic']:
                    period_str = "Jevn"
                else:
                    period_str = "Ujevn"

                retning_str = status['retning']

                print(f"Total G: {g_str: <5} | Periodisitet: {period_str: <25} | Retning: {retning_str}")

            else:
                print("Feil ved lesing av sensor data.")


            # Sørg for at loopen kjører ca ved sample_rate
            loop_duration = time.time() - start_time
            sleep_time = (1.0 / mpu.sample_rate) - loop_duration
            if sleep_time > 0:
                sleep(sleep_time)
            # else:
            #     print("Advarsel: Loop tar lengre tid enn ønsket sample rate!") # Kan skje hvis analysen er treg

        except KeyboardInterrupt:
            print("\nAvslutter program.")
            break
        except Exception as e:
            print(f"\nEn feil oppstod i hovedloopen: {e}")
            sleep(1) # Vent litt før retry ved generell feil