# -*- coding: utf-8 -*-
from mpu6050 import mpu6050
from time import sleep
import math
import time
import os
import csv
from collections import deque
import numpy as np
from scipy.signal import butter, lfilter
# import smbus2

class MPU6050_Orientation(mpu6050):
    def __init__(self, address, bus=1):
        super().__init__(address, bus)
        self.set_gyro_range(self.GYRO_RANGE_250DEG)
        self.set_accel_range(self.ACCEL_RANGE_2G)
        self.set_filter_range(self.FILTER_BW_188)

        # Variabler for FFT analyse
        self.sample_rate = 100  # Hz
        self.window_duration = 5 # sekunder
        self.window_size = int(self.sample_rate * self.window_duration)
        self.tot_G_buffer = deque(maxlen=self.window_size)

        # --- Terskel for minimum bevegelse (TUNABLE) ---
        # Hvor mye standardavvik (i G) kreves i det filtrerte signalet
        # for å i det hele tatt vurdere periodisitet?
        self.min_movement_threshold = 0.03 # Startverdi, MÅ KANSKJE JUSTERES

        # --- Hysteresis / Debouncing Variabler ---
        self.reported_periodicity_status = False # Start som Ujevn
        self.consecutive_uneven_count = 0
        self.consecutive_even_count = 0
        # Juster tersklene etter behov
        self.uneven_confirm_threshold = int(self.sample_rate * 1.0) # 1 sek for å bekrefte Ujevn
        self.even_confirm_threshold = int(self.sample_rate * 0.5)   # 0.5 sek for å bekrefte Jevn

        # --- Parametere for is_periodic (TUNABLE) ---
        # Gjør denne litt strengere også
        self.fft_threshold_ratio = 0.3 # Andel av maks peak
        self.fft_min_significant_freqs = 2 # Antall topper
        self.fft_freq_range = (0.8, 8.0) # Frekvensområde (Hz)

        # Kalibrering (valgfritt)
        # self.gyro_offset = self.calibrate_gyro(50)
        # self.accel_offset = self.calibrate_accel(50)
        print(f"Initialisert. Min Movement Thr: {self.min_movement_threshold:.3f} G (std dev)")
        print(f"FFT Params: Ratio={self.fft_threshold_ratio}, Min Peaks={self.fft_min_significant_freqs}, Range={self.fft_freq_range} Hz")
        print(f"Hysteresis: Ujevn Thr={self.uneven_confirm_threshold}, Jevn Thr={self.even_confirm_threshold} samples")


    # ... (calibrate_accel, calibrate_gyro, butter_highpass, highpass_filter forblir de samme) ...
    def calibrate_accel(self, samples=100):
        print("starter kalibrering akselerometer")
        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        for _ in range(10): self.get_accel_data(g=True); sleep(0.02) # Tøm buffer
        valid_samples = 0
        for i in range(samples):
            try:
                accel_data = self.get_accel_data(g=True)
                offset['x'] += accel_data['x']; offset['y'] += accel_data['y']; offset['z'] += accel_data['z']
                valid_samples += 1
            except Exception as e: print(f"Feil under aksel kalibrering sample {i+1}: {e}")
            sleep(1.0 / self.sample_rate)
        print(f"Ferdig kalibrert akselerometer ({valid_samples} gyldige samples)")
        if valid_samples > 0:
            offset['x'] /= valid_samples; offset['y'] /= valid_samples
            offset['z'] = (offset['z'] / valid_samples)
        else: print("Kalibrering aksel feilet - ingen samples."); offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        return offset

    def calibrate_gyro(self, samples=100):
        print("starter kalibrering gyro")
        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        for _ in range(10): self.get_gyro_data(); sleep(0.02) # Tøm buffer
        valid_samples = 0
        for i in range(samples):
            try:
                gyro_data = self.get_gyro_data()
                offset['x'] += gyro_data['x']; offset['y'] += gyro_data['y']; offset['z'] += gyro_data['z']
                valid_samples += 1
            except Exception as e: print(f"Feil under gyro kalibrering sample {i+1}: {e}")
            sleep(1.0 / self.sample_rate)
        print(f"Ferdig kalibrert gyro ({valid_samples} gyldige samples)")
        if valid_samples > 0:
            offset['x'] /= valid_samples; offset['y'] /= valid_samples; offset['z'] /= valid_samples
        else: print("Kalibrering gyro feilet - ingen samples."); offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        return offset

    def butter_highpass(self, cutoff, fs, order=5):
        nyquist = 0.5 * fs; normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

    def highpass_filter(self, data, cutoff, fs, order=5):
        data_np = np.array(data)
        if len(data_np) <= order * 3: return data_np # Returner ufiltrert hvis ikke nok data
        b, a = self.butter_highpass(cutoff, fs, order=order)
        y = lfilter(b, a, data_np)
        return y

    # is_periodic bruker nå klassevariabler for parametere
    def is_periodic(self, signal):
        """ Vurderer om et signal er periodisk basert på FFT. """
        N = len(signal)
        if N < 2: return False
        fft_values = np.fft.rfft(signal)
        fft_freqs = np.fft.rfftfreq(N, d=1.0/self.sample_rate)
        fft_magnitude = np.abs(fft_values) / N
        min_freq, max_freq = self.fft_freq_range
        freq_indices = np.where((fft_freqs >= min_freq) & (fft_freqs <= max_freq))[0]
        if len(freq_indices) == 0: return False
        max_magnitude_in_range = np.max(fft_magnitude[freq_indices])
        if max_magnitude_in_range == 0: return False
        # Bruk klassevariabel for terskel ratio
        threshold = self.fft_threshold_ratio * max_magnitude_in_range
        # Bruk klassevariabel for min significant freqs
        significant_freqs_count = np.sum(fft_magnitude[freq_indices] > threshold)
        return significant_freqs_count >= self.fft_min_significant_freqs


    def gi_status_aks(self):
        """ Henter data, oppdaterer buffer, kjører analyse MED BEVEGELSES-SJEKK og hysteresis. """
        try:
            accel_data = self.get_accel_data(g=True)
            tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
        except Exception as e:
            print(f"Klarte ikke å hente data fra MPU6050: {e}")
            return {'total_G': -1, 'is_periodic': self.reported_periodicity_status, 'retning': "Ukjent"}

        self.tot_G_buffer.append(tot_G)

        z_val = accel_data['z']
        if z_val <= -0.25: retning = "Ned"
        elif z_val >= 0.1: retning = "Opp"
        else: retning = "Plan"

        # Kjør analyse KUN HVIS bufferen er full
        raw_is_periodic = None # Indikerer at analyse ikke er kjørt ennå denne syklusen
        signal_std_dev = 0.0 # For logging
        if len(self.tot_G_buffer) == self.window_size:
            window_data = list(self.tot_G_buffer)
            cutoff_freq = 0.5 # Hz
            filtered_window = self.highpass_filter(window_data, cutoff=cutoff_freq, fs=self.sample_rate, order=5)

            # ---> NYTT: Sjekk for minimum bevegelse <---
            signal_std_dev = np.std(filtered_window)

            if signal_std_dev < self.min_movement_threshold:
                # Ikke nok bevegelse, OVERSTYR til Ujevn
                raw_is_periodic = False
                # print(f"Debug: Std Dev {signal_std_dev:.4f} < {self.min_movement_threshold:.4f} -> Force Ujevn") # Debug print
            else:
                # Nok bevegelse, kjør den vanlige FFT-baserte sjekken
                # print(f"Debug: Std Dev {signal_std_dev:.4f} >= {self.min_movement_threshold:.4f} -> Run FFT Check") # Debug print
                raw_is_periodic = self.is_periodic(filtered_window) # Bruker nå klasseparametre

            # ---> Hysteresis logikk (samme som før) <---
            if raw_is_periodic is not None: # Bare oppdater tellere hvis analyse ble kjørt
                if raw_is_periodic:
                    self.consecutive_even_count += 1
                    self.consecutive_uneven_count = 0
                else:
                    self.consecutive_uneven_count += 1
                    self.consecutive_even_count = 0

                # Sjekk om terskler er nådd for å endre rapportert status
                if self.consecutive_even_count >= self.even_confirm_threshold:
                    if not self.reported_periodicity_status: # Bare print ved endring
                        print(f"*** Bytter til JEVN (Etter {self.consecutive_even_count} samples >= {self.even_confirm_threshold}) ***")
                    self.reported_periodicity_status = True
                elif self.consecutive_uneven_count >= self.uneven_confirm_threshold:
                    if self.reported_periodicity_status: # Bare print ved endring
                        print(f"*** Bytter til UJEVN (Etter {self.consecutive_uneven_count} samples >= {self.uneven_confirm_threshold}) ***")
                    self.reported_periodicity_status = False

        # Returner ALLTID den sist *rapporterte* statusen
        return {
            'total_G': tot_G,
            'is_periodic': self.reported_periodicity_status,
            'retning': retning,
            'std_dev': signal_std_dev # Returner std dev for evt. logging/tuning
        }


# --- Hovedprogram ---
if __name__ == "__main__":
    try:
        mpu = MPU6050_Orientation(0x68, bus=1)
        print("MPU6050 initialisert.")
        # Optional kalibrering
        print("="*40)
    except Exception as e:
        print(f"Kunne ikke initialisere MPU6050: {e}")
        exit()

    last_print_time = time.time()
    print_interval = 0.2 # Sekunder mellom hver print

    while True:
        try:
            start_time = time.time()
            status = mpu.gi_status_aks()

            current_time = time.time()
            if status['total_G'] != -1 and (current_time - last_print_time >= print_interval):
                g_str = f"{status['total_G']:.2f}"
                period_str = "Jevn" if status['is_periodic'] else "Ujevn"
                retning_str = status['retning']
                std_dev_str = f"{status.get('std_dev', 0.0):.4f}" # Hent std_dev hvis den finnes
                # Bruk \r for å oppdatere linjen, legg til mellomrom på slutten for å overskrive gamle tegn
                print(f"G: {g_str:<5} | Status: {period_str:<6} | Retning: {retning_str:<5} | StdDev(filt): {std_dev_str:<7} | Cnt(U/E): {mpu.consecutive_uneven_count}/{mpu.consecutive_even_count}   ", end='\r')
                last_print_time = current_time
            elif status['total_G'] == -1:
                print("Feil ved lesing av sensor data. ", end='\r')

            loop_duration = time.time() - start_time
            sleep_time = (1.0 / mpu.sample_rate) - loop_duration
            if sleep_time > 0:
                sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nAvslutter program.")
            break
        except Exception as e:
            print(f"\nEn feil oppstod i hovedloopen: {e}")
            sleep(1) # Vent litt før retry