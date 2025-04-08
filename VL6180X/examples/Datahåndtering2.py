# -*- coding: utf-8 -*-
"""
Sanntids deteksjon av pustestopp (stabil avstand > 2s)
OG uavhengig kontinuerlig estimering av bevegelsesfrekvens (over 10s vindu)
ved bruk av VL6180X.
"""

import time
import board
import busio
import numpy as np
from collections import deque
try:
    from scipy.signal import find_peaks, savgol_filter
except ImportError:
    print("Feil: Kunne ikke importere 'scipy.signal'.")
    print("Installer SciPy med: pip install scipy")
    exit()

import adafruit_vl6180x

# --- Parametere som MÅ justeres ---
TARGET_FS = 10  # Ønsket antall målinger per sekund (Hz)

# Parametere for PUSTESTOPP-deteksjon (stabil avstand)
# Vindu for å sjekke flathet (bør være <= varighetsterskelen)
FLATNESS_CHECK_WINDOW_SEC = 2.0
# Terskel for standardavvik for å anse signalet som "flatt" (mm?)
FLATNESS_THRESHOLD = 2.0
# Min varighet av stabilitet for å trigge pustestopp (s) - ENDRET til 2.0s
APNEA_DURATION_THRESHOLD_SEC = 2.0

# Parametere for FREKVENSANALYSE (kontinuerlig)
# Vindu for frekvensanalyse (s) - ENDRET til 10.0s
FREQ_ANALYSIS_WINDOW_SEC = 10.0
# Smoothing før toppdeteksjon (s)
SMOOTHING_WINDOW_FREQ_SEC = 0.7
POLYNOMIAL_ORDER_FREQ = 3       # Smoothing orden
PEAK_MIN_DISTANCE_SEC = 0.5     # Min tid mellom topper (s)
PEAK_MIN_PROMINENCE = 5.0       # Min topp-prominens (mm?)
MAX_REASONABLE_FREQ_HZ = 5.0    # Maks fornuftig frekvens (Hz)
# ----------------------------------------------------------

# --- Beregn bufferstørrelser ---
# Pustestopp
flatness_check_window_points = round(FLATNESS_CHECK_WINDOW_SEC * TARGET_FS)
if flatness_check_window_points < 2: flatness_check_window_points = 2
min_apnea_points = round(APNEA_DURATION_THRESHOLD_SEC * TARGET_FS)
if min_apnea_points < 1: min_apnea_points = 1

# Frekvens
freq_analysis_window_points = round(FREQ_ANALYSIS_WINDOW_SEC * TARGET_FS)
if freq_analysis_window_points < 5: freq_analysis_window_points = 5
smoothing_window_freq_points = round(SMOOTHING_WINDOW_FREQ_SEC * TARGET_FS)
if smoothing_window_freq_points < 3: smoothing_window_freq_points = 3
if smoothing_window_freq_points % 2 == 0: smoothing_window_freq_points += 1
peak_min_distance_points = round(PEAK_MIN_DISTANCE_SEC * TARGET_FS)
if peak_min_distance_points < 1: peak_min_distance_points = 1

print("--- Konfigurasjon ---")
print(f"Mål Samplingsfrekvens (fs): {TARGET_FS} Hz")
print(f"* Pustestopp Deteksjon *")
print(f"  Vindu flathet: {FLATNESS_CHECK_WINDOW_SEC} s ({flatness_check_window_points} pnt)")
print(f"  Terskel flathet: {FLATNESS_THRESHOLD}")
print(f"  Min varighet: {APNEA_DURATION_THRESHOLD_SEC} s ({min_apnea_points} pnt)")
print(f"* Frekvens Analyse *")
print(f"  Vindu analyse: {FREQ_ANALYSIS_WINDOW_SEC} s ({freq_analysis_window_points} pnt)")
print(f"  Smoothing: {SMOOTHING_WINDOW_FREQ_SEC} s ({smoothing_window_freq_points} pnt), Orden: {POLYNOMIAL_ORDER_FREQ}")
print(f"  Min peak avstand: {PEAK_MIN_DISTANCE_SEC} s ({peak_min_distance_points} pnt)")
print(f"  Min peak prominens: {PEAK_MIN_PROMINENCE}")
print(f"  Maks fornuftig frekvens: {MAX_REASONABLE_FREQ_HZ} Hz")
print("----------------------")
print("VIKTIG: Juster parametere!")
print("Starter sensor og analyse...")

# --- Initialiser I2C og Sensor ---
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_vl6180x.VL6180X(i2c)
    print("VL6180X sensor initialisert.")
except Exception as e:
    print(f"Feil ved initialisering: {e}")
    exit()

# --- Databuffere ---
# Buffer for flathetssjekk (pustestopp)
data_buffer_stddev = deque(maxlen=flatness_check_window_points)
# Buffer for å sjekke varighet av flathet (pustestopp)
flat_state_buffer = deque(maxlen=min_apnea_points)
# Buffer for frekvensanalyse (lengre)
freq_analysis_buffer = deque(maxlen=freq_analysis_window_points)

# --- Tilstandsvariabler ---
apnea_active = False             # Er pustestopp aktiv NÅ?
last_valid_bpm = None            # Siste gyldige beregnede frekvens
last_freq_print_time = 0         # For periodisk utskrift av frekvens
last_apnea_print_time = 0        # For periodisk utskrift av pustestopp-status

# --- Hovedløkke ---
try:
    while True:
        loop_start_time = time.monotonic()

        # 1. Les sensor
        try:
            range_mm = sensor.range
            status = sensor.range_status
        except Exception as e:
            print(f"Feil ved lesing: {e}. Hopper over.")
            time.sleep(1.0 / TARGET_FS if TARGET_FS > 0 else 0.1)
            continue

        # --- Behandle KUN gyldige målinger ---
        if status == adafruit_vl6180x.ERROR_NONE:
            current_time = time.monotonic()

            # 2. Oppdater buffere (begge får samme data)
            data_buffer_stddev.append(range_mm)
            freq_analysis_buffer.append(range_mm)

            # === Pustestopp Deteksjon (Uavhengig) ===
            is_currently_flat = False
            if len(data_buffer_stddev) == data_buffer_stddev.maxlen:
                std_dev = np.std(data_buffer_stddev)
                is_currently_flat = std_dev < FLATNESS_THRESHOLD
            flat_state_buffer.append(is_currently_flat)

            apnea_detected_now = False
            if len(flat_state_buffer) == flat_state_buffer.maxlen:
                if all(flat_state_buffer):
                    apnea_detected_now = True

            # Sjekk om pustestopp-status har endret seg
            if apnea_detected_now and not apnea_active:
                print(f"{time.strftime('%H:%M:%S')}: +++ PUSTESTOPP STARTET (Stabil > {APNEA_DURATION_THRESHOLD_SEC:.1f} s) +++")
                apnea_active = True
                last_apnea_print_time = current_time # Oppdater tid for statusutskrift
            elif not apnea_detected_now and apnea_active:
                print(f"{time.strftime('%H:%M:%S')}: --- PUSTESTOPP AVSLUTTET ---")
                apnea_active = False
                last_apnea_print_time = current_time # Oppdater tid for statusutskrift


            # === Frekvensanalyse (Uavhengig) ===
            if len(freq_analysis_buffer) == freq_analysis_buffer.maxlen:
                signal_chunk = np.array(freq_analysis_buffer)
                # Smoothing
                smoothed_chunk = []
                if len(signal_chunk) >= smoothing_window_freq_points:
                     try:
                         smoothed_chunk = savgol_filter(signal_chunk, smoothing_window_freq_points, POLYNOMIAL_ORDER_FREQ)
                     except ValueError: smoothed_chunk = signal_chunk
                else: smoothed_chunk = signal_chunk

                # Finn topper
                peaks_indices_in_chunk = []
                try:
                    peaks_indices, _ = find_peaks(smoothed_chunk, distance=peak_min_distance_points, prominence=PEAK_MIN_PROMINENCE)
                    if len(peaks_indices) >= 2: # Trenger minst 2 topper for frekvens
                        peaks_indices_in_chunk = peaks_indices
                except Exception: peaks_indices_in_chunk = []

                # Beregn frekvens hvis nok topper ble funnet
                if len(peaks_indices_in_chunk) >= 2:
                    periods_samples = np.diff(peaks_indices_in_chunk)
                    periods_sec = periods_samples / TARGET_FS
                    valid_period_mask = periods_sec > 1e-6
                    periods_sec = periods_sec[valid_period_mask]

                    if len(periods_sec) > 0:
                        instant_frequencies_hz = 1.0 / periods_sec
                        valid_freq_mask = instant_frequencies_hz <= MAX_REASONABLE_FREQ_HZ
                        valid_frequencies_hz = instant_frequencies_hz[valid_freq_mask]

                        if len(valid_frequencies_hz) > 0:
                            mean_freq_hz = np.mean(valid_frequencies_hz)
                            current_window_bpm = mean_freq_hz * 60
                            last_valid_bpm = current_window_bpm # Oppdater alltid med siste gyldige
                        # else: Hvis ingen gyldige frekv., behold forrige last_valid_bpm
                    # else: Hvis ingen gyldige perioder, behold forrige last_valid_bpm
                # else: Hvis færre enn 2 topper, behold forrige last_valid_bpm


            # === Periodisk Utskrift ===
            # Skriv ut pustestopp-status hvert 5. sekund
            if current_time - last_apnea_print_time > 5.0:
                status_str = "AKTIV" if apnea_active else "INAKTIV"
                print(f"{time.strftime('%H:%M:%S')}: Pustestopp: {status_str}")
                last_apnea_print_time = current_time

            # Skriv ut frekvens hvert 5. sekund (uavhengig av pustestopp)
            if current_time - last_freq_print_time > 5.0:
                freq_str = f"{last_valid_bpm:.1f} BPM" if last_valid_bpm is not None else "Beregner/Ingen"
                print(f"{time.strftime('%H:%M:%S')}: Frekvens: {freq_str}")
                last_freq_print_time = current_time


        # --- Håndter ugyldige målinger ---
        else:
             error_string = adafruit_vl6180x.RANGE_STATUS.get(status, f"Ukjent ({status})")
             current_time = time.monotonic()
             # Skriv kun feilmelding hvis det er lenge siden sist utskrift
             if current_time - max(last_apnea_print_time, last_freq_print_time) > 5.0:
                 print(f"{time.strftime('%H:%M:%S')}: Ugyldig måling (Status: {error_string})")
                 # Oppdater begge tidspunkter for å unngå umiddelbar ny utskrift
                 last_apnea_print_time = current_time
                 last_freq_print_time = current_time


        # --- Kontroller løkkehastighet ---
        loop_end_time = time.monotonic()
        elapsed_time = loop_end_time - loop_start_time
        sleep_time = (1.0 / TARGET_FS) - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\nAvslutter program...")
finally:
    print("Program avsluttet.")