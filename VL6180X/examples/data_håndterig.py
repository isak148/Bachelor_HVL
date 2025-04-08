# -*- coding: utf-8 -*-
"""
Sanntids deteksjon av "pustestopp" (stabil avstand) og estimering av
bevegelsesfrekvens ved bruk av VL6180X.
Beregner først frekvens basert på de første toppene etter pustestopp,
deretter kontinuerlig oppdatert frekvens.
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

# Pustestopp-deteksjon
FLATNESS_CHECK_WINDOW_SEC = 2# Vindu for std dev (s)
FLATNESS_THRESHOLD = 2.0      # Maks std dev for "stabil" (mm?)
APNEA_DURATION_THRESHOLD_SEC = 5.0 # Min varighet for pustestopp (s)

# Frekvensanalyse
FREQ_ANALYSIS_WINDOW_SEC = 10.0 # Vindu for kontinuerlig analyse (s)
SMOOTHING_WINDOW_FREQ_SEC = 0.7 # Smoothing vindu (s)
POLYNOMIAL_ORDER_FREQ = 3       # Smoothing orden
PEAK_MIN_DISTANCE_SEC = 0.5     # Min tid mellom topper (s)
PEAK_MIN_PROMINENCE = 5.0       # Min topp-prominens (mm?)
MAX_REASONABLE_FREQ_HZ = 5.0    # Maks fornuftig frekvens (Hz)
NUM_PEAKS_FOR_INIT_FREQ = 3     # Antall topper for *initiell* frekvens etter pustestopp
# ----------------------------------------------------------

# --- Beregn bufferstørrelser ---
flatness_check_window_points = round(FLATNESS_CHECK_WINDOW_SEC * TARGET_FS)
if flatness_check_window_points < 2: flatness_check_window_points = 2
min_apnea_points = round(APNEA_DURATION_THRESHOLD_SEC * TARGET_FS)
if min_apnea_points < 1: min_apnea_points = 1
freq_analysis_window_points = round(FREQ_ANALYSIS_WINDOW_SEC * TARGET_FS)
if freq_analysis_window_points < 5: freq_analysis_window_points = 5
smoothing_window_freq_points = round(SMOOTHING_WINDOW_FREQ_SEC * TARGET_FS)
if smoothing_window_freq_points < 3: smoothing_window_freq_points = 3
if smoothing_window_freq_points % 2 == 0: smoothing_window_freq_points += 1
peak_min_distance_points = round(PEAK_MIN_DISTANCE_SEC * TARGET_FS)
if peak_min_distance_points < 1: peak_min_distance_points = 1

print("--- Konfigurasjon ---")
# ... (print av alle parametere som før) ...
print(f"Antall topper for initiell post-apnea frekvens: {NUM_PEAKS_FOR_INIT_FREQ}")
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
data_buffer_stddev = deque(maxlen=flatness_check_window_points)
flat_state_buffer = deque(maxlen=min_apnea_points)
freq_analysis_buffer = deque(maxlen=freq_analysis_window_points)

# --- Tilstandsvariabler ---
apnea_active = False
last_valid_bpm = None
last_print_time = time.monotonic()
calculating_initial_freq = False # Ny tilstand: Måler vi initiell frekvens?
initial_freq_peak_times = [] # Liste for tidspunkter for initielle topper
last_peak_add_time = 0

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

            # 2. Oppdater buffere
            data_buffer_stddev.append(range_mm)
            freq_analysis_buffer.append(range_mm)

            # === Flathet / Pustestopp Deteksjon ===
            is_currently_flat = False
            if len(data_buffer_stddev) == data_buffer_stddev.maxlen:
                std_dev = np.std(data_buffer_stddev)
                is_currently_flat = std_dev < FLATNESS_THRESHOLD
            flat_state_buffer.append(is_currently_flat)
            apnea_detected_now = False
            if len(flat_state_buffer) == flat_state_buffer.maxlen:
                if all(flat_state_buffer):
                    apnea_detected_now = True

            # === Toppdeteksjon (kjøres alltid når frekvensvindu er fullt) ===
            peaks_indices_in_chunk = []
            smoothed_chunk = []
            if len(freq_analysis_buffer) == freq_analysis_buffer.maxlen:
                signal_chunk = np.array(freq_analysis_buffer)
                # Smoothing
                if len(signal_chunk) >= smoothing_window_freq_points:
                     try:
                         smoothed_chunk = savgol_filter(signal_chunk, smoothing_window_freq_points, POLYNOMIAL_ORDER_FREQ)
                     except ValueError: smoothed_chunk = signal_chunk
                else: smoothed_chunk = signal_chunk
                # Finn topper
                try:
                    peaks_indices, _ = find_peaks(smoothed_chunk, distance=peak_min_distance_points, prominence=PEAK_MIN_PROMINENCE)
                    if len(peaks_indices) > 0: peaks_indices_in_chunk = peaks_indices
                except Exception: peaks_indices_in_chunk = []

            # === Behandle tilstandsendringer (Start/Stopp Pustestopp) ===
            status_changed = False
            # Pustestopp starter NÅ
            if apnea_detected_now and not apnea_active:
                print(f"{time.strftime('%H:%M:%S')}: *** PUSTESTOPP DETEKTERT! (Stabil > {APNEA_DURATION_THRESHOLD_SEC:.1f} s) ***")
                apnea_active = True
                status_changed = True
                # Nullstill frekvenstilstander
                calculating_initial_freq = False
                initial_freq_peak_times = []
                # last_valid_bpm beholdes til ny beregnes

            # Pustestopp slutter NÅ
            elif not apnea_detected_now and apnea_active:
                print(f"{time.strftime('%H:%M:%S')}: --- Pustestopp avsluttet. Måler initiell frekvens... ---")
                apnea_active = False
                status_changed = True
                # Start innsamling for initiell frekvens
                calculating_initial_freq = True
                initial_freq_peak_times = [] # Start med tom liste
                last_peak_add_time = 0

            # === Frekvensberegning (Både initiell og kontinuerlig) ===
            # --- Initiell frekvensberegning ---
            if calculating_initial_freq and len(peaks_indices_in_chunk) > 0:
                # Sjekk om siste topp i vinduet er nylig
                last_peak_index_in_chunk = peaks_indices_in_chunk[-1]
                peak_is_recent_threshold = 3
                if last_peak_index_in_chunk >= (len(smoothed_chunk) - peak_is_recent_threshold):
                    # Unngå å legge til samme topp for raskt
                    if current_time - last_peak_add_time > (PEAK_MIN_DISTANCE_SEC / 2.0):
                        initial_freq_peak_times.append(current_time)
                        last_peak_add_time = current_time
                        # print(f"DEBUG: Initiell peak lagt til. Antall: {len(initial_freq_peak_times)}/{NUM_PEAKS_FOR_INIT_FREQ}") # Debug

                        # Sjekk om vi har samlet nok topper for initiell beregning
                        if len(initial_freq_peak_times) >= NUM_PEAKS_FOR_INIT_FREQ:
                            # Beregn frekvens basert på innsamlede tidspunkter
                            periods_sec = np.diff(initial_freq_peak_times)
                            valid_period_mask = periods_sec > 1e-6
                            periods_sec = periods_sec[valid_period_mask]

                            if len(periods_sec) > 0:
                                instant_frequencies_hz = 1.0 / periods_sec
                                valid_freq_mask = instant_frequencies_hz <= MAX_REASONABLE_FREQ_HZ
                                valid_frequencies_hz = instant_frequencies_hz[valid_freq_mask]

                                if len(valid_frequencies_hz) > 0:
                                    mean_freq_hz = np.mean(valid_frequencies_hz)
                                    init_bpm = mean_freq_hz * 60
                                    last_valid_bpm = init_bpm # Oppdater global BPM
                                    print(f"{time.strftime('%H:%M:%S')}: === Initiell frekvens etter pustestopp: {last_valid_bpm:.1f} BPM ===")
                                else:
                                    print(f"{time.strftime('%H:%M:%S')}: Kunne ikke beregne gyldig initiell frekvens (filtrert).")
                                    # Behold evt. gammel last_valid_bpm eller sett til None? La den stå.
                            else:
                                 print(f"{time.strftime('%H:%M:%S')}: Kunne ikke beregne gyldig initiell frekvens (perioder).")

                            # Fullfør initiell måling
                            calculating_initial_freq = False
                            initial_freq_peak_times = []
                            status_changed = True # For å trigge OK-utskrift raskere


            # --- Kontinuerlig frekvensberegning ---
            # Kjør KUN hvis IKKE pustestopp OG IKKE i initiell måling OG vi har nok data (vindu fullt)
            elif not apnea_active and not calculating_initial_freq and len(peaks_indices_in_chunk) >= 2:
                 # Beregn frekvens basert på ALLE topper funnet i DETTE vinduet
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
                         last_valid_bpm = current_window_bpm # Oppdater global BPM kontinuerlig

            # === Administrer Utskrift ===
            # Oppdater utskriftstidspunkt hvis status endret seg
            if status_changed:
                 last_print_time = current_time

            # Skriv ut OK melding med jevne mellomrom når systemet er aktivt
            # (ikke pustestopp, ikke i initiell frekvensmåling)
            if not apnea_active and not calculating_initial_freq and (current_time - last_print_time > 5.0): # Redusert intervall
                 freq_str = f"{last_valid_bpm:.1f} BPM" if last_valid_bpm is not None else "Beregner..."
                 print(f"{time.strftime('%H:%M:%S')}: OK - Frekvens: {freq_str}")
                 last_print_time = current_time

        # --- Håndter ugyldige målinger ---
        else:
            # ... (samme som før) ...
             error_string = adafruit_vl6180x.RANGE_STATUS.get(status, f"Ukjent ({status})")
             current_time = time.monotonic()
             if current_time - last_print_time > 5.0:
                 print(f"{time.strftime('%H:%M:%S')}: Ugyldig måling (Status: {error_string})")
                 last_print_time = current_time

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