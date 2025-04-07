# -*- coding: utf-8 -*-
"""
Sanntids deteksjon av "pustestopp" (stabil avstand) og estimering av
bevegelsesfrekvens ved bruk av VL6180X.
"""

import time
import board
import busio
import numpy as np
from collections import deque
# Importer nødvendige funksjoner fra SciPy for signalbehandling
try:
    from scipy.signal import find_peaks, savgol_filter
except ImportError:
    print("Feil: Kunne ikke importere 'scipy.signal'.")
    print("Installer SciPy med: pip install scipy")
    exit()

# Importer det lokale Adafruit VL6180X biblioteket
# Denne importen fungerer fordi skriptet ligger i 'examples'-mappen
import adafruit_vl6180x

# --- Parametere som MÅ justeres ---
# VIKTIG: Sett den faktiske eller ønskede samplingsfrekvensen her!
TARGET_FS = 10  # Ønsket antall målinger per sekund (Hz)

# Parametere for deteksjon av stabil avstand ("pustestopp")
FLATNESS_CHECK_WINDOW_SEC = 2.0 # Vindu for å sjekke "flathet" (sekunder)
FLATNESS_THRESHOLD = 2.0      # ENDRE: Maks std dev for "stabil" avstand (i mm?)
APNEA_DURATION_THRESHOLD_SEC = 5.0 # ENDRE: Min varighet av stabil avstand for deteksjon (sekunder)

# Parametere for frekvensanalyse (basert på topper i avstandssignalet)
FREQ_ANALYSIS_WINDOW_SEC = 10.0 # Vindu for frekvensanalyse (sekunder) - lengre enn flathetssjekk
SMOOTHING_WINDOW_FREQ_SEC = 0.7 # Smoothing vindu før toppdeteksjon (sekunder) - Kan trenge justering
POLYNOMIAL_ORDER_FREQ = 3       # Orden for Savitzky-Golay smoothing
PEAK_MIN_DISTANCE_SEC = 0.5     # ENDRE: Minimum tid mellom "bevegelsestopper" (sekunder?)
PEAK_MIN_PROMINENCE = 5.0       # ENDRE: Minimum "viktighet" for en topp (mm?)
MAX_REASONABLE_FREQ_HZ = 5.0    # ENDRE: Maks sannsynlig bevegelsesfrekvens (Hz?)
# ----------------------------------------------------------

# --- Beregn bufferstørrelser basert på parametere ---
# For flathetssjekk
flatness_check_window_points = round(FLATNESS_CHECK_WINDOW_SEC * TARGET_FS)
if flatness_check_window_points < 2: flatness_check_window_points = 2
min_apnea_points = round(APNEA_DURATION_THRESHOLD_SEC * TARGET_FS)
if min_apnea_points < 1: min_apnea_points = 1

# For frekvensanalyse
freq_analysis_window_points = round(FREQ_ANALYSIS_WINDOW_SEC * TARGET_FS)
if freq_analysis_window_points < 5: freq_analysis_window_points = 5 # Trenger litt data
smoothing_window_freq_points = round(SMOOTHING_WINDOW_FREQ_SEC * TARGET_FS)
if smoothing_window_freq_points < 3: smoothing_window_freq_points = 3 # Min for savgol
if smoothing_window_freq_points % 2 == 0: smoothing_window_freq_points += 1 # Må være oddetall
peak_min_distance_points = round(PEAK_MIN_DISTANCE_SEC * TARGET_FS)
if peak_min_distance_points < 1: peak_min_distance_points = 1

print("--- Konfigurasjon ---")
print(f"Mål Samplingsfrekvens (fs): {TARGET_FS} Hz")
print(f"Vindu flathet: {FLATNESS_CHECK_WINDOW_SEC} s ({flatness_check_window_points} pnt)")
print(f"Terskel flathet: {FLATNESS_THRESHOLD}")
print(f"Min varighet 'pustestopp': {APNEA_DURATION_THRESHOLD_SEC} s ({min_apnea_points} pnt)")
print(f"Vindu frekvensanalyse: {FREQ_ANALYSIS_WINDOW_SEC} s ({freq_analysis_window_points} pnt)")
print(f"Smoothing (frekvens): {SMOOTHING_WINDOW_FREQ_SEC} s ({smoothing_window_freq_points} pnt), Orden: {POLYNOMIAL_ORDER_FREQ}")
print(f"Min peak avstand: {PEAK_MIN_DISTANCE_SEC} s ({peak_min_distance_points} pnt)")
print(f"Min peak prominens: {PEAK_MIN_PROMINENCE}")
print(f"Maks fornuftig frekvens: {MAX_REASONABLE_FREQ_HZ} Hz")
print("----------------------")
print("VIKTIG: Juster parametere (spesielt terskler, prominens, frekvensgrense)!")
print("Starter sensor og analyse...")

# --- Initialiser I2C og Sensor ---
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_vl6180x.VL6180X(i2c)
    print("VL6180X sensor initialisert.")
except ValueError as e:
    print(f"Feil: Kunne ikke initialisere I2C eller sensor. Sjekk tilkobling.")
    print(f"Feilmelding: {e}")
    exit()
except Exception as e:
    print(f"En uventet feil oppstod under initialisering: {e}")
    exit()

# --- Databuffere ---
# For flathetssjekk (standardavvik)
data_buffer_stddev = deque(maxlen=flatness_check_window_points)
# For å sjekke varighet av flathet
flat_state_buffer = deque(maxlen=min_apnea_points)
# For frekvensanalyse (lengre buffer)
freq_analysis_buffer = deque(maxlen=freq_analysis_window_points)

# --- Tilstandsvariabler ---
apnea_active = False # Holder styr på om vi er i en "pustestopp"-tilstand
last_valid_bpm = None # Holder siste gyldige beregnede frekvens (i BPM)
last_print_time = time.monotonic() # For å unngå å spamme konsollen

# --- Hovedløkke ---
try:
    while True:
        loop_start_time = time.monotonic()

        # 1. Les avstand fra sensor
        try:
            range_mm = sensor.range
            status = sensor.range_status # Få status for å sjekke gyldighet
        except Exception as e:
            print(f"Feil ved lesing fra sensor: {e}. Hopper over.")
            time.sleep(1.0 / TARGET_FS if TARGET_FS > 0 else 0.1)
            continue

        # --- Behandle KUN gyldige målinger ---
        if status == adafruit_vl6180x.ERROR_NONE:
            # 2. Legg til måling i begge databuffere
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

            # === Frekvensanalyse (kun når bufferet er fullt) ===
            current_bpm = None # BPM for denne analysen
            if len(freq_analysis_buffer) == freq_analysis_buffer.maxlen:
                # Konverter til numpy array for signalbehandling
                signal_chunk = np.array(freq_analysis_buffer)

                # Smooth data FØR toppdeteksjon
                if len(signal_chunk) >= smoothing_window_freq_points:
                     try:
                         smoothed_chunk = savgol_filter(signal_chunk,
                                                        window_length=smoothing_window_freq_points,
                                                        polyorder=POLYNOMIAL_ORDER_FREQ)
                     except ValueError as sve:
                         # Kan skje hvis window > len(signal) eller polyorder >= window
                         # print(f"Smoothing feil: {sve}. Bruker usmoothet data.")
                         smoothed_chunk = signal_chunk # Fallback
                else:
                    # For kort chunk til å smoothe skikkelig
                    smoothed_chunk = signal_chunk

                # Finn topper
                try:
                    peaks, properties = find_peaks(smoothed_chunk,
                                                   distance=peak_min_distance_points,
                                                   prominence=PEAK_MIN_PROMINENCE)
                except Exception as fp_e:
                    # print(f"Find_peaks feil: {fp_e}") # Debugging
                    peaks = [] # Ingen topper ved feil

                # Beregn frekvens hvis vi har minst 2 topper
                if len(peaks) >= 2:
                    # Beregn perioder i samples og sekunder
                    periods_samples = np.diff(peaks)
                    periods_sec = periods_samples / TARGET_FS

                    # Unngå divisjon med null hvis periode er 0 (bør ikke skje med distance>0)
                    valid_period_mask = periods_sec > 1e-6
                    periods_sec = periods_sec[valid_period_mask]

                    if len(periods_sec) > 0:
                        # Beregn frekvenser i Hz
                        instant_frequencies_hz = 1.0 / periods_sec

                        # Filtrer bort urimelig høye frekvenser
                        valid_freq_mask = instant_frequencies_hz <= MAX_REASONABLE_FREQ_HZ
                        valid_frequencies_hz = instant_frequencies_hz[valid_freq_mask]

                        # Beregn gjennomsnittlig BPM hvis det er gyldige frekvenser
                        if len(valid_frequencies_hz) > 0:
                            mean_freq_hz = np.mean(valid_frequencies_hz)
                            current_bpm = mean_freq_hz * 60
                            last_valid_bpm = current_bpm # Oppdater siste gyldige BPM
                        # else:
                            # Ingen gyldige frekvenser funnet i dette vinduet
                            # last_valid_bpm beholdes fra forrige runde
                # else:
                     # Ikke nok topper funnet i dette vinduet
                     # last_valid_bpm beholdes fra forrige runde

            # === Administrer tilstand og skriv ut ===
            current_time = time.monotonic()
            status_changed = False

            if apnea_detected_now and not apnea_active:
                print(f"{time.strftime('%H:%M:%S')}: PUSTESTOPP DETEKTERT! (Stabil avstand > {APNEA_DURATION_THRESHOLD_SEC:.1f} s)")
                apnea_active = True
                status_changed = True
            elif not apnea_detected_now and apnea_active:
                print(f"{time.strftime('%H:%M:%S')}: Pustestopp avsluttet.")
                apnea_active = False
                status_changed = True

            # Skriv ut status med frekvens med jevne mellomrom HVIS IKKE i pustestopp
            if not apnea_active and (current_time - last_print_time > 5.0 or status_changed):
                freq_str = f"{last_valid_bpm:.1f} BPM" if last_valid_bpm is not None else "Beregner..."
                print(f"{time.strftime('%H:%M:%S')}: OK - Bevegelsesfrekvens: {freq_str}")
                last_print_time = current_time
            elif status_changed: # Oppdater tiden hvis status endret seg
                 last_print_time = current_time


        # --- Håndter ugyldige målinger ---
        else:
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
    # Rydd opp?
    print("Program avsluttet.")