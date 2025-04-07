# -*- coding: utf-8 -*-
"""
Sanntids deteksjon av "pustestopp" (stabil avstand) og estimering av
bevegelsesfrekvens *etter* pustestopp ved bruk av VL6180X.
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
FREQ_ANALYSIS_WINDOW_SEC = 10.0 # Vindu for frekvensanalyse (sekunder) - MÅ være >= FLATNESS_CHECK_WINDOW_SEC
SMOOTHING_WINDOW_FREQ_SEC = 0.7 # Smoothing vindu før toppdeteksjon (sekunder) - Kan trenge justering
POLYNOMIAL_ORDER_FREQ = 3       # Orden for Savitzky-Golay smoothing
PEAK_MIN_DISTANCE_SEC = 0.5     # ENDRE: Minimum tid mellom "bevegelsestopper" (sekunder?)
PEAK_MIN_PROMINENCE = 5.0       # ENDRE: Minimum "viktighet" for en topp (mm?)
MAX_REASONABLE_FREQ_HZ = 5.0    # ENDRE: Maks sannsynlig bevegelsesfrekvens (Hz?)
NUM_PEAKS_FOR_FREQ = 3          # Antall topper å samle etter pustestopp for frekvensberegning (3 topper = 2 perioder)
# ----------------------------------------------------------

# --- Beregn bufferstørrelser basert på parametere ---
# For flathetssjekk
flatness_check_window_points = round(FLATNESS_CHECK_WINDOW_SEC * TARGET_FS)
if flatness_check_window_points < 2: flatness_check_window_points = 2
min_apnea_points = round(APNEA_DURATION_THRESHOLD_SEC * TARGET_FS)
if min_apnea_points < 1: min_apnea_points = 1

# For frekvensanalyse
freq_analysis_window_points = round(FREQ_ANALYSIS_WINDOW_SEC * TARGET_FS)
if freq_analysis_window_points < 5: freq_analysis_window_points = 5
smoothing_window_freq_points = round(SMOOTHING_WINDOW_FREQ_SEC * TARGET_FS)
if smoothing_window_freq_points < 3: smoothing_window_freq_points = 3
if smoothing_window_freq_points % 2 == 0: smoothing_window_freq_points += 1
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
print(f"Antall topper for post-apnea frekvens: {NUM_PEAKS_FOR_FREQ}")
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
data_buffer_stddev = deque(maxlen=flatness_check_window_points)
flat_state_buffer = deque(maxlen=min_apnea_points)
freq_analysis_buffer = deque(maxlen=freq_analysis_window_points)

# --- Tilstandsvariabler ---
apnea_active = False
last_valid_bpm = None
last_print_time = time.monotonic()
# Nye tilstandsvariabler for post-apnea frekvensberegning
calculating_post_apnea_freq = False
post_apnea_peak_times = [] # Liste for å lagre tidspunkter for topper funnet etter apnea
last_peak_add_time = 0 # For å unngå å legge til samme topp for raskt

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
            current_time = time.monotonic() # Få nåværende tid tidlig

            # 2. Legg til måling i buffere
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

            # === Toppdeteksjon (kjøres når frekvensvindu er fullt) ===
            peaks_indices_in_chunk = [] # Nullstill for denne runden
            smoothed_chunk = []
            if len(freq_analysis_buffer) == freq_analysis_buffer.maxlen:
                signal_chunk = np.array(freq_analysis_buffer)
                # Smooth data
                if len(signal_chunk) >= smoothing_window_freq_points:
                     try:
                         smoothed_chunk = savgol_filter(signal_chunk,
                                                        window_length=smoothing_window_freq_points,
                                                        polyorder=POLYNOMIAL_ORDER_FREQ)
                     except ValueError:
                         smoothed_chunk = signal_chunk # Fallback
                else:
                    smoothed_chunk = signal_chunk

                # Finn topper i det smoothede vinduet
                try:
                    peaks_indices, _ = find_peaks(smoothed_chunk,
                                                  distance=peak_min_distance_points,
                                                  prominence=PEAK_MIN_PROMINENCE)
                    if len(peaks_indices) > 0:
                        peaks_indices_in_chunk = peaks_indices # Lagre funnede indekser
                except Exception:
                    peaks_indices_in_chunk = []


            # === Post-Apnea Frekvensberegning (Innsamling og Kalkulering) ===
            # Sjekk om vi er i post-apnea modus og om en *ny* topp er detektert *nylig* i vinduet
            if calculating_post_apnea_freq and len(peaks_indices_in_chunk) > 0:
                # Sjekk om den siste toppen i vinduet er nær slutten av vinduet
                last_peak_index_in_chunk = peaks_indices_in_chunk[-1]
                # Hvor "nylig" må toppen være? (f.eks. innenfor de siste ~3 samples)
                peak_is_recent_threshold = 3
                if last_peak_index_in_chunk >= (len(smoothed_chunk) - peak_is_recent_threshold):
                    # Unngå å legge til samme topp (eller topper for tett) raskt etter hverandre
                    # Bruker en halv minimum peak-avstand som en heuristikk
                    if current_time - last_peak_add_time > (PEAK_MIN_DISTANCE_SEC / 2.0):
                        # print(f"DEBUG: Recent peak detected at index {last_peak_index_in_chunk} at time {current_time:.2f}") # Debug
                        post_apnea_peak_times.append(current_time)
                        last_peak_add_time = current_time
                        # print(f"DEBUG: Peaks collected: {len(post_apnea_peak_times)}/{NUM_PEAKS_FOR_FREQ}") # Debug

                        # Sjekk om vi har samlet nok topper
                        if len(post_apnea_peak_times) >= NUM_PEAKS_FOR_FREQ:
                            # print(f"DEBUG: Calculating post-apnea freq from times: {post_apnea_peak_times}") # Debug
                            # Beregn perioder og frekvens basert på de innsamlede tidspunktene
                            periods_sec = np.diff(post_apnea_peak_times)
                            valid_period_mask = periods_sec > 1e-6 # Unngå divisjon med 0
                            periods_sec = periods_sec[valid_period_mask]

                            if len(periods_sec) > 0:
                                instant_frequencies_hz = 1.0 / periods_sec
                                # Filtrer bort urimelige frekvenser
                                valid_freq_mask = instant_frequencies_hz <= MAX_REASONABLE_FREQ_HZ
                                valid_frequencies_hz = instant_frequencies_hz[valid_freq_mask]

                                if len(valid_frequencies_hz) > 0:
                                    mean_freq_hz = np.mean(valid_frequencies_hz)
                                    current_bpm = mean_freq_hz * 60
                                    last_valid_bpm = current_bpm # Oppdater siste gyldige
                                    print(f"{time.strftime('%H:%M:%S')}: === Post-pustestopp frekvens: {last_valid_bpm:.1f} BPM (basert på {len(post_apnea_peak_times)} topper) ===")
                                else:
                                    print(f"{time.strftime('%H:%M:%S')}: Kunne ikke beregne gyldig post-pustestopp frekvens (filtrert).")
                                    last_valid_bpm = None # Nullstill hvis filtrert bort
                            else:
                                 print(f"{time.strftime('%H:%M:%S')}: Kunne ikke beregne gyldig post-pustestopp frekvens (perioder).")
                                 last_valid_bpm = None # Nullstill

                            # Nullstill for neste post-apnea periode
                            calculating_post_apnea_freq = False
                            post_apnea_peak_times = []
                            # La systemet gå litt før neste "OK" melding
                            last_print_time = current_time + 2.0


            # === Administrer Hovedtilstand og skriv ut ===
            status_changed = False
            # Pustestopp starter NÅ
            if apnea_detected_now and not apnea_active:
                print(f"{time.strftime('%H:%M:%S')}: *** PUSTESTOPP DETEKTERT! (Stabil avstand > {APNEA_DURATION_THRESHOLD_SEC:.1f} s) ***")
                apnea_active = True
                status_changed = True
                # Nullstill frekvensberegning hvis apnea starter
                calculating_post_apnea_freq = False
                post_apnea_peak_times = []

            # Pustestopp slutter NÅ
            elif not apnea_detected_now and apnea_active:
                print(f"{time.strftime('%H:%M:%S')}: --- Pustestopp avsluttet. Måler frekvens... ---")
                apnea_active = False
                status_changed = True
                # Start innsamling for post-apnea frekvens
                calculating_post_apnea_freq = True
                post_apnea_peak_times = [] # Start med tom liste
                last_peak_add_time = 0

            # Oppdater utskriftstidspunkt hvis status endret seg
            if status_changed:
                 last_print_time = current_time

            # Skriv ut OK melding sjeldnere hvis ingenting spesielt skjer
            # og vi IKKE er i pustestopp og IKKE aktivt måler post-apnea frekvens
            elif not apnea_active and not calculating_post_apnea_freq and (current_time - last_print_time > 10.0):
                 # Viser siste gyldige frekvens (fra forrige post-apnea eller eldre)
                 freq_str = f"{last_valid_bpm:.1f} BPM" if last_valid_bpm is not None else "N/A"
                 print(f"{time.strftime('%H:%M:%S')}: OK - Siste kjente frekvens: {freq_str}")
                 last_print_time = current_time


        # --- Håndter ugyldige målinger ---
        else:
             error_string = adafruit_vl6180x.RANGE_STATUS.get(status, f"Ukjent ({status})")
             current_time = time.monotonic()
             if current_time - last_print_time > 5.0:
                 # Unngå å spamme ved kontinuerlige feil
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
    # Rydd opp? (Ikke nødvendigvis noe her)
    print("Program avsluttet.")