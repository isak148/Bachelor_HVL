# -*- coding: utf-8 -*-
import time
import numpy as np
from mpu6050 import mpu6050 # Importer MPU6050 biblioteket
import math
from collections import deque # For effektiv bufferhåndtering (valgfritt)

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

# Gjennomsnittsberegning
avg_group_size = 10 # Antall klassifiseringer å ta gjennomsnitt av

# --- Initialiser Sensor ---
try:
    sensor = mpu6050(sensor_address)
    print(f"MPU6050 initialisert på adresse {hex(sensor_address)}.")
    # Optional: Sett sensor-rekkevidde hvis ønskelig
    # sensor.set_accel_range(sensor.ACCEL_RANGE_2G)
except Exception as e:
    print(f"Kunne ikke initialisere MPU6050: {e}")
    print("Sjekk tilkobling og I2C-adresse.")
    exit()

# --- Sanntidsanalyse ---
print(f"Starter kontinuerlig analyse...")
print(f"Vindustørrelse: {window_size} samples ({window_duration_s} sek)")
print(f"Gjennomsnitt beregnes over {avg_group_size} vinduer ({avg_group_size * window_duration_s} sek)")
print("Trykk Ctrl+C for å avslutte.")

current_window_samples = [] # Liste for å samle samples for ett vindu
classification_history = [] # Liste for å lagre nivå (1, 2, 3)

try:
    while True:
        loop_start_time = time.time()

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

        # --- Oppretthold samplingsrate ---
        loop_end_time = time.time()
        processing_time = loop_end_time - loop_start_time
        sleep_time = (1.0 / target_sample_rate_hz) - processing_time
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\nAvslutter program.")