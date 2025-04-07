# -*- coding: utf-8 -*-
"""
Sanntids deteksjon av "pustestopp" (stabil avstand) ved bruk av VL6180X.
Basert på "hurtig deteksjon"-logikk fra MATLAB-skript.
"""

import time
import board
import busio
import numpy as np
from collections import deque

# Importer det lokale Adafruit VL6180X biblioteket
# Denne importen fungerer fordi skriptet ligger i 'examples'-mappen
import adafruit_vl6180x

# --- Parametere som MÅ justeres ---
# VIKTIG: Sett den faktiske eller ønskede samplingsfrekvensen her!
TARGET_FS = 10  # Ønsket antall målinger per sekund (Hz)

# Parametere for deteksjon av stabil avstand ("pustestopp")
# !! Disse MÅ sannsynligvis justeres MYE for avstandsdata !!
FLATNESS_CHECK_WINDOW_SEC = 2.0 # Vindu for å sjekke "flathet" (sekunder)
FLATNESS_THRESHOLD = 2.0      # ENDRE: Maks std dev for "stabil" avstand (i mm?)
APNEA_DURATION_THRESHOLD_SEC = 5.0 # ENDRE: Min varighet av stabil avstand for deteksjon (sekunder)
# ----------------------------------------------------------

# --- Beregn bufferstørrelser basert på parametere ---
# Antall punkter for å beregne standardavvik
flatness_check_window_points = round(FLATNESS_CHECK_WINDOW_SEC * TARGET_FS)
if flatness_check_window_points < 2:
    flatness_check_window_points = 2 # Trenger minst 2 punkter for std dev

# Antall punkter for å sjekke kontinuerlig flathet (apné-varighet)
min_apnea_points = round(APNEA_DURATION_THRESHOLD_SEC * TARGET_FS)
if min_apnea_points < 1:
    min_apnea_points = 1

print("--- Konfigurasjon ---")
print(f"Mål Samplingsfrekvens (fs): {TARGET_FS} Hz")
print(f"Vindu for flathetssjekk: {FLATNESS_CHECK_WINDOW_SEC} s ({flatness_check_window_points} punkter)")
print(f"Terskel for flathet (maks std dev): {FLATNESS_THRESHOLD}")
print(f"Min varighet for 'pustestopp': {APNEA_DURATION_THRESHOLD_SEC} s ({min_apnea_points} punkter)")
print("----------------------")
print("VIKTIG: Juster parametere (spesielt terskler) for dine behov!")
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
# Holder de siste målingene for std dev beregning
data_buffer = deque(maxlen=flatness_check_window_points)
# Holder de siste 'flat'/'ikke flat'-statusene for varighetssjekk
flat_state_buffer = deque(maxlen=min_apnea_points)

# --- Tilstandsvariabler ---
apnea_active = False # Holder styr på om vi er i en "pustestopp"-tilstand
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
            print(f"Feil ved lesing fra sensor: {e}. Hopper over denne målingen.")
            # Vent litt før neste forsøk ved feil
            time.sleep(1.0 / TARGET_FS if TARGET_FS > 0 else 0.1)
            continue # Gå til neste iterasjon

        # Sjekk om målingen er gyldig
        if status == adafruit_vl6180x.ERROR_NONE:
            # 2. Legg til måling i databuffer
            data_buffer.append(range_mm)

            is_currently_flat = False # Anta ikke flat som default

            # 3. Beregn standardavvik hvis bufferet er fullt
            if len(data_buffer) == data_buffer.maxlen:
                std_dev = np.std(data_buffer)

                # 4. Sjekk om signalet er "flatt"
                is_currently_flat = std_dev < FLATNESS_THRESHOLD

            # 5. Oppdater flathetstilstand-bufferet
            # Vi legger alltid til status, selv om std dev ikke ble beregnet ennå
            # (dvs. i starten når data_buffer ikke er full). Da legges False til.
            flat_state_buffer.append(is_currently_flat)

            # 6. Sjekk for "pustestopp" (kontinuerlig flathet)
            apnea_detected_now = False
            if len(flat_state_buffer) == flat_state_buffer.maxlen:
                # Sjekk om ALLE elementene i bufferet er True
                if all(flat_state_buffer):
                    apnea_detected_now = True

            # 7. Administrer tilstand og skriv ut melding
            current_time = time.monotonic()
            if apnea_detected_now and not apnea_active:
                print(f"{time.strftime('%H:%M:%S')}: PUSTESTOPP DETEKTERT! (Stabil avstand i > {APNEA_DURATION_THRESHOLD_SEC:.1f} sek)")
                apnea_active = True
                last_print_time = current_time
            elif not apnea_detected_now and apnea_active:
                print(f"{time.strftime('%H:%M:%S')}: Pustestopp avsluttet.")
                apnea_active = False
                last_print_time = current_time
            # Skriv ut en "OK"-melding innimellom hvis ingenting skjer
            elif not apnea_active and (current_time - last_print_time > 10.0):
                 print(f"{time.strftime('%H:%M:%S')}: Ingen pustestopp detektert...")
                 last_print_time = current_time

        else:
             # Håndter ugyldig måling (kan skje hvis objektet er for nært/langt unna etc.)
             error_string = adafruit_vl6180x.RANGE_STATUS[status]
             # Skriv bare ut feilmelding en gang i blant for å unngå spam
             current_time = time.monotonic()
             if current_time - last_print_time > 5.0:
                 print(f"{time.strftime('%H:%M:%S')}: Ugyldig måling fra sensor (Status: {status} - {error_string})")
                 last_print_time = current_time
             # Ikke legg ugyldige målinger til i bufferne

        # 8. Kontroller løkkehastighet for å matche TARGET_FS
        loop_end_time = time.monotonic()
        elapsed_time = loop_end_time - loop_start_time
        sleep_time = (1.0 / TARGET_FS) - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)
        # else:
            # print(f"Advarsel: Løkken brukte for lang tid ({elapsed_time:.4f}s)")


except KeyboardInterrupt:
    print("\nAvslutter program...")

finally:
    # Rydd opp? (Ikke nødvendigvis noe å rydde opp her)
    print("Program avsluttet.")