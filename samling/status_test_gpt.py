#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import os
import board
import busio
import math # Brukes i placeholder status-logikk

# --- Legg til rotmappen i sys.path for å finne lokale moduler ---
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)
# --- Slutt på sti-justering ---

# --- Importer sensor-spesifikke biblioteker og klasser ---
try:
    # Antar at MPU6050_Orientation håndterer sin egen MPU6050-initialisering
    from mpu6050_1.mpu6050_2.Bachelor_test1 import MPU6050_Orientation
except ImportError as e:
    print(f"ADVARSEL: Kunne ikke importere 'MPU6050_Orientation'. {e}")
    MPU6050_Orientation = None

try:
    import ms5837
except ImportError:
    print("ADVARSEL: Kunne ikke importere 'ms5837'. Sjekk at biblioteket er tilgjengelig.")
    ms5837 = None

try:
    import adafruit_vl6180x
except ImportError:
    print("ADVARSEL: Kunne ikke importere 'adafruit_vl6180x'. Sjekk at biblioteket er installert.")
    adafruit_vl6180x = None

try:
    import adafruit_ads1x15.ads1015 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
except ImportError:
    print("ADVARSEL: Kunne ikke importere 'adafruit_ads1x15'. Sjekk at biblioteket er installert.")
    ADS = None
    AnalogIn = None

# --- Konstanter ---
PRESSURE_THRESHOLD_WATER_MBAR = 1030  # Trykk i mbar for å anta at personen er i vann (ca. >15-20cm dybde)
                                      # MÅ KALIBRERES! Normalt lufttrykk er ~1013 mbar.
ADC_PULSE_CHANNEL = ADS.P0 if ADS else None # Kanal for puls på ADS1015 (A0)

class status:

    def __init__(self):
        """Initialiserer sensorobjekter og statusvariabler."""
        print("Initialiserer status-klassen og sensorer...")
        self.variabler = {} # Kan brukes til å lagre historikk e.l. senere

        # Sensorobjekter initialiseres til None
        self.i2c = None
        self.mpu_orient = None
        self.sensor_ms5837 = None
        self.sensor_vl6180x = None
        self.ads = None
        self.puls_channel = None

        # Statusflagg for sensorer
        self.mpu_ok = False
        self.ms5837_ok = False
        self.vl6180x_ok = False
        self.ads_ok = False
        self.pulse_ok = False # Status for om pulskanalen er klar

        # --- Initialiser I2C ---
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            print("[INFO] I2C-buss initialisert OK.")
        except Exception as e:
            print(f"[FEIL] Kunne ikke initialisere I2C: {e}")
            # Prøver ikke å initialisere I2C-baserte sensorer hvis bussen feilet
            return

        # --- Initialiser MPU6050 ---
        if MPU6050_Orientation:
            try:
                # Antar at MPU6050_Orientation tar I2C-buss eller adresse som argument?
                # Eller håndterer den init selv? Må sjekkes i Bachelor_test1.py
                # Hvis den initierer selv, trenger den kanskje ikke self.i2c her.
                self.mpu_orient = MPU6050_Orientation() # Juster kall om nødvendig
                # Trenger en måte å vite om init lyktes. Kanskje en init-metode i MPU6050_Orientation?
                # For nå antar vi at det går bra hvis ingen exception kastes.
                print("[INFO] MPU6050 (via MPU6050_Orientation) initialisert OK.")
                self.mpu_ok = True
            except Exception as e:
                print(f"[FEIL] Kunne ikke initialisere MPU6050_Orientation: {e}")
        else:
            print("[INFO] MPU6050: Initialisering hoppet over (modul mangler).")


        # --- Initialiser MS5837 (Trykk/Temperatur) ---
        if ms5837:
            try:
                self.sensor_ms5837 = ms5837.MS5837_30BA() # Bruker standard I2C buss (1)
                if not self.sensor_ms5837.init():
                    print("[FEIL] MS5837: Kunne ikke initialisere sensoren. Sjekk tilkobling/adresse.")
                    self.sensor_ms5837 = None
                else:
                    print("[OK] MS5837: Sensor initialisert.")
                    self.ms5837_ok = True
            except Exception as e:
                print(f"[FEIL] MS5837: Kunne ikke initialisere: {e}")
        else:
            print("[INFO] MS5837: Initialisering hoppet over (bibliotek mangler).")

        # --- Initialiser VL6180X (Avstand/Lys) ---
        if adafruit_vl6180x:
            try:
                self.sensor_vl6180x = adafruit_vl6180x.VL6180X(self.i2c)
                # Testlesing for å bekrefte kommunikasjon
                _ = self.sensor_vl6180x.range
                print("[OK] VL6180X: Sensor initialisert.")
                self.vl6180x_ok = True
            except Exception as e:
                print(f"[FEIL] VL6180X: Kunne ikke initialisere eller kommunisere: {e}")
                self.sensor_vl6180x = None
        else:
            print("[INFO] VL6180X: Initialisering hoppet over (bibliotek mangler).")


        # --- Initialiser ADS1015 (for AD8232 EKG) ---
        if ADS and AnalogIn and ADC_PULSE_CHANNEL is not None:
            try:
                self.ads = ADS.ADS1015(self.i2c)
                # Test lesing av kanal for å bekrefte kommunikasjon
                self.puls_channel = AnalogIn(self.ads, ADC_PULSE_CHANNEL)
                _ = self.puls_channel.value # Test read
                print(f"[OK] ADS1015: ADC initialisert, kanal {ADC_PULSE_CHANNEL} klar.")
                self.ads_ok = True
                self.pulse_ok = True # Betyr at ADC og kanal er klar
            except Exception as e:
                print(f"[FEIL] ADS1015: Kunne ikke initialisere ADC eller kanal {ADC_PULSE_CHANNEL}: {e}")
                self.ads = None
                self.puls_channel = None
        else:
            print("[INFO] ADS1015: Initialisering hoppet over (bibliotek/kanal mangler).")

        print("--- Initialisering fullført ---")

    def get_data_aks(self):
        """ Henter status og orienteringsdata fra MPU6050. """
        default_aks_data = {"status": "ukjent", "roll": 0.0, "pitch": 0.0, "raw": None, "error": True}
        if self.mpu_ok and self.mpu_orient:
            try:
                # Kall den faktiske metoden som gir data
                # Formatet på returverdien MÅ sjekkes i Bachelor_test1.py
                aks_data = self.mpu_orient.gi_status_aks()

                # Sørg for at returformatet er som forventet, ellers bruk default
                if isinstance(aks_data, dict) and 'status' in aks_data and 'roll' in aks_data and 'pitch' in aks_data:
                     aks_data["error"] = False
                     return {"aks": aks_data}
                else:
                     print("[FEIL] MPU6050: Uventet dataformat fra gi_status_aks().")
                     return {"aks": default_aks_data}

            except Exception as e:
                print(f"[FEIL] MPU6050: Kunne ikke hente data: {e}")
                return {"aks": default_aks_data}
        else:
            # print("[DEBUG] MPU6050: Henting hoppet over (ikke initialisert).")
            return {"aks": default_aks_data}

    def get_data_LFR_Preasure(self):
        """ Henter avstand (VL6180X) og trykk (MS5837) data. """
        data_lfr = None
        data_pressure = None
        lfr_error = True
        pressure_error = True

        # Hent data fra VL6180X (Avstand)
        if self.vl6180x_ok and self.sensor_vl6180x:
            try:
                data_lfr = self.sensor_vl6180x.range
                lfr_error = False
            except Exception as e:
                print(f"[FEIL] VL6180X: Kunne ikke lese avstand: {e}")
        # else:
            # print("[DEBUG] VL6180X: Henting hoppet over (ikke initialisert).")

        # Hent data fra MS5837 (Trykk)
        if self.ms5837_ok and self.sensor_ms5837:
            try:
                if self.sensor_ms5837.read(): # Viktig å kalle read() først
                    data_pressure = self.sensor_ms5837.pressure(ms5837.UNITS_mbar)
                    # data_temp = self.sensor_ms5837.temperature(ms5837.UNITS_Centigrade) # Kan hentes hvis nødvendig
                    pressure_error = False
                else:
                    print("[FEIL] MS5837: Kunne ikke lese data (read() feilet).")
            except Exception as e:
                print(f"[FEIL] MS5837: Kunne ikke lese trykk: {e}")
        # else:
            # print("[DEBUG] MS5837: Henting hoppet over (ikke initialisert).")


        return {
            "LFR": {"value": data_lfr, "error": lfr_error}, # LFR = Light/Range -> Range
            "pressure": {"value": data_pressure, "error": pressure_error}
        }

    def get_data_pulse(self):
        """ Henter rå pulsdata (spenning/verdi) fra ADS1015. """
        data_pulse = None
        pulse_error = True

        if self.pulse_ok and self.puls_channel: # Sjekker om ADC og kanal er klar
            try:
                # Henter rå spenning. For BPM trengs mer prosessering.
                data_pulse = self.puls_channel.voltage
                # Eller bruk .value for rå ADC-verdi:
                # data_pulse = self.puls_channel.value
                pulse_error = False
            except Exception as e:
                print(f"[FEIL] ADS1015: Kunne ikke lese puls-kanal: {e}")
        # else:
            # print("[DEBUG] Puls: Henting hoppet over (ikke initialisert).")

        return {
            "pulse": {"value": data_pulse, "error": pulse_error}
            }

    def aktivert(self):
        """
        Bestemmer om svømmeren er i vann basert på trykksensoren.
        Returnerer True hvis i vann, False ellers.
        """
        # Hent ferske trykkdata
        pressure_data = self.get_data_LFR_Preasure()["pressure"]

        if not pressure_data["error"] and pressure_data["value"] is not None:
            current_pressure = pressure_data["value"]
            if current_pressure > PRESSURE_THRESHOLD_WATER_MBAR:
                # print(f"[DEBUG] aktivert(): Trykk={current_pressure} mbar > {PRESSURE_THRESHOLD_WATER_MBAR} mbar -> I VANN")
                return True
            else:
                # print(f"[DEBUG] aktivert(): Trykk={current_pressure} mbar <= {PRESSURE_THRESHOLD_WATER_MBAR} mbar -> IKKE I VANN")
                return False
        else:
            print("[INFO] aktivert(): Kan ikke bestemme status (trykkdata utilgjengelig). Antar ikke i vann.")
            return False # Sikkerhetsantakelse: Hvis vi ikke vet, antar vi ikke aktivert

    # --- Status-evalueringsmetoder ---
    # Disse mottar en samlet dictionary med siste data fra alle sensorer
    # Logikken her er KUN et eksempel og MÅ tilpasses/kalibreres!

    def Flyter(self, data):
        """ Sjekker om tilstanden 'Flyter' er oppfylt. """
        aks_status = data.get("aks", {}).get("status", "ukjent")
        aks_roll = data.get("aks", {}).get("roll", 0)
        aks_pitch = data.get("aks", {}).get("pitch", 0)
        pressure = data.get("pressure", {}).get("value")

        # Enkel logikk: Rolig bevegelse ('jevn') og nær overflaten
        if pressure is not None and (PRESSURE_THRESHOLD_WATER_MBAR < pressure < PRESSURE_THRESHOLD_WATER_MBAR + 50): # Nær overflaten
             if aks_status == 'jevn' and abs(aks_roll) < 20 and abs(aks_pitch) < 20: # Lite bevegelse/krengning
                 return True
        return False

    def Svømmer(self, data):
        """ Sjekker om tilstanden 'Svømmer' er oppfylt. """
        aks_status = data.get("aks", {}).get("status", "ukjent")
        pressure = data.get("pressure", {}).get("value")
        pulse = data.get("pulse", {}).get("value") # Råverdi, ikke BPM

         # Enkel logikk: Ujevn bevegelse, nær overflaten, puls til stede (?).
        if pressure is not None and (PRESSURE_THRESHOLD_WATER_MBAR < pressure < PRESSURE_THRESHOLD_WATER_MBAR + 100): # Nær overflaten
             if aks_status == 'ujevn':
                 # Kanskje sjekke puls-signalets amplitude/varians?
                 return True
        return False

    def Dykker(self, data):
        """ Sjekker om tilstanden 'Dykker' er oppfylt. """
        pressure = data.get("pressure", {}).get("value")

        # Enkel logikk: Betydelig dypere enn overflaten
        if pressure is not None and pressure > (PRESSURE_THRESHOLD_WATER_MBAR + 150): # Dypere enn f.eks. 1.5 meter
            return True
        return False

    def Svømmer_opp(self, data):
         """ Sjekker om tilstanden 'Svømmer_opp' er oppfylt. """
         # Trenger historikk for å se om trykket *minsker* over tid.
         # Kan implementeres ved å lagre forrige trykk i self.variabler.
         # Forenklet eksempel uten historikk: Antar 'ujevn' bevegelse på moderat dybde.
         aks_status = data.get("aks", {}).get("status", "ukjent")
         pressure = data.get("pressure", {}).get("value")
         if pressure is not None and (PRESSURE_THRESHOLD_WATER_MBAR + 50 < pressure < PRESSURE_THRESHOLD_WATER_MBAR + 150):
             if aks_status == 'ujevn':
                 return True
         return False


    def Drukner(self, data):
        """ Sjekker om tilstanden 'Drukner' er oppfylt. """
        # VANSKELIG å definere. Krever sannsynligvis analyse over tid.
        # Mulige tegn (veldig forenklet):
        # 1. Ingen bevegelse ('jevn') under vann over lengre tid.
        # 2. Uregelmessig/ingen puls (krever puls-prosessering).
        # 3. Ligger stille med ansiktet ned (krever mer nøyaktig orientering).

        aks_status = data.get("aks", {}).get("status", "ukjent")
        aks_pitch = data.get("aks", {}).get("pitch", 0)
        pressure = data.get("pressure", {}).get("value")

        # Eksempel 1: Ligger stille dypt under vann
        if pressure is not None and pressure > (PRESSURE_THRESHOLD_WATER_MBAR + 100): # Dypere enn 1m
            if aks_status == 'jevn':
                # Kan legge til tidsfaktor: har vært 'jevn' i X sekunder?
                # Kan legge til sjekk på pitch (f.eks. > 60 grader for å ligge på magen?)
                # print("[DEBUG] Mulig drukning: Stille under vann")
                # return True # Vær forsiktig med å aktivere denne uten mer logikk
                pass
        return False # Returnerer False inntil mer robust logikk er på plass


if __name__ == "__main__":
    print("Starter hovedprogram...")
    # Opprett instans av status-klassen (initialiserer sensorer)
    status_monitor = status()
    last_status_check_time = time.time()
    status_update_interval = 1.0 # Sekunder mellom hver status-sjekk

    while True:
        try:
            # 1. Sjekk om personen er i vannet
            ivann = status_monitor.aktivert()

            if ivann:
                # --- Aktiv statusovervåkning ---
                current_time = time.time()

                # Hent data fra alle sensorer (sekvensielt)
                # TODO: Vurder threading her hvis nødvendig for ytelse
                data_aks = status_monitor.get_data_aks()
                data_lfr_pressure = status_monitor.get_data_LFR_Preasure()
                data_puls = status_monitor.get_data_pulse()

                # Samle all data i én dictionary
                current_data = {**data_aks, **data_lfr_pressure, **data_puls}
                # print(f"[DEBUG] Samlet data: {current_data}") # For feilsøking

                # Bare kjør status-sjekk med definert intervall
                if current_time - last_status_check_time >= status_update_interval:
                    last_status_check_time = current_time
                    current_state = "Uvisst status" # Default

                    # Evaluer status basert på data (rekkefølge kan være viktig)
                    # Start med de mest spesifikke eller kritiske statusene
                    if status_monitor.Drukner(current_data): # Vær forsiktig med denne
                        current_state = "DRUKNER (Advarsel!)"
                    elif status_monitor.Dykker(current_data):
                        current_state = "Dykker"
                    elif status_monitor.Svømmer_opp(current_data):
                         current_state = "Svømmer opp"
                    elif status_monitor.Svømmer(current_data):
                        current_state = "Svømmer"
                    elif status_monitor.Flyter(current_data):
                        current_state = "Flyter"
                    # Hvis ingen av de over stemmer, forblir status "Uvisst"

                    print(f"Tid: {time.strftime('%H:%M:%S')} | Status: {current_state}")
                    # Kan legge til logging til fil her hvis ønskelig

            else:
                # --- Ikke i vann ---
                print("Ikke i vann. Venter...")
                # Vent lenger når ikke i vann for å spare ressurser
                time.sleep(5)

            # Generell liten pause for å unngå 100% CPU-bruk (juster etter behov)
            # Hvis statusoppdatering skjer hvert sekund, er denne kanskje unødvendig i 'ivann'-blokken
            if not ivann:
                 time.sleep(1) # Liten pause også i hovedløkken

        except KeyboardInterrupt:
            print("\nAvslutter programmet.")
            break
        except Exception as e:
            print(f"\n[KRITISK FEIL] En uventet feil oppstod i hovedløkken: {e}")
            print("Prøver å fortsette om 5 sekunder...")
            time.sleep(5)