# -*- coding: utf-8 -*-
"""
Klassebasert sanntids deteksjon av pustestopp (stabil avstand)
og estimering av bevegelsesfrekvens ved bruk av VL6180X.
Hovedlogikk er flyttet til en egen funksjon for lesbarhet.
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

# Importer det lokale Adafruit VL6180X biblioteket
try:
    import adafruit_vl6180x
except ImportError:
    print("Feil: Kunne ikke importere 'adafruit_vl6180x'.")
    print("Sørg for at biblioteket er installert eller tilgjengelig i PYTHONPATH.")
    exit()

# --- Klassen VL6180XAnalyser (samme som før) ---
class VL6180XAnalyser:
    """
    En klasse for å analysere VL6180X sensordata i sanntid for å
    detektere perioder med stabil avstand (apnea/pustestopp) og
    estimere bevegelsesfrekvens.
    """

    def __init__(self,
        # Generelle parametere
        target_fs: float = 10.0,
        # Parametere for Pustestopp-deteksjon
        flatness_check_window_sec: float = 2.0,
        flatness_threshold: float = 2.0,
        apnea_duration_threshold_sec: float = 2.0,
        # Parametere for Frekvensanalyse
        freq_analysis_window_sec: float = 10.0,
        smoothing_window_freq_sec: float = 0.7,
        polynomial_order_freq: int = 3,
        peak_min_distance_sec: float = 0.5,
        peak_min_prominence: float = 5.0,
        max_reasonable_freq_hz: float = 5.0):
        """
        Initialiserer analysatoren med nødvendige parametere og sensor.
        (Se forrige versjon for detaljert Args beskrivelse)
        """
        print("Initialiserer VL6180XAnalyser...")
        self.target_fs = target_fs
        if self.target_fs <= 0:
            raise ValueError("target_fs må være positiv.")

        # Lagre parametere
        self.flatness_check_window_sec = flatness_check_window_sec
        self.flatness_threshold = flatness_threshold
        self.apnea_duration_threshold_sec = apnea_duration_threshold_sec
        self.freq_analysis_window_sec = freq_analysis_window_sec
        self.smoothing_window_freq_sec = smoothing_window_freq_sec
        self.polynomial_order_freq = polynomial_order_freq
        self.peak_min_distance_sec = peak_min_distance_sec
        self.peak_min_prominence = peak_min_prominence
        self.max_reasonable_freq_hz = max_reasonable_freq_hz

        # Beregn bufferstørrelser
        self._calculate_buffer_sizes()

        # Initialiser I2C og Sensor
        self.sensor = None
        try:
            # Standard I2C-pinner for RPi via Blinka
            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_vl6180x.VL6180X(i2c)
            print("VL6180X sensor initialisert.")
        except Exception as e:
            print(f"KRITISK FEIL: Kunne ikke initialisere I2C eller sensor: {e}")
            print("Fortsetter uten sensor. update() vil returnere feil.")

        # Initialiser databuffere
        self.data_buffer_stddev = deque(maxlen=self.flatness_check_window_points)
        self.flat_state_buffer = deque(maxlen=self.min_apnea_points)
        self.freq_analysis_buffer = deque(maxlen=self.freq_analysis_window_points)

        # Initialiser tilstandsvariabler
        self.apnea_active = False
        self.last_valid_bpm = None

        print("Initialisering fullført.")

    def _calculate_buffer_sizes(self):
        """Beregner interne bufferstørrelser basert på parametere."""
        # Pustestopp
        self.flatness_check_window_points = round(self.flatness_check_window_sec * self.target_fs)
        if self.flatness_check_window_points < 2: self.flatness_check_window_points = 2
        self.min_apnea_points = round(self.apnea_duration_threshold_sec * self.target_fs)
        if self.min_apnea_points < 1: self.min_apnea_points = 1

        # Frekvens
        self.freq_analysis_window_points = round(self.freq_analysis_window_sec * self.target_fs)
        if self.freq_analysis_window_points < 5: self.freq_analysis_window_points = 5
        self.smoothing_window_freq_points = round(self.smoothing_window_freq_sec * self.target_fs)
        if self.smoothing_window_freq_points < 3: self.smoothing_window_freq_points = 3
        if self.smoothing_window_freq_points % 2 == 0: self.smoothing_window_freq_points += 1
        self.peak_min_distance_points = round(self.peak_min_distance_sec * self.target_fs)
        if self.peak_min_distance_points < 1: self.peak_min_distance_points = 1

    def _calculate_frequency(self) -> float | None:
        """
        Intern hjelpemetode for å beregne frekvens fra freq_analysis_buffer.
        Returnerer beregnet BPM eller None hvis ingen gyldig frekvens ble funnet.
        """
        if len(self.freq_analysis_buffer) != self.freq_analysis_buffer.maxlen:
            return None

        signal_chunk = np.array(self.freq_analysis_buffer)
        smoothed_chunk = []
        if len(signal_chunk) >= self.smoothing_window_freq_points:
             try:
                 smoothed_chunk = savgol_filter(signal_chunk, self.smoothing_window_freq_points, self.polynomial_order_freq)
             except ValueError: smoothed_chunk = signal_chunk
        else: smoothed_chunk = signal_chunk

        peaks_indices_in_chunk = []
        try:
            peaks_indices, _ = find_peaks(smoothed_chunk, distance=self.peak_min_distance_points, prominence=self.peak_min_prominence)
            if len(peaks_indices) >= 2: peaks_indices_in_chunk = peaks_indices
        except Exception: peaks_indices_in_chunk = []

        if len(peaks_indices_in_chunk) >= 2:
            periods_samples = np.diff(peaks_indices_in_chunk)
            periods_sec = periods_samples / self.target_fs
            valid_period_mask = periods_sec > 1e-6
            periods_sec = periods_sec[valid_period_mask]

            if len(periods_sec) > 0:
                instant_frequencies_hz = 1.0 / periods_sec
                valid_freq_mask = instant_frequencies_hz <= self.max_reasonable_freq_hz
                valid_frequencies_hz = instant_frequencies_hz[valid_freq_mask]

                if len(valid_frequencies_hz) > 0:
                    mean_freq_hz = np.mean(valid_frequencies_hz)
                    current_window_bpm = mean_freq_hz * 60
                    return current_window_bpm
        
        return None

    def update(self) -> dict:
        """
        Leser en ny verdi fra sensoren, oppdaterer interne buffere og tilstander,
        og returnerer en status-ordbok.
        """
        if self.sensor is None:
             return {
                'range_mm': -1, 'status_code': -99, 'status_ok': False,
                'status_text': "Sensor ikke initialisert", 'apnea_active': self.apnea_active,
                'apnea_started': False, 'apnea_stopped': False,
                'frequency_bpm': self.last_valid_bpm
            }

        range_mm = -1
        status = -1
        status_text = "Ukjent feil"
        try:
            
            range_mm = self.sensor.range
            status = self.sensor.range_status
           
        except Exception as e:
            status = -100
            status_text = f"Sensorlesefeil: {e}"

        status_ok = (status == adafruit_vl6180x.ERROR_NONE)
        apnea_started_now = False
        apnea_stopped_now = False

        if status_ok:
            self.data_buffer_stddev.append(range_mm)
            self.freq_analysis_buffer.append(range_mm)

            # Pustestopp Deteksjon
            is_currently_flat = False
            if len(self.data_buffer_stddev) == self.data_buffer_stddev.maxlen:
                std_dev = np.std(self.data_buffer_stddev)
                is_currently_flat = std_dev < self.flatness_threshold
            self.flat_state_buffer.append(is_currently_flat)

            apnea_detected_now = False
            if len(self.flat_state_buffer) == self.flat_state_buffer.maxlen:
                if all(self.flat_state_buffer): apnea_detected_now = True

            if apnea_detected_now and not self.apnea_active:
                apnea_started_now = True
                self.apnea_active = True
            elif not apnea_detected_now and self.apnea_active:
                apnea_stopped_now = True
                self.apnea_active = False

            # Frekvensanalyse
            calculated_bpm = self._calculate_frequency()
            if calculated_bpm is not None:
                self.last_valid_bpm = calculated_bpm

        # Returner status
        return {
            'range_mm': range_mm, 'status_code': status, 'status_ok': status_ok,
            'status_text': status_text, 'apnea_active': self.apnea_active,
            'apnea_started': apnea_started_now, 'apnea_stopped': apnea_stopped_now,
            'frequency_bpm': self.last_valid_bpm
        }

# --- Funksjon for Hovedlogikk ---
def kjør_sanntidsanalyse(params: dict):
    """
    Initialiserer analysatoren og kjører hovedløkken for sanntidsanalyse.

    Args:
        params: En ordbok med parametere for VL6180XAnalyser.
    """
    print("Starter hovedprogram...")
    try:
        analysator = VL6180XAnalyser(**params)
    except Exception as init_e:
        print(f"Kunne ikke opprette VL6180XAnalyser: {init_e}")
        return # Avslutt funksjonen hvis initialisering feiler

    if analysator.sensor is None:
        print("Avslutter siden sensor ikke kunne initialiseres.")
        return # Avslutt funksjonen

    last_status_print_time = 0.0
    print_interval_sec = 5.0 # Intervall for periodisk statusutskrift

    print("\nStarter sanntidsanalyse (Trykk Ctrl+C for å avslutte)...")
    try:
        while True:
            loop_start_time = time.monotonic()

            # Kall update-metoden
            status_resultat = analysator.update()
            current_time = time.monotonic()

            # Håndter sensorfeil
            if not status_resultat['status_ok']:
                if current_time - last_status_print_time > print_interval_sec:
                    print(f"{time.strftime('%H:%M:%S')}: Sensorfeil - {status_resultat['status_text']}")
                    last_status_print_time = current_time
                # Ikke gå videre med statusutskrift ved feil, men fortsett løkken

            else:
                # Skriv ut ved endringer i pustestopp-status
                if status_resultat['apnea_started']:
                    print(f"{time.strftime('%H:%M:%S')}: +++ PUSTESTOPP STARTET +++")
                    last_status_print_time = current_time
                elif status_resultat['apnea_stopped']:
                    print(f"{time.strftime('%H:%M:%S')}: --- PUSTESTOPP AVSLUTTET ---")
                    last_status_print_time = current_time

                # Periodisk utskrift av generell status
                if current_time - last_status_print_time > print_interval_sec:
                    apnea_status_str = "AKTIV" if status_resultat['apnea_active'] else "INAKTIV"
                    freq_str = f"{status_resultat['frequency_bpm']:.1f} BPM" if status_resultat['frequency_bpm'] is not None else "N/A"
                    # Viser også siste gyldige range-måling for kontekst
                    range_str = f"{status_resultat['range_mm']}mm" if status_resultat['range_mm'] != -1 else "Ukjent"
                    print(f"{time.strftime('%H:%M:%S')} | Pustestopp: {apnea_status_str} | Frekvens: {freq_str}")
                    last_status_print_time = current_time

            # Kontroller løkkehastighet
            loop_end_time = time.monotonic()
            elapsed_time = loop_end_time - loop_start_time
            # Bruk target_fs fra det opprettede analysator-objektet
            sleep_time = (1.0 / analysator.target_fs) - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nAvslutter program...")
    finally:
        print("Analysefunksjon avsluttet.")


# --- Hovedinngangspunkt ---
if __name__ == "__main__":
    # Definer parametere som en ordbok (dictionary)
    analyse_parametere = {
        "target_fs": 10.0,
        "flatness_check_window_sec": 2.0,
        "flatness_threshold": 2.0,
        "apnea_duration_threshold_sec": 2.0,
        "freq_analysis_window_sec": 10.0,
        "smoothing_window_freq_sec": 0.7,
        "polynomial_order_freq": 3,
        "peak_min_distance_sec": 0.5,
        "peak_min_prominence": 5.0,
        "max_reasonable_freq_hz": 5.0
    }

    # Kall hovedfunksjonen med parameterne
    kjør_sanntidsanalyse(analyse_parametere)

    print("Hovedprogram ferdig.")