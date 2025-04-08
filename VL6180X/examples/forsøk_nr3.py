# -*- coding: utf-8 -*-
"""
Klassebasert sanntids deteksjon av pustestopp (stabil avstand > 2s)
OG uavhengig estimering av bevegelsesfrekvens basert på DE TO SISTE toppene,
med timeout til 0 BPM ved inaktivitet. Bruker VL6180X.
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

try:
    import adafruit_vl6180x
except ImportError:
    print("Feil: Kunne ikke importere 'adafruit_vl6180x'.")
    print("Sørg for at biblioteket er installert eller tilgjengelig i PYTHONPATH.")
    exit()

# --- Klassen VL6180XAnalyser ---
class VL6180XAnalyser:
    """
    Analyserer VL6180X data: detekterer stabile perioder og estimerer
    frekvens basert på de to siste toppene, med timeout til 0 BPM.
    """

    def __init__(self,
                 # Generelle parametere
                 target_fs: float = 10.0,
                 # Pustestopp-deteksjon
                 flatness_check_window_sec: float = 2.0,
                 flatness_threshold: float = 2.0,
                 apnea_duration_threshold_sec: float =4.0,
                 # Frekvensanalyse
                 freq_analysis_window_sec: float = 10.0,
                 smoothing_window_freq_sec: float = 0.7,
                 polynomial_order_freq: int = 3,
                 peak_min_distance_sec: float = 0.5,
                 peak_min_prominence: float = 5.0,
                 max_reasonable_freq_hz: float = 5.0,
                 # Ny parameter for frekvens-timeout
                 freq_timeout_sec: float = 3.0): # Tid uten gyldig freq før den settes til 0
        """
        Initialiserer analysatoren.
        Args:
            ... (tidligere parametere) ...
            freq_timeout_sec: Sekunder uten gyldig frekvensberegning før BPM går til 0.
        """
        print("Initialiserer VL6180XAnalyser...")
        self.target_fs = target_fs
        if self.target_fs <= 0: raise ValueError("target_fs må være positiv.")

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
        self.freq_timeout_sec = freq_timeout_sec # Ny parameter

        # Beregn bufferstørrelser
        self._calculate_buffer_sizes()

        # Initialiser I2C og Sensor
        self.sensor = None
        try:
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
        self.last_valid_bpm = 0.0 # Starter på 0 istedenfor None
        self.last_freq_calc_time = 0.0 # Tidspunkt for siste gyldige frekvensberegning

        print("Initialisering fullført.")
        # Print parametere for verifikasjon
        print("--- Bruker følgende parametere ---")
        print(f"  target_fs: {self.target_fs} Hz")
        print(f"  flatness_check_window_sec: {self.flatness_check_window_sec} s")
        print(f"  flatness_threshold: {self.flatness_threshold}")
        print(f"  apnea_duration_threshold_sec: {self.apnea_duration_threshold_sec} s")
        print(f"  freq_analysis_window_sec: {self.freq_analysis_window_sec} s")
        print(f"  smoothing_window_freq_sec: {self.smoothing_window_freq_sec} s")
        print(f"  polynomial_order_freq: {self.polynomial_order_freq}")
        print(f"  peak_min_distance_sec: {self.peak_min_distance_sec} s")
        print(f"  peak_min_prominence: {self.peak_min_prominence}")
        print(f"  max_reasonable_freq_hz: {self.max_reasonable_freq_hz} Hz")
        print(f"  freq_timeout_sec: {self.freq_timeout_sec} s")
        print("----------------------------------")


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

    def _calculate_frequency_from_last_two_peaks(self) -> float | None:
        """
        Beregner frekvens KUN basert på de to siste toppene i vinduet.
        Returnerer beregnet BPM eller None.
        """
        if len(self.freq_analysis_buffer) != self.freq_analysis_buffer.maxlen:
            return None

        signal_chunk = np.array(self.freq_analysis_buffer)
        # Smoothing
        smoothed_chunk = []
        if len(signal_chunk) >= self.smoothing_window_freq_points:
            try: smoothed_chunk = savgol_filter(signal_chunk, self.smoothing_window_freq_points, self.polynomial_order_freq)
            except ValueError: smoothed_chunk = signal_chunk
        else: smoothed_chunk = signal_chunk

        # Finn topper
        peaks_indices_in_chunk = []
        try:
            peaks_indices, _ = find_peaks(smoothed_chunk, distance=self.peak_min_distance_points, prominence=self.peak_min_prominence)
            # Krever minst 2 topper for å beregne periode
            if len(peaks_indices) >= 2:
                peaks_indices_in_chunk = peaks_indices
        except Exception: peaks_indices_in_chunk = []

        # Beregn frekvens KUN fra de to siste toppene
        if len(peaks_indices_in_chunk) >= 2:
            # Indeks for siste og nest siste topp
            last_peak_idx = peaks_indices_in_chunk[-1]
            second_last_peak_idx = peaks_indices_in_chunk[-2]

            # Beregn periode
            period_samples = last_peak_idx - second_last_peak_idx
            period_sec = period_samples / self.target_fs

            # Sjekk gyldighet og beregn frekvens
            if period_sec > 1e-6: # Unngå divisjon med 0
                frequency_hz = 1.0 / period_sec
                # Sjekk om frekvensen er innenfor rimelig grense
                if 0 < frequency_hz <= self.max_reasonable_freq_hz:
                    current_bpm = frequency_hz * 60
                    # Returner den beregnede BPM-verdien
                    return current_bpm
                # else: Frekvens utenfor rimelig grense
            # else: Ugyldig periode (0 eller negativ?)
        # else: Ikke nok topper funnet

        # Returner None hvis ingen gyldig frekvens ble beregnet
        return None


    def update(self) -> dict:
        """
        Leser sensor, oppdaterer tilstander, beregner frekvens (med timeout),
        og returnerer status.
        """
        current_time = time.monotonic() # Få tidspunkt for denne oppdateringen

        if self.sensor is None:
             return { # Returner feilstatus hvis sensor mangler
                'range_mm': -1, 'status_code': -99, 'status_ok': False,
                'status_text': "Sensor ikke initialisert", 'apnea_active': self.apnea_active,
                'apnea_started': False, 'apnea_stopped': False,
                'frequency_bpm': self.last_valid_bpm # Returner sist kjente
            }

        # 1. Les sensor
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
            # 2. Oppdater buffere
            self.data_buffer_stddev.append(range_mm)
            self.freq_analysis_buffer.append(range_mm)

            # === Pustestopp Deteksjon ===
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

            # === Frekvensanalyse ===
            # Prøv å beregne frekvens basert på de to siste toppene
            calculated_bpm = self._calculate_frequency_from_last_two_peaks()

            # Oppdater siste gyldige BPM og tidspunkt HVIS beregningen var vellykket
            if calculated_bpm is not None:
                self.last_valid_bpm = calculated_bpm
                self.last_freq_calc_time = current_time # Registrer tidspunkt for gyldig beregning
            else:
                # Ingen gyldig frekvens beregnet DENNE gangen.
                # Sjekk om timeout har skjedd siden SISTE gyldige beregning.
                if current_time - self.last_freq_calc_time > self.freq_timeout_sec:
                    self.last_valid_bpm = 0.0 # Sett frekvens til 0 ved timeout

        # Håndter tilfelle der sensorlesing feilet
        else:
            # Hvis sensor feiler, sjekk også om frekvenstimeout har skjedd
            if current_time - self.last_freq_calc_time > self.freq_timeout_sec:
                self.last_valid_bpm = 0.0


        # Returner status (inkluderer den oppdaterte/timeoutede last_valid_bpm)
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
    """
    print("Starter hovedprogram...")
    try:
        analysator = VL6180XAnalyser(**params)
    except Exception as init_e:
        print(f"Kunne ikke opprette VL6180XAnalyser: {init_e}")
        return

    if analysator.sensor is None:
        print("Avslutter siden sensor ikke kunne initialiseres.")
        return

    last_status_print_time = 0.0
    print_interval_sec = 1.0

    print("\nStarter sanntidsanalyse (Trykk Ctrl+C for å avslutte)...")
    try:
        while True:
            loop_start_time = time.monotonic()

            status_resultat = analysator.update()
            current_time = time.monotonic()

            if not status_resultat['status_ok']:
                if current_time - last_status_print_time > print_interval_sec:
                    print(f"{time.strftime('%H:%M:%S')}: Sensorfeil - {status_resultat['status_text']}")
                    # Skriver også ut sist kjente BPM selv ved sensorfeil (som kan ha blitt 0 pga timeout)
                    freq_str = f"{status_resultat['frequency_bpm']:.1f} BPM" if status_resultat['frequency_bpm'] is not None else "N/A"
                    print(f"{time.strftime('%H:%M:%S')} | Pustestopp: {'UKJENT (Sensorfeil)'} | Frekvens: {freq_str}")
                    last_status_print_time = current_time

            else:
                if status_resultat['apnea_started']:
                    print(f"{time.strftime('%H:%M:%S')}: +++ PUSTESTOPP STARTET +++")
                    last_status_print_time = current_time
                elif status_resultat['apnea_stopped']:
                    print(f"{time.strftime('%H:%M:%S')}: --- PUSTESTOPP AVSLUTTET ---")
                    last_status_print_time = current_time

                if current_time - last_status_print_time > print_interval_sec:
                    apnea_status_str = "AKTIV" if status_resultat['apnea_active'] else "INAKTIV"
                    freq_str = f"{status_resultat['frequency_bpm']:.1f} BPM" if status_resultat['frequency_bpm'] is not None else "N/A"
                    range_str = f"{status_resultat['range_mm']}mm"
                    print(f"{time.strftime('%H:%M:%S')} | Pustestopp: {apnea_status_str} | Frekvens: {freq_str} | Range: {range_str}")
                    last_status_print_time = current_time

            # Kontroller løkkehastighet
            loop_end_time = time.monotonic()
            elapsed_time = loop_end_time - loop_start_time
            sleep_time = (1.0 / analysator.target_fs) - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nAvslutter program...")
    finally:
        print("Analysefunksjon avsluttet.")
    return 

# --- Hovedinngangspunkt ---
if __name__ == "__main__":
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
        "max_reasonable_freq_hz": 5.0,
        "freq_timeout_sec": 3.0 # Ny parameter her også
    }

    kjør_sanntidsanalyse(analyse_parametere)
    print("Hovedprogram ferdig.")