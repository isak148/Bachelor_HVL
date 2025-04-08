# -*- coding: utf-8 -*-
"""
Klassebasert sanntidsanalyse av VL6180X sensordata for å detektere
pustestopp (stabil avstand) og estimere pustefrekvens.
"""

import time
import board
import busio
import numpy as np
from collections import deque

try:
    # Forsøker relativ import siden filen ligger i examples/
    # Juster denne stien om nødvendig
    from .. import adafruit_vl6180x
except (ImportError, ValueError):
    # Fallback til standard import hvis relativ feiler
    try:
        import adafruit_vl6180x
        print("Advarsel: Bruker standard import for adafruit_vl6180x.")
    except ImportError:
        print("KRITISK FEIL: Kunne ikke importere adafruit_vl6180x.")
        print("Sørg for at biblioteket er installert og tilgjengelig.")
        exit()

try:
    from scipy.signal import find_peaks, savgol_filter
except ImportError:
    print("Feil: Kunne ikke importere 'scipy.signal'.")
    print("Installer SciPy med: pip install scipy")
    # Vurder å avslutte hvis SciPy er essensielt
    # exit()
    find_peaks = None # Sett til None hvis SciPy mangler
    savgol_filter = None
    print("Advarsel: SciPy ikke funnet. Frekvensanalyse vil være deaktivert.")


class AnalyseVL6180X:
    """
    Analyserer VL6180X data for pustestopp og frekvens.
    """
    def __init__(self,
                 i2c,
                 address=0x29,
                 # Generelle parametere
                 target_fs: float = 10.0, # Omtrentlig samplingfrekvens (Hz)
                 # Parametere for Pustestopp-deteksjon
                 flatness_check_window_sec: float = 2.0, # Vindu for stabilitetssjekk (s)
                 flatness_threshold: float = 2.0,      # Terskel for std dev (mm) for "flat"
                 apnea_duration_threshold_sec: float = 2.0, # Min varighet av flathet for pustestopp (s)
                 # Parametere for Frekvensanalyse
                 freq_analysis_window_sec: float = 10.0, # Vindu for frekvensanalyse (s)
                 smoothing_window_freq_sec: float = 0.7, # Vindu for Savitzky-Golay filter (s)
                 polynomial_order_freq: int = 3,       # Polynomorden for Savitzky-Golay
                 peak_min_distance_sec: float = 0.5,     # Min tid mellom topper (s)
                 peak_min_prominence: float = 5.0,       # Min prominens for en topp (mm)
                 max_reasonable_freq_hz: float = 2.0):   # Maks fornuftig puste-freq (Hz)
        """
        Initialiserer sensor og setter opp analyseparametere.

        Args:
            i2c: I2C bus-objektet (fra board/busio).
            address: I2C-adressen til sensoren.
            target_fs: Mål-samplingfrekvens for løkken (best effort).
            flatness_check_window_sec: Tidsvindu (s) for å sjekke stabilitet (std dev).
            flatness_threshold: Maksimal standardavvik (mm) for at signalet regnes som flatt.
            apnea_duration_threshold_sec: Hvor lenge signalet må være flatt (s) for å trigge pustestopp.
            freq_analysis_window_sec: Tidsvindu (s) for å beregne frekvens.
            smoothing_window_freq_sec: Bredden (s) på glattevinduet (Savitzky-Golay).
            polynomial_order_freq: Orden på polynomet for glatting.
            peak_min_distance_sec: Minimum tid (s) mellom detekterte pustetopper.
            peak_min_prominence: Minimum "høyde" (mm) en topp må ha relativt til omgivelsene.
            max_reasonable_freq_hz: Øvre grense for hva som regnes som gyldig pustefrekvens (Hz).
        """
        print("Initialiserer AnalyseVL6180X...")
        # --- Sensor Initialisering ---
        self.sensor = None
        try:
            self.sensor = adafruit_vl6180x.VL6180X(i2c, address)
            print(f"VL6180X sensor funnet på adresse {hex(address)}.")
            # Start i kontinuerlig modus for raskere lesing (kan justeres)
            # self.sensor.start_range_continuous(int(1000 / target_fs)) # periode i ms
        except Exception as e:
            print(f"KRITISK FEIL: Kunne ikke initialisere sensor på {hex(address)}: {e}")
            # Klassen kan fortsatt instansieres, men update vil feile.

        # --- Parametere & Bufferstørrelser ---
        if target_fs <= 0:
            raise ValueError("target_fs må være positiv.")
        self.target_fs = target_fs
        self.flatness_threshold = flatness_threshold
        self.peak_min_prominence = peak_min_prominence
        self.max_reasonable_freq_hz = max_reasonable_freq_hz
        self.scipy_available = find_peaks is not None and savgol_filter is not None

        # Pustestopp buffere
        self.flatness_check_window_points = self._seconds_to_points(flatness_check_window_sec, min_val=2)
        self.min_apnea_points = self._seconds_to_points(apnea_duration_threshold_sec, min_val=1)
        self.data_buffer_stddev = deque(maxlen=self.flatness_check_window_points) # For std dev beregning
        self.flat_state_buffer = deque(maxlen=self.min_apnea_points) # For å sjekke varighet av flathet

        # Frekvensanalyse buffere
        self.freq_analysis_window_points = self._seconds_to_points(freq_analysis_window_sec, min_val=5)
        self.smoothing_window_freq_points = self._seconds_to_points(smoothing_window_freq_sec, min_val=3, ensure_odd=True)
        self.polynomial_order_freq = polynomial_order_freq
        self.peak_min_distance_points = self._seconds_to_points(peak_min_distance_sec, min_val=1)
        # Lagrer (tidspunkt, verdi) for frekvensanalyse
        self.freq_analysis_buffer = deque(maxlen=self.freq_analysis_window_points)

        # --- Tilstandsvariabler ---
        self.apnea_active = False # Er pustestopp aktiv nå?
        self.last_calculated_bpm = None # Sist beregnede frekvens (BPM)
        self.last_update_time = time.monotonic()

        print("Initialisering fullført.")
        print(f"  Pustestopp sjekk: {self.flatness_check_window_points} punkter ({flatness_check_window_sec}s)")
        print(f"  Pustestopp varighet: {self.min_apnea_points} punkter ({apnea_duration_threshold_sec}s)")
        print(f"  Frekvensanalyse: {self.freq_analysis_window_points} punkter ({freq_analysis_window_sec}s)")
        if self.scipy_available:
             print(f"  Glatting: {self.smoothing_window_freq_points} punkter, orden {self.polynomial_order_freq}")
             print(f"  Toppavstand: {self.peak_min_distance_points} punkter ({peak_min_distance_sec}s)")
        else:
             print("  SciPy mangler - Frekvensanalyse deaktivert.")

    def _seconds_to_points(self, seconds: float, min_val: int = 1, ensure_odd: bool = False) -> int:
        """Konverterer sekunder til antall punkter basert på target_fs."""
        points = max(min_val, round(seconds * self.target_fs))
        if ensure_odd and points % 2 == 0:
            points += 1
        return points

    def update(self) -> dict:
        """
        Leser sensoren, oppdaterer interne buffere og tilstander,
        utfører analyser (pustestopp, frekvens) og returnerer status.
        Bør kalles jevnlig (ca. med target_fs intervall).
        """
        current_time = time.monotonic()
        dt = current_time - self.last_update_time # Tidssteg siden sist
        self.last_update_time = current_time

        # --- 1. Les sensor ---
        range_mm = -1
        status_code = -99 # Indikerer sensorfeil internt
        status_ok = False
        status_text = "Sensor ikke initialisert"

        if self.sensor:
            try:
                range_mm = self.sensor.range
                status_code = self.sensor.range_status
                status_ok = (status_code == adafruit_vl6180x.ERROR_NONE)
                if not status_ok:
                    status_text = f"Sensorfeil: {status_code}"
                else:
                    status_text = "OK"

            except Exception as e:
                status_ok = False
                status_text = f"Sensorlesefeil: {e}"
                status_code = -100 # Indikerer lesefeil

        apnea_now = False
        apnea_started_this_update = False
        apnea_stopped_this_update = False
        calculated_bpm_this_update = None

        # --- 2. Behandle data HVIS lesing var OK ---
        if status_ok:
            # Legg til i buffere
            self.data_buffer_stddev.append(range_mm)
            self.freq_analysis_buffer.append((current_time, range_mm))

            # --- 3. Pustestopp-analyse ---
            apnea_now = self._check_pustestopp()
            if apnea_now and not self.apnea_active:
                apnea_started_this_update = True
                self.apnea_active = True
            elif not apnea_now and self.apnea_active:
                apnea_stopped_this_update = True
                self.apnea_active = False
            # Ellers forblir self.apnea_active som den var

            # --- 4. Frekvensberegning ---
            # Beregn kun hvis IKKE i pustestopp (eller når den nettopp stoppet?)
            # Kan justeres - her beregner vi ikke frekvens under aktiv pustestopp.
            if not self.apnea_active:
                 calculated_bpm_this_update = self._calculate_frequency()
                 if calculated_bpm_this_update is not None:
                     self.last_calculated_bpm = calculated_bpm_this_update
            # Hvis i pustestopp, behold siste gyldige BPM (eller sett til 0?)
            # Her beholder vi siste gyldige. Frekvensen "fryser" under stopp.

        # Hvis lesing feilet, reset apnea status? For nå: behold status.
        # TODO: Vurder timeout på frekvens hvis ingen topper detekteres over tid.

        # --- 5. Returner status ---
        return {
            'timestamp': current_time,
            'dt': dt,
            'range_mm': range_mm,
            'status_code': status_code,
            'status_ok': status_ok,
            'status_text': status_text,
            'apnea_active': self.apnea_active,
            'apnea_started': apnea_started_this_update,
            'apnea_stopped': apnea_stopped_this_update,
            'frequency_bpm': self.last_calculated_bpm # Returner sist kjente
        }


    def _check_pustestopp(self) -> bool:
        """
        Sjekker om pustestopp-kriteriene er møtt basert på
        data_buffer_stddev og flat_state_buffer.
        Returnerer True hvis pustestopp er aktivt NÅ, ellers False.
        """
        # Sjekk om nok data for standardavvik
        if len(self.data_buffer_stddev) < self.data_buffer_stddev.maxlen:
            self.flat_state_buffer.append(False) # Ikke flatt hvis ikke nok data
            return False # Kan ikke avgjøre ennå

        # Beregn standardavvik
        std_dev = np.std(self.data_buffer_stddev)
        is_currently_flat = std_dev < self.flatness_threshold

        # Oppdater buffer for flat-tilstand
        self.flat_state_buffer.append(is_currently_flat)

        # Sjekk om nok data i flat-tilstand bufferet
        if len(self.flat_state_buffer) < self.flat_state_buffer.maxlen:
            return False # Kan ikke avgjøre varighet ennå

        # Sjekk om ALLE i flat-tilstand bufferet er True
        if all(self.flat_state_buffer):
            return True # Ja, pustestopp er aktivt nå
        else:
            return False


    def _calculate_frequency(self) -> float | None:
        """
        Beregner pustefrekvens basert på data i freq_analysis_buffer.
        Bruker Savitzky-Golay filter og peak detection.
        Returnerer frekvens i BPM (Breaths Per Minute) eller None hvis
        ikke nok data eller ingen gyldige topper funnet.
        """
        if not self.scipy_available:
            return None # Kan ikke beregne uten SciPy

        # Sjekk om nok data for analyse
        if len(self.freq_analysis_buffer) < self.freq_analysis_buffer.maxlen:
            return None

        # Hent ut data og tidspunkter
        times = np.array([item[0] for item in self.freq_analysis_buffer])
        signal = np.array([item[1] for item in self.freq_analysis_buffer])

        # --- Glatting (Smoothing) ---
        smoothed_signal = signal # Fallback hvis filter feiler
        if len(signal) >= self.smoothing_window_freq_points:
            try:
                # Bruker mode='interp' for å håndtere kantene bedre, men krever SciPy >= 0.14
                smoothed_signal = savgol_filter(signal,
                                                self.smoothing_window_freq_points,
                                                self.polynomial_order_freq)
            except ValueError as e:
                # Kan skje hvis vindu > data, selv om vi sjekket lengde
                print(f"Advarsel: savgol_filter feilet: {e}")
                smoothed_signal = signal # Bruk originalsignalet
        else:
             # Skal i teorien ikke skje pga. sjekk over, men for sikkerhets skyld
             smoothed_signal = signal

        # --- Toppdeteksjon (Peak Finding) ---
        try:
            peaks_indices, properties = find_peaks(smoothed_signal,
                                                 distance=self.peak_min_distance_points,
                                                 prominence=self.peak_min_prominence)
        except Exception as e:
             print(f"Advarsel: find_peaks feilet: {e}")
             return None # Kan ikke fortsette uten topper

        # --- Frekvensberegning ---
        if len(peaks_indices) < 2:
            return None # Trenger minst to topper for å beregne en periode

        # Få tidspunktene for toppene
        peak_times = times[peaks_indices]

        # Beregn periodene (tid mellom påfølgende topper)
        periods = np.diff(peak_times)

        # Filtrer bort ugyldige perioder (f.eks. 0)
        valid_periods = periods[periods > 1e-6] # Liten toleranse for flyttall

        if len(valid_periods) == 0:
            return None

        # Beregn øyeblikksfrekvenser (Hz)
        instant_freqs_hz = 1.0 / valid_periods

        # Filtrer bort urimelig høye frekvenser
        valid_freqs_hz = instant_freqs_hz[instant_freqs_hz <= self.max_reasonable_freq_hz]

        if len(valid_freqs_hz) == 0:
            return None

        # Beregn gjennomsnittsfrekvens over vinduet
        mean_freq_hz = np.mean(valid_freqs_hz)

        # Konverter til BPM
        mean_bpm = mean_freq_hz * 60.0

        return mean_bpm


# --- Eksempel på bruk ---
if __name__ == "__main__":
    print("Starter eksempel på bruk av AnalyseVL6180X...")

    # Sett opp I2C
    try:
        i2c_bus = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        print(f"Feil ved oppsett av I2C: {e}")
        print("Sjekk tilkoblinger og at I2C er aktivert på systemet.")
        exit()

    # Lag en instans av analysatoren med standard parametere
    # Juster parametere her om nødvendig
    analyser_params = {
        "target_fs": 10.0,
        "flatness_check_window_sec": 2.0,
        "flatness_threshold": 2.0,
        "apnea_duration_threshold_sec": 2.0,
        "freq_analysis_window_sec": 10.0,
        "smoothing_window_freq_sec": 0.7,
        "polynomial_order_freq": 3,
        "peak_min_distance_sec": 0.5,
        "peak_min_prominence": 5.0,
        "max_reasonable_freq_hz": 2.0 # ~120 BPM maks
    }
    try:
        analysator = AnalyseVL6180X(i2c_bus, **analyser_params)
    except Exception as e:
        print(f"Feil under instansiering av AnalyseVL6180X: {e}")
        exit()

    if analysator.sensor is None:
         print("Avslutter siden sensor ikke ble initialisert.")
         exit()


    print("\nStarter sanntidsanalyse (Trykk Ctrl+C for å avslutte)...")
    last_print_time = 0.0
    print_interval = 2.0 # Sekunder mellom statusutskrifter

    try:
        while True:
            loop_start = time.monotonic()

            # Kall update-metoden for å få siste status
            status = analysator.update()
            now = status['timestamp'] # Bruk timestamp fra status

            # Skriv ut umiddelbart ved endring i pustestopp
            if status['apnea_started']:
                print(f"{time.strftime('%H:%M:%S')}: +++ PUSTESTOPP STARTET +++ (Range: {status['range_mm']}mm)")
                last_print_time = now
            elif status['apnea_stopped']:
                print(f"{time.strftime('%H:%M:%S')}: --- PUSTESTOPP AVSLUTTET ---")
                last_print_time = now

            # Periodisk utskrift av status
            if now - last_print_time >= print_interval:
                apnea_str = "AKTIV" if status['apnea_active'] else "INAKTIV"
                freq_str = f"{status['frequency_bpm']:.1f}" if status['frequency_bpm'] is not None else "N/A"
                range_str = f"{status['range_mm']}" if status['status_ok'] else "FEIL"

                print(f"{time.strftime('%H:%M:%S')} | Status: {status['status_text']:<15} | Range: {range_str:>4}mm | P-stopp: {apnea_str:<7} | Freq: {freq_str:>5} BPM")
                last_print_time = now

            # Håndter sensorfeil separat (kan logges oftere om ønskelig)
            if not status['status_ok'] and status['status_code'] != -99: # Ignorer "ikke initialisert" feilen her
                 # Skriv ut feil sjeldnere?
                 if now - last_print_time >= print_interval * 2 : # F.eks. hvert 4. sekund
                    print(f"{time.strftime('%H:%M:%S')} | ! SENSORFEIL: {status['status_text']} (Kode: {status['status_code']}) !")
                    last_print_time = now


            # --- Kontroller løkkehastighet (Best Effort) ---
            loop_end = time.monotonic()
            loop_duration = loop_end - loop_start
            target_interval = 1.0 / analysator.target_fs
            sleep_time = target_interval - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)
            #else:
            #    print(f"Advarsel: Løkken brukte for lang tid ({loop_duration:.3f}s > {target_interval:.3f}s)")


    except KeyboardInterrupt:
        print("\nAvslutter program...")
    except Exception as e:
        print(f"\nUventet feil: {e}")
    finally:
        # Rydd opp? Sensoren stopper vanligvis ikke kontinuerlig modus selv.
        # if analysator.sensor and hasattr(analysator.sensor, 'stop_range_continuous'):
        #     try:
        #         analysator.sensor.stop_range_continuous()
        #         print("Stoppet kontinuerlig modus.")
        #     except Exception as stop_e:
        #         print(f"Feil ved stopping av sensor: {stop_e}")
        print("Program avsluttet.")