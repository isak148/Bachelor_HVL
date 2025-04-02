import numpy as np
import time
import math
from scipy.signal import butter, lfilter
from collections import deque
from mpu6050 import mpu6050

class MPU6050_Orientation(mpu6050):
    def __init__(self, address, bus=1):
        super().__init__(address, bus)
        self.set_gyro_range(self.GYRO_RANGE_250DEG)
        self.set_accel_range(self.ACCEL_RANGE_2G)
        self.set_filter_range(self.FILTER_BW_188)

        # --- Buffer og Timing ---
        self.sample_rate = 100  # Hz
        self.buffer_size = 1000  # Antall målinger i det løpende vinduet (tilpass for 10 sekunder med 100 data/s)
        self.tot_G_buffer = deque(maxlen=self.buffer_size)
        self.check_interval = 1.0  # Sekunder mellom hver status-sjekk
        self.last_check_time = 0.0

        # --- Terskel for minimum bevegelse (TUNABLE) ---
        self.min_movement_threshold = 0.01  # Redusert for å fange opp små bevegelser ved svømming

        # --- Parametere for is_periodic (TUNABLE) ---
        self.fft_threshold_ratio = 0.15  # Redusert for lavere frekvenser
        self.fft_min_significant_freqs = 1
        self.fft_freq_range = (0.05, 0.5)  # Fanger lavere frekvenser (typisk for svømming)

        # --- Status ---
        self.reported_periodicity_status = False  # Start som Ujevn

        print(f"Initialisert. Buffer: {self.buffer_size} samples. Sjekkintervall: {self.check_interval} s.")
        print(f"Min Movement Thr: {self.min_movement_threshold:.3f} G (std dev)")
        print(f"FFT Params: Ratio={self.fft_threshold_ratio}, Min Peaks={self.fft_min_significant_freqs}, Range={self.fft_freq_range} Hz")

    def butter_highpass(self, cutoff, fs, order=5):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

    def highpass_filter(self, data, cutoff, fs, order=5):
        data_np = np.array(data)
        if len(data_np) <= order * 3:
            return data_np  # Returner ufiltrert hvis ikke nok data
        b, a = self.butter_highpass(cutoff, fs, order)
        y = lfilter(b, a, data_np)
        return y

    def is_periodic(self, signal):
        N = len(signal)
        if N < 2:
            return False
        fft_values = np.fft.rfft(signal)
        fft_freqs = np.fft.rfftfreq(N, d=1.0/self.sample_rate)
        fft_magnitude = np.abs(fft_values) / N
        min_freq, max_freq = self.fft_freq_range
        freq_indices = np.where((fft_freqs >= min_freq) & (fft_freqs <= max_freq))[0]
        if len(freq_indices) == 0:
            return False
        max_magnitude_in_range = np.max(fft_magnitude[freq_indices]) if len(freq_indices) > 0 else 0
        if max_magnitude_in_range == 0:
            return False
        threshold = self.fft_threshold_ratio * max_magnitude_in_range
        significant_freqs_count = np.sum(fft_magnitude[freq_indices] > threshold)
        return significant_freqs_count >= self.fft_min_significant_freqs

    def gi_status_aks(self):
        try:
            # Hent akselerometerdata
            accel_data = self.get_accel_data(g=True)
            tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
        except Exception as e:
            print(f"Klarte ikke å hente data fra MPU6050: {e}")
            return {'total_G': -1, 'is_periodic': self.reported_periodicity_status, 'retning': "Ukjent"}

        # Legg til nyeste data i bufferen
        self.tot_G_buffer.append(tot_G)

        # Bestem retning basert på Z-aksen
        z_val = accel_data['z']
        if z_val <= -0.25:
            retning = "Ned"
        elif z_val >= 0.1:
            retning = "Opp"
        else:
            retning = "Plan"

        # Tidsstyrt sjekk
        current_time = time.time()
        time_since_last_check = current_time - self.last_check_time

        # Oppdater status kun hvis det er på tide OG bufferen er full
        if time_since_last_check >= self.check_interval:
            self.last_check_time = current_time

            if len(self.tot_G_buffer) == self.buffer_size:
                window_data = list(self.tot_G_buffer)  # Bruker `window_data` her
                cutoff_freq = 0.05  # Hz
                filtered_window = self.highpass_filter(window_data, cutoff=cutoff_freq, fs=self.sample_rate, order=5)

                signal_std_dev = np.std(filtered_window)

                # Sjekk minimum bevegelse
                if signal_std_dev < self.min_movement_threshold:
                    new_status = False  # Ujevn (flyting)
                else:
                    new_status = self.is_periodic(filtered_window)  # Jevn (svømming)

                # Oppdater den rapporterte statusen hvis den har endret seg
                if new_status != self.reported_periodicity_status:
                    print(f"*** Status endret til {'JEVN' if new_status else 'UJEVN'} (StdDev: {signal_std_dev:.4f}) ***")
                    self.reported_periodicity_status = new_status

        return {
            'total_G': tot_G,
            'is_periodic': self.reported_periodicity_status,
            'retning': retning,
            'std_dev': signal_std_dev if signal_std_dev is not None else 0.0  # Returner siste beregnede std dev
        }


# --- Hovedprogram ---
if __name__ == "__main__":
    try:
        mpu = MPU6050_Orientation(0x68, bus=1)
        print("MPU6050 initialisert.")
        print("="*40)
    except Exception as e:
        print(f"Kunne ikke initialisere MPU6050: {e}")
        exit()

    last_print_time = time.time()
    print_interval = 0.2  # Oppdater skjermen oftere enn sjekk-intervallet

    while True:
        try:
            start_time = time.time()
            status = mpu.gi_status_aks()  # Denne kjører nå potensielt sjekken internt

            current_time = time.time()
            # Print status jevnlig for å se siste status og data
            if status['total_G'] != -1 and (current_time - last_print_time >= print_interval):
                g_str = f"{status['total_G']:.2f}"
                period_str = "Jevn" if status['is_periodic'] else "Ujevn"
                retning_str = status['retning']
                # Vis std_dev fra *siste sjekk* (kan være opptil 1 sek gammel)
                std_dev_str = f"{status.get('std_dev', 0.0):.4f}"
                # Vis også hvor lenge siden siste sjekk (ca)
                time_since_check_str = f"{current_time - mpu.last_check_time:.2f}"

                print(f"G: {g_str:<5} | Status: {period_str:<6} | Retning: {retning_str:<5} | StdDev(last): {std_dev_str:<7} | SinceChk: {time_since_check_str:<4}s ", end='\r')
                last_print_time = current_time

            # Sov for å opprettholde sample rate
            loop_duration = time.time() - start_time
            sleep_time = (1.0 / mpu.sample_rate) - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nAvslutter program.")
            break
        except Exception as e:
            print(f"En feil oppstod i hovedloopen: {e}")
            time.sleep(1)
