import numpy as np
import time
import math
from scipy.signal import butter, lfilter
from collections import deque

class MPU6050_Orientation:
    def __init__(self, address, bus=1):
        # Initialisering av sensoren og parametere
        self.sample_rate = 100  # Hz
        self.buffer_size = 1000
        self.tot_G_buffer = deque(maxlen=self.buffer_size)
        self.check_interval = 1.0  # Sekunder mellom hver sjekk
        self.last_check_time = 0.0

        # Terskel for minimum bevegelse (TUNABLE)
        self.min_movement_threshold = 0.01  # Justert for mindre bevegelse

        # Parametere for FFT-analyse
        self.fft_threshold_ratio = 0.1  # Justert for lavere frekvenser
        self.fft_min_significant_freqs = 1
        self.fft_freq_range = (0.05, 1.0)  # Bredere frekvensrange

        # Status for bevegelse
        self.reported_periodicity_status = False
        print(f"MPU6050 initialisert. Buffer størrelse: {self.buffer_size}, Sjekk intervall: {self.check_interval}s")

    def butter_highpass(self, cutoff, fs, order=5):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

    def highpass_filter(self, data, cutoff, fs, order=5):
        data_np = np.array(data)
        if len(data_np) <= order * 3:
            return data_np
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
            accel_data = self.get_accel_data(g=True)
            tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
        except Exception as e:
            print(f"Klarte ikke å hente data: {e}")
            return {'total_G': -1, 'is_periodic': self.reported_periodicity_status, 'retning': "Ukjent"}

        self.tot_G_buffer.append(tot_G)

        # Bestem retning
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
                window_data = list(self.tot_G_buffer)
                cutoff_freq = 0.05  # Hz
                filtered_window = self.highpass_filter(window_data, cutoff=cutoff_freq, fs=self.sample_rate, order=5)

                signal_std_dev = np.std(filtered_window)

                # Sjekk minimum bevegelse
                if signal_std_dev < self.min_movement_threshold:
                    new_status = False
                else:
                    new_status = self.is_periodic(filtered_window)

                if new_status != self.reported_periodicity_status:
                    print(f"*** Status endret til {'JEVN' if new_status else 'UJEVN'} (StdDev: {signal_std_dev:.4f}) ***")
                    self.reported_periodicity_status = new_status

        return {
            'total_G': tot_G,
            'is_periodic': self.reported_periodicity_status,
            'retning': retning,
            'std_dev': np.std(window_data) if len(window_data) > 0 else 0.0
        }

# Hovedprogram
if __name__ == "__main__":
    try:
        mpu = MPU6050_Orientation(0x68, bus=1)
        print("MPU6050 initialisert.")
        print("="*40)
    except Exception as e:
        print(f"Kunne ikke initialisere MPU6050: {e}")
        exit()

    while True:
        try:
            status = mpu.gi_status_aks()
            if status['total_G'] != -1:
                print(f"Total G: {status['total_G']:.2f} | Status: {'Jevn' if status['is_periodic'] else 'Ujevn'} | Retning: {status['retning']}")
            time.sleep(1)
        except KeyboardInterrupt:
            print("\nAvslutter program.")
            break
        except Exception as e:
            print(f"Feil i hovedloopen: {e}")
            time.sleep(1)
