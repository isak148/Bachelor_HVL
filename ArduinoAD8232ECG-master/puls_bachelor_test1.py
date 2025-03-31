import time
import numpy as np
import adafruit_ads1x15.ads1015 as ADS1015
import adafruit_ads1x15.analog_in as AnalogIn
import board
import busio
from scipy.signal import butter, filtfilt, find_peaks
from ecgdetectors import Detectors
import csv
import os 


# Konfigurasjon
sample_rate = 100
duration_sec = 5
samples_per_window = sample_rate * duration_sec
bpm_history = []

# I2C og ADC
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS1015.ADS1015(i2c)
channel = AnalogIn.AnalogIn(adc, ADS1015.P0)

def bandpass_filter(data, lowcut=0.5, highcut=5, fs=100, order=4):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return filtfilt(b, a, data)

def get_adc_value():
    '''
    file_path = "sensor_data_ECG.csv"
    file_exists = os.path.exists(file_path)

    with open(file_path, "a", newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
                writer.writerow(["signal"])
        writer.writerow([channel.value])'
        '''
    return channel.value

def calculate_bpm_time_domain(filtered_data, fs):
    # Finn topper (sl�r inn p� pulsrytme)
    peaks, _ = find_peaks(filtered_data, distance=fs*0.4, height=np.std(filtered_data))
    if len(peaks) < 2:
        return None

    intervals = np.diff(peaks) / fs  # tid mellom topper i sekunder
    avg_interval = np.mean(intervals)
    if avg_interval <= 0:
        return None
    bpm = 60 / avg_interval
    return bpm

def smooth_bpm(bpm, history, window=3):
    history.append(bpm)
    if len(history) > window:
        history.pop(0)
    return np.mean(history)

# Hovedlokke
try:
    print("Starter m�ling (tidsdomene med toppdeteksjon)...")
    while True:
        raw_data = np.zeros(samples_per_window)

        for i in range(samples_per_window):
            raw_data[i] = get_adc_value()
            time.sleep(1 / sample_rate)

        filtered = bandpass_filter(raw_data, lowcut=0.7, highcut=3.0, fs=sample_rate)
        bpm = calculate_bpm_time_domain(filtered, sample_rate)

        if bpm is not None and 40 < bpm < 220:
            smoothed = smooth_bpm(bpm, bpm_history)
            print(f"Puls estimert: {smoothed:.1f} BPM")
        else:
            print("Ingen gyldig puls registrert.")
except KeyboardInterrupt:
    print("Avslutter...")
