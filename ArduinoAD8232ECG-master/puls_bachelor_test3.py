import time
import numpy as np
import adafruit_ads1x15.ads1015 as ADS1015
import adafruit_ads1x15.analog_in as AnalogIn
import board
import busio
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

# Initier Hamilton-detektor
detectors = Detectors(sample_rate)

def get_adc_value():
    return channel.value

def calculate_bpm_hamilton(signal, fs):
    r_peaks = detectors.hamilton_detector(signal)
    
    # Filtrer ut topper som ligger for nært hverandre (< 0.27 s)
    rr_intervals = []
    for i in range(1, len(r_peaks)):
        rr = (r_peaks[i] - r_peaks[i - 1]) / fs
        if rr >= 0.27:
            rr_intervals.append(rr)

    if len(rr_intervals) < 5:
        return None

    # Bruk gjennomsnitt av de 5 første gyldige intervallene
    avg_rr = np.mean(rr_intervals[:5])
    bpm = 60 / avg_rr
    return bpm

def smooth_bpm(bpm, history, window=3):
    history.append(bpm)
    if len(history) > window:
        history.pop(0)
    return np.mean(history)

# Hovedløkke
try:
    print("Starter måling (Hamilton-algoritme på råsignal)...")
    while True:
        raw_data = np.zeros(samples_per_window)

        for i in range(samples_per_window):
            raw_data[i] = get_adc_value()
            time.sleep(1 / sample_rate)

        bpm = calculate_bpm_hamilton(raw_data, sample_rate)

        if bpm is not None and 40 < bpm < 220:
            smoothed = smooth_bpm(bpm, bpm_history)
            print(f"Puls estimert: {smoothed:.1f} BPM")
        else:
            print("Ingen gyldig puls registrert.")
except KeyboardInterrupt:
    print("Avslutter...")
