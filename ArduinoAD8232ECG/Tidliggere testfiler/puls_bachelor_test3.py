import time
import numpy as np
import adafruit_ads1x15.ads1015 as ADS1015
import adafruit_ads1x15.analog_in as AnalogIn
import board
import busio
from ecgdetectors import Detectors
import csv
import os

# --- Konfigurasjon ---
SAMPLE_RATE = 100
WINDOW_SEC = 5
SAMPLES = SAMPLE_RATE * WINDOW_SEC
MIN_RR_SEC = 0.27
MIN_RR_SAMPLES = int(MIN_RR_SEC * SAMPLE_RATE)

# Init ADC
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS1015.ADS1015(i2c)
channel = AnalogIn.AnalogIn(adc, ADS1015.P0)

# Init Hamilton-detektor
detectors = Detectors(SAMPLE_RATE)

def read_window(samples):
    data = []
    for _ in range(samples):
        data.append(channel.value)
        time.sleep(1 / SAMPLE_RATE)
    return np.array(data)

def estimate_bpm(signal, fs):
    r_peaks = detectors.hamilton_detector(signal)

    # Filtrer bort R-takker som ligger for tett (RR < 0.27s)
    rr_intervals = []
    for i in range(1, len(r_peaks)):
        rr = r_peaks[i] - r_peaks[i - 1]
        if rr >= MIN_RR_SAMPLES:
            rr_intervals.append(rr / fs)
        else:
            # hopper over hele beregning hvis et intervall er for kort
            return None

    if len(rr_intervals) < 10:
        return None

    avg_rr = np.mean(rr_intervals[:5])
    bpm = 60 / avg_rr
    return bpm

# --- Hovedløkke ---
try:
    print("Starter måling med Hamilton-algoritme (råsignal)...")
    while True:
        raw = read_window(SAMPLES)
        bpm = estimate_bpm(raw, SAMPLE_RATE)

        if bpm:
            print(f"✅ Estimert puls: {bpm:.1f} BPM")
        else:
            print("❌ Ingen gyldig puls (for tett eller for få R-takker).")
except KeyboardInterrupt:
    print("Avslutter måling.")
