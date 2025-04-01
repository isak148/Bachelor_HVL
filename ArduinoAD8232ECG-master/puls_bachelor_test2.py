import time
import numpy as np
import board
import busio
import adafruit_ads1x15.ads1015 as ADS1015
import adafruit_ads1x15.analog_in as AnalogIn
from scipy.signal import find_peaks

# --- Konfigurasjon ---
SAMPLE_RATE = 100             # Hz
DURATION_SEC = 5              # sekunder
TOTAL_SAMPLES = SAMPLE_RATE * DURATION_SEC
MIN_PEAK_HEIGHT = 28000       # justeres etter behov
MIN_RR_SEC = 0.27
MIN_RR_SAMPLES = int(MIN_RR_SEC * SAMPLE_RATE)

# --- Sett opp ADC ---
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS1015.ADS1015(i2c)
channel = AnalogIn.AnalogIn(adc, ADS1015.P0)

# --- Funksjon for presis sampling ---
def read_signal_with_timing(duration_sec, sample_rate):
    total_samples = duration_sec * sample_rate
    signal = []
    start_time = time.time()

    for i in range(total_samples):
        signal.append(channel.value)

        next_sample_time = start_time + (i + 1) / sample_rate
        while time.time() < next_sample_time:
            pass  # busy wait for presis timing

    return np.array(signal)

# --- Funksjon for pulsberegning med scipy find_peaks ---
def estimate_bpm_from_peaks(signal, fs):
    peaks, _ = find_peaks(signal, height=MIN_PEAK_HEIGHT, distance=MIN_RR_SAMPLES)

    if len(peaks) < 2:
        return None

    rr_intervals = np.diff(peaks) / fs  # alle RR-intervaller i vinduet

    # Behold kun RR-intervaller som er lange nok
    valid_rr = rr_intervals[rr_intervals >= MIN_RR_SEC]

    if len(valid_rr) == 0:
        return None

    avg_rr = np.mean(valid_rr)
    bpm = 60 / avg_rr
    return bpm


# --- Hovedløkke ---
try:
    print("Starter måling (100 Hz, scipy peak detection)...")
    while True:
        signal = read_signal_with_timing(DURATION_SEC, SAMPLE_RATE)
        bpm = estimate_bpm_from_peaks(signal, SAMPLE_RATE)

        if bpm:
            print(f"✅ Estimert puls: {bpm:.1f} BPM")
        else:
            print("❌ Ugyldig måling (for få eller for tette topper).")
except KeyboardInterrupt:
    print("Avslutter måling.")
