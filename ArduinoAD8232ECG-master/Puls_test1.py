import time
import adafruit_ads1x15.ads1015 as ADS1015
import adafruit_ads1x15.analog_in as AnalogIn
import board
import busio
import numpy as np
import scipy.signal

# ✅ Set up I2C and ADC
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS1015.ADS1015(i2c)
channel = AnalogIn.AnalogIn(adc, ADS1015.P0)

# ✅ Sampling & Buffer
sample_rate = 50  # ✅ Minimum required for accurate BPM (increase if needed)
buffer_size = sample_rate * 10  # Store only last 10 sec of data
data_buffer = np.zeros(buffer_size)
pulse_history = []  # Stores last 10 BPM values for smoothing

def get_adc_value():
    """ Read ADC and convert to voltage. """
    return (channel.value * 3.3) / 32767  # Convert ADC to voltage

def calculate_pulse(data, sample_rate):
    """ Compute heart rate (BPM) from ADC peaks """
    global pulse_history

    if len(data) < sample_rate:  # Need at least 1 second of data
        return None

    # ✅ Detect peaks (heartbeats)
    peaks, _ = scipy.signal.find_peaks(data, height=np.mean(data), distance=sample_rate * 0.6)

    if len(peaks) > 1:
        # ✅ Compute RR intervals (time between beats)
        rr_intervals = np.diff(peaks) / sample_rate
        mean_rr = np.mean(rr_intervals) if len(rr_intervals) > 0 else None

        if mean_rr and mean_rr > 0:
            bpm = 60 / mean_rr  # ✅ Convert to BPM
            pulse_history.append(bpm)
            if len(pulse_history) > 10:  # Keep last 10 values
                pulse_history.pop(0)
            return np.mean(pulse_history)  # Return smoothed BPM

    return None  # No valid heart rate detected

# ✅ Main Loop: Read ADC & Calculate Pulse
try:
    start_time = time.time()

    for i in range(buffer_size):
        data_buffer[i % buffer_size] = get_adc_value()  # Rolling buffer
        if i > sample_rate:  # ✅ Wait at least 1 sec before calculating BPM
            pulse = calculate_pulse(data_buffer, sample_rate)

        print(f"BPM: {pulse if pulse else 'Measuring...'}")

        time.sleep(1 / sample_rate)  # ✅ Matches sampling rate

except KeyboardInterrupt:
    print("Stopping...")