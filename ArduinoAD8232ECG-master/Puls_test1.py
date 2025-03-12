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

# ✅ Sampling Configuration
sample_rate = 100  # ✅ Higher sample rate for real-time response
buffer_size = sample_rate * 3  # ✅ Rolling buffer (last 3 sec)
data_buffer = np.zeros(buffer_size)
pulse_history = []  # Stores recent BPM values

def get_adc_value():
    """ Read ADC and convert to voltage. """
    return (channel.value * 3.3) / 32767  # Convert ADC to voltage

def calculate_pulse(data, sample_rate):
    """ Compute continuous heart rate (BPM) with real-time updates """
    global pulse_history

    if len(data) < sample_rate:  # Need at least 1 sec of data
        return None

    # ✅ Dynamic threshold: Adjusts based on signal variations
    dynamic_threshold = np.mean(data) + 0.5 * np.std(data)
    
    # ✅ Detect peaks (heartbeats)
    peaks, _ = scipy.signal.find_peaks(data, height=dynamic_threshold, distance=sample_rate * 0.5)

    if len(peaks) > 1:
        # ✅ Compute RR intervals (time between beats)
        rr_intervals = np.diff(peaks) / sample_rate
        mean_rr = np.mean(rr_intervals) if len(rr_intervals) > 0 else None

        if mean_rr and mean_rr > 0:
            bpm = 60 / mean_rr  # ✅ Convert to BPM
            pulse_history.append(bpm)
            if len(pulse_history) > 3:  # ✅ Short rolling history for fast updates
                pulse_history.pop(0)
            return np.mean(pulse_history)  # ✅ Return updated BPM

    return None  # No valid heart rate detected

# ✅ Continuous Heart Rate Loop
try:
    start_time = time.time()

    while True:
        # Shift data to keep buffer size constant
        data_buffer[:-1] = data_buffer[1:]
        data_buffer[-1] = get_adc_value()  # Read new ADC value

        if len(data_buffer) > sample_rate:  # ✅ Wait at least 1 sec before calculating BPM
            pulse = calculate_pulse(data_buffer, sample_rate)
        
        if pulse:
            print(f"Real-Time BPM: {pulse:.1f}")
        else:
            print("Measuring...")

        time.sleep(1 / sample_rate)  # ✅ Matches sampling rate

except KeyboardInterrupt:
    print("Stopping continuous measurement...")
