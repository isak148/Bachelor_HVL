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
sample_rate = 100  # ✅ 100 Hz for better accuracy
buffer_size = sample_rate * 5  # ✅ Rolling buffer (last 5 sec)
data_buffer = np.zeros(buffer_size)
pulse_history = []  # Stores recent BPM values for smoothing

def get_adc_value():
    """ Read ADC and convert to voltage. Ignore zero values. """
    value = (channel.value * 3.3) / 32767  # Convert ADC to voltage
    return value if value > 0 else None  # ✅ Ignore `0` values

def calculate_pulse(data, sample_rate):
    """ Compute heart rate (BPM) with real-time updates and smoothing """
    global pulse_history

    # ✅ Remove zero values
    filtered_data = data[data > 0]
    if len(filtered_data) < sample_rate:  # Need at least 1 sec of valid data
        return None

    # ✅ Dynamic threshold for adaptive peak detection
    dynamic_threshold = np.mean(filtered_data) + 0.5 * np.std(filtered_data)

    # ✅ Detect peaks (heartbeats)
    peaks, _ = scipy.signal.find_peaks(filtered_data, height=dynamic_threshold, distance=sample_rate * 0.5)

    if len(peaks) > 1:
        # ✅ Compute RR intervals (time between beats)
        rr_intervals = np.diff(peaks) / sample_rate
        mean_rr = np.mean(rr_intervals) if len(rr_intervals) > 0 else None

        if mean_rr and mean_rr > 0:
            bpm = 60 / mean_rr  # ✅ Convert to BPM

            # ✅ Apply weighted smoothing (70% new, 30% previous)
            if pulse_history:
                bpm = 0.7 * bpm + 0.3 * np.mean(pulse_history)  

            pulse_history.append(bpm)
            if len(pulse_history) > 5:  # ✅ Keep last 5 values for stability
                pulse_history.pop(0)
            return np.mean(pulse_history)  # ✅ Return smoother BPM

    return None  # No valid heart rate detected

# ✅ Continuous Heart Rate Loop (Prints Every 1 Second)
try:
    start_time = time.time()
    last_print_time = start_time

    while True:
        # Get new ADC value
        new_value = get_adc_value()
        if new_value:  # ✅ Only add non-zero values
            data_buffer[:-1] = data_buffer[1:]
            data_buffer[-1] = new_value  

        if len(data_buffer) > sample_rate:  # ✅ Wait at least 1 sec before calculating BPM
            pulse = calculate_pulse(data_buffer, sample_rate)
        
        # ✅ Print pulse every 1 second for accuracy
        if time.time() - last_print_time >= 1:
            last_print_time = time.time()
            if pulse:
                print(f"Stable BPM: {pulse:.1f}")
            else:
                print("Measuring...")

        time.sleep(1 / sample_rate)  # ✅ Matches sampling rate

except KeyboardInterrupt:
    print("Stopping continuous measurement...")
