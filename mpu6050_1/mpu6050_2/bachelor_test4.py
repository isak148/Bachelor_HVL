# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd # For efficient rolling window calculations
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt

# --- Configuration ---
fs = 100  # Sampling frequency in Hz (adjust as needed)
low_cutoff_freq = 0.5  # Low-pass filter cutoff frequency in Hz
order = 4  # Filter order
window_size = 100  # Window size for standard deviation (samples)
threshold_multiplier = 1.5 # Multiplier for median std dev to find threshold
exclude_end_samples = 100 # Number of samples to ignore at the very end

# --- Step 1: Get Input Data ---
# In a real application, 'signal_data' would come from your MPU6050 readings
# (e.g., tot_G_buffer, x_data, etc., converted to a NumPy array).
# For demonstration, let's create some sample data:
print("Generating sample data for demonstration...")
duration = 20 # seconds
num_samples = duration * fs
t = np.linspace(0, duration, num_samples, endpoint=False)
# Combine a slow sine wave, some stable periods, and noise
signal_data = (
    0.5 * np.sin(2 * np.pi * 0.2 * t) # Slow oscillation
    + np.random.randn(num_samples) * 0.1 # Noise
)
# Add stable periods (e.g., between t=5-8s and t=12-15s)
stable_mask1 = (t >= 5) & (t < 8)
stable_mask2 = (t >= 12) & (t < 15)
signal_data[stable_mask1] = np.random.randn(np.sum(stable_mask1)) * 0.01 # Very low noise
signal_data[stable_mask2] = 0.05 + np.random.randn(np.sum(stable_mask2)) * 0.01 # Low noise around a small offset
print(f"Generated {len(signal_data)} samples.")

# Ensure data is a NumPy array
signal_data = np.array(signal_data)

if len(signal_data) < window_size + exclude_end_samples:
    print(f"Error: Signal length ({len(signal_data)}) is too short for the chosen window size ({window_size}) and end exclusion ({exclude_end_samples}).")
    exit()

# --- Step 2: Apply Low-pass Filter ---
print(f"Applying low-pass filter (cutoff: {low_cutoff_freq} Hz)...")
nyquist = 0.5 * fs
low = low_cutoff_freq / nyquist
b, a = butter(order, low, btype='low', analog=False)
filtered_signal = filtfilt(b, a, signal_data)
print("Filtering complete.")

# --- Step 3: Calculate Rolling Standard Deviation ---
# Using Pandas for efficient rolling calculation
print(f"Calculating rolling standard deviation (window: {window_size} samples)...")
s = pd.Series(filtered_signal)
# The result index corresponds to the *end* of the window
rolling_std_series = s.rolling(window=window_size, center=False).std()
print("Rolling standard deviation calculated.")

# --- Step 4: Identify Stable Points ---
# Calculate threshold based on the median of the non-NaN std values
valid_rolling_std = rolling_std_series.dropna() # Remove NaNs from the start
if len(valid_rolling_std) == 0:
    print("Error: Could not calculate rolling standard deviation (perhaps window size is too large?).")
    exit()

threshold = valid_rolling_std.median() * threshold_multiplier
print(f"Stability threshold calculated: {threshold:.4f} (based on median {valid_rolling_std.median():.4f} * {threshold_multiplier})")

# Find points where rolling std is below the threshold
# Boolean mask aligns with the original filtered_signal length
stable_mask = rolling_std_series < threshold # This mask includes NaNs which evaluate to False

# Get the indices where the mask is True
stable_indices_raw = np.where(stable_mask)[0]
print(f"Found {len(stable_indices_raw)} potential stable points.")

# --- Step 5: Exclude points near the end ---
exclude_start_index = len(filtered_signal) - exclude_end_samples
stable_indices_filtered = stable_indices_raw[stable_indices_raw < exclude_start_index]
print(f"Found {len(stable_indices_filtered)} stable points after excluding the last {exclude_end_samples} samples.")

# --- Step 6: Plot the results ---
print("Generating plots...")
plt.figure(figsize=(12, 9))

# Plot the filtered signal
plt.subplot(3, 1, 1)
plt.plot(filtered_signal, label='Filtered Signal')
plt.title(f'Filtered Signal (Low-pass {low_cutoff_freq} Hz)')
plt.xlabel('Sample Index')
plt.ylabel('Amplitude')
plt.grid(True)
plt.legend()

# Plot the filtered signal and highlight all identified stable points (before end exclusion)
plt.subplot(3, 1, 2)
plt.plot(filtered_signal, label='Filtered Signal')
# Plot stable points using the filtered indices
plt.plot(stable_indices_filtered, filtered_signal[stable_indices_filtered], 'ro', markersize=4, label=f'Stable Points (< {threshold:.4f})')
plt.title(f'Stable Sections Identified (Excluding Last {exclude_end_samples} Samples)')
plt.xlabel('Sample Index')
plt.ylabel('Amplitude')
plt.grid(True)
plt.legend()

# Plot only the stable points on top of the original filtered signal for clarity
plt.subplot(3, 1, 3)
plt.plot(filtered_signal, color='lightblue', label='_nolegend_') # Plot full signal faintly
plt.plot(stable_indices_filtered, filtered_signal[stable_indices_filtered], 'ro', markersize=4, label=f'Stable Points (< {threshold:.4f})')
plt.title('Stable Points Highlighted')
plt.xlabel('Sample Index')
plt.ylabel('Amplitude')
plt.grid(True)
plt.legend()


plt.tight_layout()
plt.show()

print("Plot displayed.")