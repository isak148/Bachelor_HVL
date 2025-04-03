# -*- coding: utf-8 -*-
import time
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
from mpu6050 import mpu6050 # Importer MPU6050 biblioteket
import math # For sqrt

# --- Innstillinger ---
# MPU6050 & Datainnsamling
sensor_address = 0x68      # Standard I2C adresse
target_sample_rate_hz = 100 # Ønsket samplingsfrekvens (Hz)
collection_duration_s = 20  # Hvor lenge data skal samles inn (sekunder)

# Stabilitetsanalyse parametere (fra MATLAB)
low_cutoff_freq = 0.5  # Lavpassfilter cutoff (Hz)
filter_order = 4       # Filter orden
window_size = 100      # Vindustørrelse for rullerende std dev (samples)
threshold_multiplier = 1.5 # Multiplikator for median std dev -> terskel
exclude_end_samples = 100  # Antall samples å ignorere på slutten

# --- Initialiser Sensor ---
try:
    sensor = mpu6050(sensor_address)
    print(f"MPU6050 initialisert på adresse {hex(sensor_address)}.")
except Exception as e:
    print(f"Kunne ikke initialisere MPU6050: {e}")
    print("Sjekk tilkobling og I2C-adresse.")
    exit()

# --- Datainnsamling ---
print(f"Starter datainnsamling i {collection_duration_s} sekunder...")

timestamps = []
tot_G_data = [] # Vi lagrer Total G for stabilitetsanalyse

start_time = time.time()
loop_start_time = start_time
actual_samples = 0

while (time.time() - start_time) < collection_duration_s:
    loop_start_time = time.time() # Nøyaktig tid ved start av loop

    try:
        # Les akselerometerdata (som G-krefter)
        accel_data = sensor.get_accel_data(g=True)

        # Beregn Total G
        tot_g = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)

        # Lagre data og tidspunkt
        timestamps.append(loop_start_time)
        tot_G_data.append(tot_g)
        actual_samples += 1

    except Exception as e:
        print(f"Feil under lesing fra sensor: {e}")
        # time.sleep(0.1) # Vurder en pause hvis feil skjer ofte

    # Prøv å opprettholde samplingsraten
    loop_end_time = time.time()
    processing_time = loop_end_time - loop_start_time
    sleep_time = (1.0 / target_sample_rate_hz) - processing_time
    if sleep_time > 0:
        time.sleep(sleep_time)

print(f"Datainnsamling ferdig. {actual_samples} samples samlet.")

# --- Databehandling og Stabilitetsanalyse ---

if actual_samples < window_size + exclude_end_samples:
    print(f"Feil: For få samples ({actual_samples}) samlet for analyse med vindustørrelse {window_size} og ekskludering {exclude_end_samples}.")
    exit()

# Konverter til NumPy array
signal_data = np.array(tot_G_data)
t_np = np.array(timestamps)

# Beregn faktisk samplingsfrekvens (viktig for filter!)
actual_fs = actual_samples / (t_np[-1] - t_np[0]) if actual_samples > 1 else target_sample_rate_hz
print(f"Faktisk gjennomsnittlig samplingsfrekvens: {actual_fs:.2f} Hz")
fs = actual_fs # Bruk faktisk fs for filterdesign

# --- Step 2: Apply Low-pass Filter ---
print(f"Anvender lavpassfilter (cutoff: {low_cutoff_freq} Hz)...")
nyquist = 0.5 * fs
low = low_cutoff_freq / nyquist
# Sjekk om cutoff er gyldig (unngå feil hvis fs er for lav)
if low >= 1.0 or low <= 0.0:
    print(f"Advarsel: Ugyldig low-pass cutoff ({low_cutoff_freq} Hz) for samplingsfrekvens ({fs:.2f} Hz). Hopper over filtrering.")
    filtered_signal = signal_data # Bruk ufiltrert signal
else:
    b, a = butter(filter_order, low, btype='low', analog=False)
    filtered_signal = filtfilt(b, a, signal_data)
    print("Filtrering fullført.")

# --- Step 3: Calculate Rolling Standard Deviation ---
print(f"Beregner rullerende standardavvik (vindu: {window_size} samples)...")
s = pd.Series(filtered_signal)
rolling_std_series = s.rolling(window=window_size, center=False).std()
print("Rullerende standardavvik beregnet.")

# --- Step 4: Identify Stable Points ---
valid_rolling_std = rolling_std_series.dropna()
if len(valid_rolling_std) == 0:
    print("Feil: Kunne ikke beregne rullerende standardavvik.")
    exit()

threshold = valid_rolling_std.median() * threshold_multiplier
print(f"Stabilitetsterskel beregnet: {threshold:.4f}")

stable_mask = rolling_std_series < threshold
stable_indices_raw = np.where(stable_mask)[0]
print(f"Funnet {len(stable_indices_raw)} potensielle stabile punkter.")

# --- Step 5: Exclude points near the end ---
exclude_start_index = len(filtered_signal) - exclude_end_samples
stable_indices_filtered = stable_indices_raw[stable_indices_raw < exclude_start_index]
print(f"Funnet {len(stable_indices_filtered)} stabile punkter etter ekskludering.")

# --- Step 6: Plot the results ---
print("Genererer plot...")
plt.figure(figsize=(12, 9))
sample_indices = np.arange(len(filtered_signal)) # Lag en akse for sample index

# Plot 1: Filtrert signal
plt.subplot(3, 1, 1)
plt.plot(sample_indices, filtered_signal, label='Filtrert Total G')
plt.title(f'Filtrert Signal (Lavpass {low_cutoff_freq} Hz)')
plt.xlabel('Sample Index')
plt.ylabel('Amplitude (G)')
plt.grid(True)
plt.legend()

# Plot 2: Filtrert signal med stabile punkter markert
plt.subplot(3, 1, 2)
plt.plot(sample_indices, filtered_signal, label='Filtrert Total G')
if len(stable_indices_filtered) > 0:
    plt.plot(stable_indices_filtered, filtered_signal[stable_indices_filtered],
             'ro', markersize=4, label=f'Stabile Punkt (< {threshold:.4f})')
plt.title(f'Identifiserte Stabile Segmenter (Ekskl. siste {exclude_end_samples} samples)')
plt.xlabel('Sample Index')
plt.ylabel('Amplitude (G)')
plt.grid(True)
plt.legend()

# Plot 3: Samme som 2, for å matche MATLAB-struktur (kan endres om ønskelig)
plt.subplot(3, 1, 3)
plt.plot(sample_indices, filtered_signal, label='Filtrert Total G')
if len(stable_indices_filtered) > 0:
    plt.plot(stable_indices_filtered, filtered_signal[stable_indices_filtered],
             'ro', markersize=4, label=f'Stabile Punkt (< {threshold:.4f})')
plt.title('Stabile Punkt Markert')
plt.xlabel('Sample Index')
plt.ylabel('Amplitude (G)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

print("Plot vist. Ferdig.")