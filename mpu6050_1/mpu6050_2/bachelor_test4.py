# -*- coding: utf-8 -*-
import time
import numpy as np
import matplotlib.pyplot as plt
from mpu6050 import mpu6050 # Importer MPU6050 biblioteket du bruker

# --- Innstillinger ---
sensor_address = 0x68      # Standard I2C adresse for MPU6050
target_sample_rate_hz = 100 # Ønsket samplingsfrekvens (Hz)
collection_duration_s = 10  # Hvor lenge data skal samles inn (sekunder)
use_g_force = True         # Hent data som G-krefter (True) eller m/s^2 (False)?

# --- Initialiser Sensor ---
try:
    sensor = mpu6050(sensor_address)
    print(f"MPU6050 initialisert på adresse {hex(sensor_address)}.")
    # Optional: Sett sensor-rekkevidde hvis ønskelig (fra tidligere kode)
    # sensor.set_accel_range(sensor.ACCEL_RANGE_2G)
except Exception as e:
    print(f"Kunne ikke initialisere MPU6050: {e}")
    print("Sjekk tilkobling og I2C-adresse.")
    exit()

# --- Datainnsamling ---
print(f"Starter datainnsamling i {collection_duration_s} sekunder...")

timestamps = []
x_data = []
y_data = []
z_data = []

start_time = time.time()
loop_start_time = start_time
actual_samples = 0

while (time.time() - start_time) < collection_duration_s:
    loop_start_time = time.time() # Nøyaktig tid ved start av loop

    try:
        # Les akselerometerdata
        accel_data = sensor.get_accel_data(g=use_g_force)

        # Lagre data og tidspunkt
        timestamps.append(loop_start_time) # Bruk tiden målingen ble tatt
        x_data.append(accel_data['x'])
        y_data.append(accel_data['y'])
        z_data.append(accel_data['z'])
        actual_samples += 1

    except Exception as e:
        print(f"Feil under lesing fra sensor: {e}")
        # Kan legge inn en kort pause her hvis feil skjer ofte
        # time.sleep(0.1)

    # Prøv å opprettholde samplingsraten
    loop_end_time = time.time()
    processing_time = loop_end_time - loop_start_time
    sleep_time = (1.0 / target_sample_rate_hz) - processing_time
    if sleep_time > 0:
        time.sleep(sleep_time)
    # else:
    #     print(f"Advarsel: Loop tar lengre tid ({processing_time:.4f}s) enn sample period ({1.0/target_sample_rate_hz:.4f}s)")


print(f"Datainnsamling ferdig. {actual_samples} samples samlet.")
if actual_samples < 2: # Trenger minst 2 punkter for å plotte en linje
     print("Ikke nok data samlet for plotting.")
     exit()

# --- Databehandling og Plotting ---

# Konverter lister til NumPy arrays
t_np = np.array(timestamps)
x_np = np.array(x_data)
y_np = np.array(y_data)
z_np = np.array(z_data)

# Beregngått tid fra start (første timestamp = 0)
t_elapsed = t_np - t_np[0]

# Beregn faktisk gjennomsnittlig samplingsfrekvens
actual_fs = actual_samples / (t_np[-1] - t_np[0]) if len(t_np)>1 else 0
print(f"Faktisk gjennomsnittlig samplingsfrekvens: {actual_fs:.2f} Hz")

# Sett Y-akse label basert på valg
if use_g_force:
    ylabel_text = 'Akselerasjon (G)'
else:
    ylabel_text = 'Akselerasjon (m/s^2)' # Hvis du brukte g=False

# Plot dataene
plt.figure(figsize=(12, 8)) # Litt større figur

# Subplot for X-data
plt.subplot(3, 1, 1)
plt.plot(t_elapsed, x_np, label='X')
plt.title('Akselerometer X Data')
plt.xlabel('Tid (s)')
plt.ylabel(ylabel_text)
plt.grid(True)
plt.legend()

# Subplot for Y-data
plt.subplot(3, 1, 2)
plt.plot(t_elapsed, y_np, label='Y', color='orange')
plt.title('Akselerometer Y Data')
plt.xlabel('Tid (s)')
plt.ylabel(ylabel_text)
plt.grid(True)
plt.legend()

# Subplot for Z-data
plt.subplot(3, 1, 3)
plt.plot(t_elapsed, z_np, label='Z', color='green')
plt.title('Akselerometer Z Data')
plt.xlabel('Tid (s)')
plt.ylabel(ylabel_text)
plt.grid(True)
plt.legend()

# Juster layout for å unngå overlapp
plt.tight_layout()

# Vis plottet
print("Viser plott...")
plt.show()

print("Ferdig.")