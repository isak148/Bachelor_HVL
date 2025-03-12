import time
import matplotlib.pyplot as plt
import adafruit_ads1x15.ads1015 as ADS1015
import adafruit_ads1x15.analog_in as AnalogIn
import board
import busio
import numpy as np
import math

# Opprett I2C-bussen
i2c = busio.I2C(board.SCL, board.SDA)

# Opprett ADS1015-objektet
adc = ADS1015.ADS1015(i2c)

# Opprett en analog inngangskanal
channel = AnalogIn.AnalogIn(adc, ADS1015.P0)  # P0 tilsvarer kanal 0

# Variabler for plotting og pulsmåling
sample_rate = 3# Prøver per sekund (juster om nødvendig)
buffer_size = 2500000  # 10 sekunder med data
data_buffer = np.zeros(buffer_size)
time_buffer = np.zeros(buffer_size)
pulse_history = []
pulse = 0

def get_adc_value():
    return channel.value#Bruker .value for og hente ut verdien

def calculate_pulse(data, sample_rate):
    # Enkel pulsmåling: Teller topper i signalet
    threshold = (np.max(data) - np.min(data)) * 0.6 # Finne toppene i signalet.
    peaks = np.where((data[1:-1] > threshold) & (data[1:-1] > data[0:-2]) & (data[1:-1] > data[2:]))[0] + 1
    if len(peaks) > 1:
        time_diffs = np.diff(peaks) / sample_rate
        if len(time_diffs) > 0 :
            mean_time_diff = np.mean(time_diffs)
            if mean_time_diff > 0:
                pulse = 60 / mean_time_diff
                pulse_history.append(pulse)
                if len(pulse_history) > 25: # Tar et gjennomsnitt av de 10 siste pulsene.
                  pulse_history.pop(0)
                return np.mean(pulse_history)
            else:
                return None
"""
def update_plot():
    plt.clf()
    plt.plot(time_buffer, data_buffer)
    plt.xlabel('Tid (sekunder)')
    plt.ylabel('ADC Verdi')
    plt.title('EKG Signal')
    if pulse is not None:
      plt.text(0.05, 0.95, f'Estimert Puls: {pulse:.2f} BPM', transform=plt.gca().transAxes)
    plt.pause(0.01)
"""
# Hovedløkke
try:
    start_time = time.time()
    for i in range(buffer_size):
        data_buffer[i] = get_adc_value()
        time_buffer[i] = time.time() - start_time
        time.sleep(1 / sample_rate)
        if i > sample_rate: #Måles ikke puls før det har gått et sekund.
          pulse = calculate_pulse(data_buffer[0:i], sample_rate)
        print(f"BMP:{pulse}, {data_buffer[i]}")
#
 
#       if i % (sample_rate / 2) == 0:  # Oppdater plottet hvert halve sekund
#            update_plot()
except KeyboardInterrupt:
    print('Avslutter...')
    plt.close()