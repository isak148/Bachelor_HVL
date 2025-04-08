import time
import board
import busio
import numpy as np
from collections import deque
import adafruit_vl6180x
from scipy.signal import find_peaks, savgol_filter
from collections import deque  # Import deque if not already imported


class AnalyseVL6180X:
    def __init__(self, i2c, address=0x29):
        # Initialiserer sensor og setter opp i2c.
        # Setter opp basisveriden for filter og buffer
        self.sensor = adafruit_vl6180x.VL6180X(i2c, address)
        self.buffer_size = 100
        self.data_buffer = deque(maxlen=self.buffer_size)
        self.last_read_time = time.monotonic()  # legger til for å spore tidspunkt for siste avlesning
        self.stable_start_time = None  # Legg til for å spore starten på stabil periode
        self.apnea_threshold = 5.0  # Sett en terskel for standardavvik for å definere stabilitet
        self.apnea_duration = 2.0  # Sett varighet for pustestopp [cite: 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123]
        self.last_read_times = deque(maxlen=self.buffer_size) # Add this line
        self.sensor._load_settings()

    def read_sensor(self):
        # Leser av verdien til sensoren og sender det til get_data

            distance = self.sensor.range
            read_time = time.monotonic() # Get the current time
            return distance, read_time # Return both distance and time
        # Henter data fra sensor og lagrer det i en liste
        # Dataen lagres og ser etter topper som videre blir sendt videre til frekvens_bergning for å se på tiden mellom toppene.
        
    def get_data(self):
        result = self.read_sensor()
        if result is not None:
            distance, read_time = result  # Unpack the tuple
            self.data_buffer.append(distance)
            self.last_read_times.append(read_time)

    def pustestopp(self):
        # Sjekker etter stabilitet i dataen som indikerer at brukeren holder pusten
        # Sjekker etter stabilitet: ingen endring i avstanden +- threshold i mer enn 2 sek

        if len(self.data_buffer) < self.buffer_size:
            return False  # Ikke nok data til å vurdere stabilitet

        data_window = list(self.data_buffer)[-int(self.apnea_duration * 10):]  # Get last 2 seconds of data (assuming 10Hz)
        if not data_window:
            return False  # Ingen data å sjekke

        first_value = data_window[0]
        for value in data_window:
            if not (first_value - self.apnea_threshold <= value <= first_value + self.apnea_threshold):
                return False  # Verdi utenfor terskel, ikke stabilt

        return True  # Alle verdier innenfor terskel, stabilt

    def frekvens_bergening(self):
        # Beregner frekvensen på pustingen.
        # Beregner frekvensen så fort det er 2 topper.
        # Bruker et sliding window for å kun se på de n siste toppene
        if len(self.data_buffer) < 2:
            return 0  # Trenger minst 2 datapunkter
        if self.data_buffer:

            data_array = np.array(list(self.data_buffer))
        else:
            return 0
        peaks, properties = find_peaks(data_array, prominence=10)
    
        if len(peaks) < 2:
            return 0  # Trenger minst 2 topper
        
        # Bruk de to siste toppene for å beregne frekvens
        if len(peaks) >= 2:
            peak_times = [self.last_read_times[i] for i in peaks[-2:] if i < len(self.last_read_times)]
            time_diffs = np.diff(peak_times)
            if time_diffs:
                mean_time_diff = np.mean(time_diffs)
                if mean_time_diff > 0:
                    frequency = 60 / mean_time_diff
            return frequency
        else:
            return 0
        

    def gi_status(self):
        # Gir ut infoen fra pustestopp og frekvensberging
        apnea = self.pustestopp()
        frequency = self.frekvens_bergening()

        status = {
            "pustestopp": apnea,
            "frekvens": frequency
        }
        return status


if __name__ == "__main__":
  i2c = busio.I2C(board.SCL, board.SDA)
  sensor_analyzer = AnalyseVL6180X(i2c)
 

  last_print_time = time.monotonic()
  print_interval = 1.0  # Print status every 1 second
  read_interval = 0.01 # Read sensor every 0.01 seconds (100Hz)
  last_read_time = time.monotonic()
 

  try:
    while True:  # Main loop for sensor reading
        if time.monotonic() - last_read_time >= read_interval:
            sensor_analyzer.get_data()  # Read sensor data
            last_read_time = time.monotonic()
            

        while time.monotonic() - last_print_time >= print_interval: # Independent loop for printing
            status = sensor_analyzer.gi_status()
            print(f"Status: Pustestopp = {status['pustestopp']}, Frekvens = {status['frekvens']:.2f}")
            last_print_time = time.monotonic()
    

  except KeyboardInterrupt:
    print("Programmet avsluttet")

    