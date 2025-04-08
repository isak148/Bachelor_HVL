import time
import board
import busio
import numpy as np
from collections import deque
import adafruit_vl6180x
from scipy.signal import find_peaks, savgol_filter
 

class AnalyseVL6180X:
    def __init__(self, i2c, address=0x29):
        self.sensor = adafruit_vl6180x.VL6180X(i2c, address)
        self.buffer_size = 100
        self.data_buffer = deque(maxlen=self.buffer_size)
        self.last_read_times = deque(maxlen=self.buffer_size)
    

    def read_sensor(self):
        try:
            distance = self.sensor.range
            read_time = time.monotonic()
            return distance, read_time
        except Exception as e:
            print(f"Kunne ikke lese sensor data: {e}")
            return None, None
        

    def get_data(self):
        result = self.read_sensor()
        if result is not None:
            distance, read_time = result
            self.data_buffer.append(distance)
            self.last_read_times.append(read_time)
    

    def pustestopp(self):
        if len(self.data_buffer) < self.buffer_size:
            return False
    

        data_window = list(self.data_buffer)[-int(2 * 10):] # Get last 2 seconds of data (assuming 10Hz)
        if not data_window:
            return False # Ingen data å sjekke
    

        first_value = data_window[0]
        for value in data_window:
            if not (first_value - 2 <= value <= first_value + 2):
                return False # Verdi utenfor terskel, ikke stabilt
    

            return True # Alle verdier innenfor terskel, stabilt
    

    def frekvens_bergening(self):
        if len(self.data_buffer) < 2:
            return 0 # Trenger minst 2 datapunkter
        

        data_array = np.array(self.data_buffer)
        peaks, properties = find_peaks(data_array, prominence=10)
        

        if len(peaks) < 2:
            return 0 # Trenger minst 2 topper
        

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
        else:
            return 0
        

 
    def gi_status(self):
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
    print_interval = 1.0 # Print status every 1 second
    read_interval = 0.01 # Read sensor every 0.01 seconds (100Hz)
    last_read_time = time.monotonic()


    try:
        while True: # Main loop for sensor reading
            if time.monotonic() - last_read_time >= read_interval:
                sensor_analyzer.get_data() # Read sensor data
                last_read_time = time.monotonic()


            while time.monotonic() - last_print_time >= print_interval: # Independent loop for printing
                status = sensor_analyzer.gi_status()
                print(f"Status: Pustestopp = {status['pustestopp']}, Frekvens = {status['frekvens']:.2f}")
                last_print_time = time.monotonic()
            

    except KeyboardInterrupt:
        print("Programmet avsluttet")