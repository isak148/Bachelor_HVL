import time
import board
import busio
import numpy as np
from collections import deque
import adafruit_vl6180x
from scipy.signal import find_peaks, savgol_filter


class AnalyseVL6180X:
    def __init__(self, i2c, address=0x29):
        # Initialiserer sensor og setter opp i2c.
        # Setter opp basisveriden for filter og buffer
        self.sensor = adafruit_vl6180x.VL6180X(i2c, address)
        self.buffer_size = 100
        self.data_buffer = deque(maxlen=self.buffer_size)
        self.last_read_time = time.monotonic()  # legger til for å spore tidspunkt for siste avlesning
        self.stable_start_time = None  # Legg til for å spore starten på stabil periode
        self.apnea_threshold = 2.0  # Sett en terskel for standardavvik for å definere stabilitet
        self.apnea_duration = 2.0  # Sett varighet for pustestopp [cite: 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123]

    def read_sensor(self):
        # Leser av verdien til sensoren og sender det til get_data
        try:
            distance = self.sensor.range
            return distance
        except Exception as e:
            print(f"Kunne ikke lese sensor data: {e}")
            return None

    def get_data(self):
        # Henter data fra sensor og lagrer det i en liste
        # Dataen lagres og ser etter topper som videre blir sendt videre til frekvens_bergning for å se på tiden mellom toppene.
        distance = self.read_sensor()
        if distance is not None:
            self.data_buffer.append(distance)

    def pustestopp(self):
        # Sjekker etter stabilitet i dataen som indikerer at brukeren holder pusten
        # Sjekker etter stabilitet på ca. 2 sek
        if len(self.data_buffer) < self.buffer_size:
            return False  # Ikke nok data til å vurdere stabilitet

        data_array = np.array(self.data_buffer)
        std_dev = np.std(data_array)
        is_stable = std_dev < self.apnea_threshold

        if is_stable:
            if self.stable_start_time is None:
                self.stable_start_time = time.monotonic()
            elif time.monotonic() - self.stable_start_time >= self.apnea_duration:
                return True  # Pustestopp oppdaget
        else:
            self.stable_start_time = None  # Reset timer hvis ikke stabil
        return False

    def frekvens_bergening(self):
        # Beregner frekvensen på pustingen til brukeren over en lengere periode på ca. 10 sek
        # Skal sende ut verdien selv om det er en pustestopp eller ikke, men frekvensen vil gå ned så lenge
        # brukeren holder pusten siden det ikke vil være noen topper i dataen.
        # Toppene får verdien fra get_data og finner toppene i dataen og lagrer dem i en liste
        # Finner tiden mellom toppene og regner ut frekvensen fra dei
        if len(self.data_buffer) < self.buffer_size:
            return None  # Ikke nok data til å beregne frekvens

        data_array = np.array(self.data_buffer)
        peaks, properties = find_peaks(data_array, prominence=10)  # Add prominence threshold

        if len(peaks) < 2:
            return 0  # Kan ikke beregne frekvens med mindre enn 2 topper

        peak_times = [i * (1 / 10) for i in peaks]  # estimerer tid basert på bufferposisjoner (10hz)
        time_diffs = np.diff(peak_times)
        if len(time_diffs) > 0:
            mean_time_diff = np.mean(time_diffs)
            if mean_time_diff > 0:
              frequency = 60 / mean_time_diff
              return frequency
            else:
              return 0
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

    try:
        while True:
            sensor_analyzer.get_data()
            status = sensor_analyzer.gi_status()

            print(f"Status: Pustestopp = {status['pustestopp']}, Frekvens = {status['frekvens']:.2f} ")
            time.sleep(0.1)  # Leser sensoren med 10Hz
    except KeyboardInterrupt:
        print("Programmet avsluttet")