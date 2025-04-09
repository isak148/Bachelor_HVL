import time
import board
import busio
import statistics
import numpy as np
from collections import deque
import adafruit_vl6180x
from scipy.signal import find_peaks, savgol_filter


class AnalyseVL6180X:
    def __init__(self, i2c, address=0x29):
        #initialiserer sensor og setter opp i2c.
        #setter opp basisveriden for filter og buffer
        self.sensor = adafruit_vl6180x.VL6180X(i2c, address)
        self.buffer_size = 100
        self.data_buffer = deque(maxlen=self.buffer_size)
        self.range_arry = np.array([])
        # Felles variabler for statusvurdering
        self.sample_rate = 20  # Hz (Samples per sekund)
         # Samme vindusstørrelse brukes for både akselerometer og gyro analyse
        self.window_size_stopp = 40 # 2 sekunds vindu (50 samples)
        self.window_size_frekvens = 100 # 5 sekunds vindu (50 samples) 
        self.raw_stopp_buffer =deque(maxlen=self.window_size_stopp)
        self.raw_frekvens_buffer = deque(maxlen=self.window_size_frekvens)

        self.last_computed_stopp_status = "venter på data på pustestopp..."
        self.last_computed_frekvens_status = "venter på data på frekvens..."
        

    def read_sensor(self):
        #leser av veriden til sensoren og sendere det til get_data
        pass

    def get_data(self):
        #henter data fra sensor og lagrer det i en liste
        #dataen lagres og ser etter topper som videre blir sendt videre til frekvens_bergning for å ser på tiden mellom toppene.
        pass    

    def pustestopp(self, range, threshold=5):
        range_array = self.raw_stopp_buffer
        lav_øvre_grense = range + threshold
        lav_nedre_grense = range - threshold

        if np.all(range_array >= lav_nedre_grense) and np.all(range_array <= lav_øvre_grense):
           print("Pustestopp registrert")

        
        

    def frekvens_bergening(self,):
        #bergener frekvensen på pustingen til brukeren over en lengere periode på ca. 10 sek
        #Skal sende ut veirden selv om det er en pustestopp eller ikke, men frekvensen vil gå ned så lenge
        #brukeren holder pusten siden det ikke vil være noen topper i dataen.
        #toppene får verdien fra get_data og finner toppene i dataen og lagrer dem i en liste
        #finner tiden mellom toppene og rekner ut frekvensen fra dei
        DEBOUNCE_TIME = 0.25
        value = self.sensor.range
        now = time.time()
        bpm_history = self.raw_frekvens_buffer
        MAX_HISTORY = 5

        THRESHOLD = 3
        # Detekter ny topp (går over terskel og var under forrige gang)
        if value > THRESHOLD and last_value <= THRESHOLD:
            if last_peak_time is None:
                last_peak_time = now
                print("Første topp registrert")
            else:
                rr_interval = now - last_peak_time
                if rr_interval > DEBOUNCE_TIME:
                    bpm = 60 / rr_interval
                    bpm_history.append(bpm)
                    if len(bpm_history) > MAX_HISTORY:
                        bpm_history.pop(0)

                    median_bpm = statistics.median(bpm_history)
                    print(f"Puls (median av siste {len(bpm_history)}): {median_bpm:.1f} BPM")

                    last_peak_time = now

        last_value = value
        

    def gi_status(self):
        #gir ut infoen fra pustestopp og frekvensberging
        Range_data = self.sensor.range

        self.raw_stopp_buffer.append(Range_data)
        self.raw_frekvens_buffer.append(Range_data)
        

        status_fra_pustestopp = self.last_computed_stopp_status
        status_fra_frekvens = self.last_computed_frekvens_status

    

        if len(self.raw_stopp_buffer) == self.window_size_stopp:
            # Kall de (nå modifiserte) vurderingsfunksjonene
            status_fra_G = self.pustestopp(list(self.raw_stopp_buffer))
        
            self.raw_stopp_buffer.clear()    

        if len(self.raw_frekvens_buffer) == self.window_size_frekvens:
            # Kall de (nå modifiserte) vurderingsfunksjonene
            status_fra_G = self.frekvens_bergening(list(self.raw_frekvens_buffer))

            self.raw_frekvens_buffer.clear()
        


if __name__ == "__main__":
    sensor = AnalyseVL6180X()
    
    while(True):
        status =sensor.gi_status()
        print(status)
        time.sleep(0.05)
    
