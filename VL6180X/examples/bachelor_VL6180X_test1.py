import time
import board
import busio
import numpy as np
from collections import deque
try:
    from scipy.signal import find_peaks, savgol_filter
except ImportError:
    print("Feil: Kunne ikke importere 'scipy.signal'.")
    print("Installer SciPy med: pip install scipy")
    exit()

# Importer det lokale Adafruit VL6180X biblioteket
try:
    import adafruit_vl6180x
except ImportError:
    print("Feil: Kunne ikke importere 'adafruit_vl6180x'.")
    print("Sørg for at biblioteket er installert eller tilgjengelig i PYTHONPATH.")
    exit()



class VL6180XAnalyser:
    """
    En klasse for å analysere VL6180X sensordata i sanntid for å
    detektere perioder med stabil avstand (apnea/pustestopp) og
    estimere bevegelsesfrekvens.
    """

    def __init__(self):
         self.sensor = None
         try:
            # Standard I2C-pinner for RPi via Blinka
            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_vl6180x.VL6180X(i2c)
            print("VL6180X sensor initialisert.")
         except Exception as e:
            print(f"KRITISK FEIL: Kunne ikke initialisere I2C eller sensor: {e}")
            print("Fortsetter uten sensor. update() vil returnere feil.")

         # Felles variabler for statusvurdering
         self.sample_rate = 20  # Hz (Samples per sekund)
         # Samme vindusstørrelse brukes for både akselerometer og gyro analyse
         self.window_size = 20 # 2 sekunds vindu (50 samples)

            # Buffere for rådata (magnitude)
         self.raw_accel_buffer = deque(maxlen=self.window_size)
         self.raw_gyro_buffer = deque(maxlen=self.window_size)

         # Variabler for Akselerometer (G) stabilitetsvurdering
         self.accel_stabilitet_historikk = []
         self.last_computed_accel_status = "Initialiserer..."

         self.svar = "initialiserer"

    def vurder_stabilitet_G(self, tot_G_values, toleranse=10):
            """
            Vurderer akselerometer-aktivitet basert på terskler INNENFOR vinduet,
            UTEN å beregne gjennomsnitt av tot_G_values.
            Klassifiserer vinduet basert på om noen verdier er høye, eller om alle er lave.
            Den videre logikken med å samle 10 nivåer og ta snittet av DEM er beholdt.
            """
            #if not tot_G_values: return self.last_computed_accel_status

            # Konverter til numpy array for enklere testing
            g_array = np.array(tot_G_values)

            # Finn maks og min
            max_val = np.max(g_array)
            min_val = np.min(g_array)

            # Sjekk om forskjellen overstiger 5
            if max_val - min_val > toleranse:
                #print("Avvik større enn 5!")
                self.svar = "Puster"
            else:
                #print("Alt innenfor grense.")
                self.svar = "Puster Ikke"

            self.last_computed_accel_status = self.svar
            return self.svar 


    def analyserer_stopp(self):

        Range_data = self.sensor.range

        self.raw_accel_buffer.append(Range_data)
        

        status_fra_G = self.last_computed_accel_status
    

        if len(self.raw_accel_buffer) == self.window_size:
            # Kall de (nå modifiserte) vurderingsfunksjonene
            status_fra_G = self.vurder_stabilitet_G(list(self.raw_accel_buffer))
            

            self.raw_accel_buffer.clear()
    
            

        return {        
            'aks_status': status_fra_G,
        }
    

if __name__ == "__main__":
    sensor = VL6180XAnalyser()

    while(True):
        status =sensor.analyserer_stopp()
        print(status)
        time.sleep(0.05)

    


    