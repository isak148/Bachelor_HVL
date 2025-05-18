import time
import board
import busio
import numpy as np
from collections import deque
import csv
try:
    from scipy.signal import find_peaks, savgol_filter
except ImportError:
    print("Feil: Kunne ikke importere 'scipy.signal'.")
    print("Installer SciPy med: pip install scipy")
    exit()

# Importer det lokale Adafruit VL6180X biblioteket
try:
    from VL6180X.examples import adafruit_vl6180x
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

         self.window_size = 20 # 2 sekunds vindu (50 samples)
         self.window_size_pust = 160

            # Buffere for rådata (magnitude)
         self.raw_puste_buffer = deque(maxlen=self.window_size)
         self.raw_puste_frekvens_buffer = deque(maxlen=self.window_size_pust)

         # Variabler for Akselerometer (G) stabilitetsvurdering
         self.last_computed_puste_status = "Initialiserer..."

         self.svar = "initialiserer"

         self.last_range_data = 0
         self.pust = 0.0
         self.count = None

    def vurder_stabilitet_P(self, tot_P_values, toleranse=5):
            """
            Vurderer stabilitet basert på en 2 sekunders samplingsperiode, funksjonen tar høyeste og laveste i listen
            og sjekker om differansen er over 5mm, Om differansen ikke er over dette blir puste stopp retunert. Dersom 
            puste stopp blir telt kontinuerlig over en periode på 60 sekunder retuneres puster ikke. 
            """

            # Konverter til numpy array for enklere testing
            p_array = np.array(tot_P_values)

            # Finn maks og min
            max_val = np.max(p_array)
            min_val = np.min(p_array)

            # Sjekk om forskjellen overstiger 5
            if max_val - min_val > toleranse:
                #print("Avvik større enn 5!")
                self.svar = "Puster"
                self.count = 0
            else:
                #print("Alt innenfor grense.")
                self.svar = "Puste_Stopp"
                self.count = +1
                if (self.count < 30):
                    self.svar = "Puster_Ikke"


            self.last_computed_puste_status = self.svar
            return self.svar 



    def kalkuler_pustefrekvens(self, data):
        '''
        Denne funksjonen teller bytter mellom + og - av den deriverte mellom hver tidsenhet av en liste med 160 verdier.
        Dette tilsvarer 8 sekunder med en sampling på 20hz, når det er telt enten 5 positive eller 5 negative verdier blir
        en teller oppdatert med +1. Det betyr at refleksjonflaten må holde samme retning i 250 ms sammenhengende. 
        Dette fjerner sannsynlighten for og telle "feil ved støy. Resultatet blir brukt til og beregne pustefrekvens
        '''
        count = 0
        current_sign = None
        streak = 0
        ready_for_switch = False

        for value in data:
            # Nullverdier hopper vi over uten å påvirke streaken
            if value == 0:
                continue

            sign = 1 if value > 0 else -1

            if sign == current_sign:
                streak += 1
            else:
                if streak >= 5:
                    ready_for_switch = True
                else:
                    ready_for_switch = False

                if ready_for_switch and sign != current_sign:
                    count += 1
                    ready_for_switch = False
                    streak = 1  # start ny streak
                else:
                    streak = 1

            current_sign = sign

        return count


    def analyserer_stopp(self, Lagre = False, Filnavn = ""):
        '''Lager og  sender buffervindu til vurder_stabilitet og kalkuler_pustefrekvens.
        metoden brukes av en thread i hovedprogrammet.'''
        Range_data = self.sensor.range

        self.raw_puste_buffer.append(Range_data)
        Range_data_pust = Range_data - self.last_range_data
        self.raw_puste_frekvens_buffer.append(Range_data_pust)

        self.last_range_data = Range_data

        status_fra_pust = self.last_computed_puste_status
    

        if len(self.raw_puste_buffer) == self.window_size:
            # Kall de (nå modifiserte) vurderingsfunksjonene
            status_fra_pust = self.vurder_stabilitet_P(list(self.raw_puste_buffer))
            

            self.raw_puste_buffer.clear()

        if len(self.raw_puste_frekvens_buffer) == self.window_size_pust:
            count  = self.kalkuler_pustefrekvens(list(self.raw_puste_frekvens_buffer))

            self.pust = (count/8/2) * 60
            self.raw_puste_frekvens_buffer.clear()

        if (self.pust <= 20):
             puste_frekvens = "Lav"
        elif (12 < self.pust <= 30):
            puste_frekvens = "Normal"
        elif (self.pust > 40):
            puste_frekvens = "Høy"
        else:
            puste_frekvens = "initialiserer"

        if(Lagre == True): # skriver til fil om Lagre er satt til True
            self.skriv_til_fil(Filnavn, Range_data)
      

        return {        
            'Pust_Status': status_fra_pust,
            'Pust_Frekvens': puste_frekvens
        }
    
    def skriv_til_fil(self, filnavn, verdi):
                if not filnavn.endswith(".csv"):
                        filnavn += ".csv"  # Legger til .csv hvis det mangler

                with open(filnavn, mode='a', newline='', encoding='utf-8') as fil:
                        writer = csv.writer(fil)
                        writer.writerow([verdi])  # Skriver én tallverdi på ny linje


    

if __name__ == "__main__":
    sensor = VL6180XAnalyser()

    while(True):
        status =sensor.analyserer_stopp()
        print(status['Pust_Status'])
        print(status['Pust_Frekvens'])
        time.sleep(0.05)

    


    