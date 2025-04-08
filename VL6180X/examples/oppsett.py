import time
import board
import busio
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
        

    def read_sensor(self):
        #leser av veriden til sensoren og sendere det til get_data
        pass

    def get_data(self):
        #henter data fra sensor og lagrer det i en liste
        #dataen lagres og ser etter topper som videre blir sendt videre til frekvens_bergning for å ser på tiden mellom toppene.
        pass    

    def pustestopp(self):
        #sjekker etter stabilitet i dataen som indikerer at brukeren holder pusten
        #skjekker etter sabilitet på ca.2sek 
        pass

    def frekvens_bergening(self):
        #bergener frekvensen på pustingen til brukeren over en lengere periode på ca. 10 sek
        #Skal sende ut veirden selv om det er en pustestopp eller ikke, men frekvensen vil gå ned så lenge
        #brukeren holder pusten siden det ikke vil være noen topper i dataen.
        #toppene får verdien fra get_data og finner toppene i dataen og lagrer dem i en liste
        #finner tiden mellom toppene og rekner ut frekvensen fra dei
        pass

    def gi_status(self):
        #gir ut infoen fra pustestopp og frekvensberging
        pass


    if __name__ == "__main__":

        pass
