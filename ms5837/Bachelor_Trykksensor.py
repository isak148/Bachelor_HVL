#!/usr/bin/python
from ms5837 import ms5837
import time
import csv
import os

class analyse_MS5837:
        def __init__(self):
                self.sensor = ms5837.MS5837_02BA()
                self.last_trykk = 0.0
                #print("starter ")
                self.status_endring = None
                self.sensor.setFluidDensity(1010)
                self.Ivann = False
                self.forrgie_status = None
                self.retningsendring = None
                self.teller = 0 
                self.under_vann_30s=False
                self.under_vann=False
                self.Ivann=False

                if not self.sensor.init():
                        # We must initialize the sensor before reading it
                        print("Sensor could not be initialized")
                        exit(1)
 
       
        def read_sensor_data(self, Lagre = False , Filnavn = ""):    
                # Spew readings
                
                if self.sensor.read():
                        print(("P: %0.1f mbar ")%(self.sensor.pressure())) # Default is mbar (no arguments)

                        Trykk = self.sensor.pressure()
                        Trykk_dev = Trykk - self.last_trykk
                        self.last_trykk = Trykk
                        if (Trykk_dev < 0 - 0.5):
                                self.status_endring = "Synkende"
                                if (self.status_endring == self.forrgie_status):
                                        self.retningsendring = "Synkende"
                                else:
                                        self.retningsendring = "Uendret"
                        elif (Trykk_dev > 0 + 0.5):
                                self.status_endring = "ØKende"
                                if (self.status_endring == self.forrgie_status):
                                        self.retningsendring = "Økende"
                                else:
                                        self.retningsendring = "Uendret" 
                        else:
                                self.status_endring = "Uendret"

                else:
                        print("Sensor read failed!")
                        exit(1)
                
                if (Trykk > 1070): # ca 50cm under vann. 
                        self.under_vann = True
                        self.teller += 1 
                else:
                        self.under_vann = False
                        self.teller = 0
                
                if (self.teller > 60):
                        self.under_vann_30s = True

                
                if (Trykk > 1025.6): # 1cm under vannoverflate = 0.981mbar: tilsvarer nå 2cm
                        self.Ivann = True
                else:
                        self.Ivann = False
                
                if (Lagre == True):
                        self.skriv_til_fil(Filnavn, Trykk)


                return   {'Retningsendring' : self.retningsendring,
                         'Trykk': Trykk,   
                         'I_Vann': self.Ivann,
                         'Under_Vann': self.under_vann,
                         'Under_Vann_30s': self.under_vann_30s}
        
        def skriv_til_fil(self, filnavn, verdi):
                if not filnavn.endswith(".csv"):
                        filnavn += ".csv"  # Legger til .csv hvis det mangler

                with open(filnavn, mode='a', newline='', encoding='utf-8') as fil:
                        writer = csv.writer(fil)
                        writer.writerow([verdi])  # Skriver én tallverdi på ny linje




if __name__ == "__main__":
        sensor1 = analyse_MS5837()

        while(True):
                sensor1.read_sensor_data()
                time.sleep(0.5)
