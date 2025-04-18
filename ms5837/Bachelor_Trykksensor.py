#!/usr/bin/python
from ms5837 import ms5837
import time
import csv
import os

class analyse_MS5837:
        def __init__(self):
                self.sensor = ms5837.MS5837_02BA()
                self.last_trykk = 0.0
                print("starter ")
                self.status_endring = None
                self.sensor.setFluidDensity(1010)
                self.Ivann = False
                self.forrgie_status = None
                self.retningsendring = None

                if not self.sensor.init():
                        # We must initialize the sensor before reading it
                        print("Sensor could not be initialized")
                        exit(1)
 
       
        def read_sensor_data(self):    
                # Spew readings
                time.sleep(0.5)
                if self.sensor.read():
                        print(("P: %0.1f mbar ")%(self.sensor.pressure())) # Default is mbar (no arguments)

                        Trykk = self.sensor.pressure()
                        Trykk_dev = Trykk - self.last_trykk
                        self.last_trykk = Trykk
                        if (Trykk_dev < 0 - 0.5):
                                self.status_endring = "synkende"
                                if (self.status_endring == self.forrgie_status):
                                        self.retningsendring = "synkende"
                                else:
                                        self.retningsendring = "uendret"
                        elif (Trykk_dev > 0 + 0.5):
                                self.status_endring = "økende"
                                if (self.status_endring == self.forrgie_status):
                                        self.retningsendring = "økende"
                                else:
                                        self.retningsendring = "uendret" 
                        else:
                                self.status_endring = "uendret"
                                
                       
                        

                        #print(("Trykk_dev: %0.1f mbar ")%(Trykk_dev)) # Default is mbar (no arguments)

                        
                        #time.sleep(0.5) # Sleep for 500ms # Trengs i hovedløkke


                else:
                        print("Sensor read failed!")
                        exit(1)
                
                if (Trykk > 1090): # ca 70cm under vann. 
                        under_vann = True
                else:
                        under_vann = False 

                
                if (Trykk > 1025.6): # 1cm under vannoverflate = 0.981mbar: tilsvarer nå 2cm
                        Ivann = True
                else:
                        Ivann = False

                return   {'status' : self.retningsendring,
                         'Trykk': Trykk,
                         'I_vann': Ivann,
                         'under_vann': under_vann} 



if __name__ == "__main__":
        sensor1 = analyse_MS5837()

        while(True):
                sensor1.read_sensor_data()
                time.sleep(0.5)
