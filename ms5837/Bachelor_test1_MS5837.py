#!/usr/bin/python
import ms5837
import time
import csv
import os
class analyse_MS5837:
        def __init__(self):
                self.sensor = ms5837.MS5837_02BA()
                self.last_trykk = 0.0

        def read_sensor_data(self):
                # We must initialize the sensor before reading it
                if not self.sensor.init():
                        print("Sensor could not be initialized")
                        exit(1)

                # We have to read values from sensor to update pressure and temperature
                if not self.sensor.read():
                        print("Sensor read failed!")
                exit(1)

               
                # Spew readings
          
                if self.sensor.read():
                        print(("P: %0.1f mbar ")%(self.sensor.pressure())) # Default is mbar (no arguments)

                        Trykk = self.sensor.pressure()
                        Trykk_dev = Trykk - self.last_trykk
                        self.last_trykk = Trykk

                        print(("Trykk_dev: %0.1f mbar ")%(Trykk_dev)) # Default is mbar (no arguments)

                        
                        time.sleep(0.5) # Sleep for 500ms


                else:
                        print("Sensor read failed!")
                        exit(1)

if __name__ == "__main__":
        sensor = analyse_MS5837()
       
        
        while(True):
                status =sensor.read_sensor_data()
                print(status)
                time.sleep(0.05)
