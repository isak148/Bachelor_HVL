from mpu6050 import mpu6050
from time import sleep 
import math
import time
import os
import csv

#import smbus2

class MPU6050_Orientation(mpu6050):
    def __init__(self, address, bus=1):
        super(). __init__(address, bus)

        #Justere følsomhet skala(grader/s) for gyro
        self.set_gyro_range(self.GYRO_RANGE_250DEG)
        #Juster akselrometer følsomhet 
        self.set_accel_range(self.ACCEL_RANGE_2G)
        #Velg filtrerng for mpu mer filtrering treigere respons 
        self.set_filter_range(self.FILTER_BW_188)

        self.gyro_offset = self.calibrate_gyro(50)
        self.accel_offset = self.calibrate_accel(50)

        self.gyro_angle_x = 0.0
        self.gyro_angle_y = 0.0
        self.alpha = 0.95 #Komplementær filtervekt
        self.last_time = time.time()

        # variabler for automatiskkalibrering av roll pitch 
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        #variabler for å telle stabile målinger
        self.stable_count_x = 0
        self.stable_count_y = 0
        self.stable_count_z = 0
        self.required_stable_count = 20
        self.threshold = 0.05





    def calibrate_accel(self, samples=100):
        print("starter kalibrering accelrometer")
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        for _ in range(samples):
            accel_data = self.get_accel_data(g=True)
            offset['x'] += accel_data['x']
            offset['y'] += accel_data['y']
            offset['z'] += accel_data['z']
            sleep(0.05)
        
        print("Ferdig kalibrert accelrometer")

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] = offset['z'] / samples - 1.0 # juster for gravitasjon 
        
        return offset  

    def calibrate_gyro(self, samples=100):
        print("starter kalibrering gyro")
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        for _ in range(samples):
            gyro_data = self.get_gyro_data() 
            offset['x'] += gyro_data['x']
            offset['y'] += gyro_data['y']
            offset['z'] += gyro_data['z']
            sleep(0.05)
       
        print("Ferdig kalibrert gyro")

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples
        
        return offset  



    def get_orientation(self, dt= 0.01):

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        #Beregner vinkelorientering basert på akselerometer og gyroskop
        accel_data = self.get_accel_data(g=True)
        gyro_data = self.get_gyro_data()
        
        #Juster for kalibrert bias
        gyro_data['x'] -= self.gyro_offset['x']
        gyro_data['y'] -= self.gyro_offset['y']
        gyro_data['z'] -= self.gyro_offset['z']

        accel_data['x'] -= self.accel_offset['x']
        accel_data['y'] -= self.accel_offset['y']
        accel_data['z'] -= self.accel_offset['z'] 
      
        
        # Beregn vinkel fra akselerometer i grader
        accel_angle_x = math.atan2(accel_data['y'], accel_data['x']) * (180/math.pi)
        accel_angle_y = math.atan2(-accel_data['x'], accel_data['z']) * (180/math.pi)
        
        
        
       
        #sjekker om en av aksene er stabil på en 1G
        if abs(accel_data['x']) > (1.0 - self.threshold) and abs(accel_data['x']) < (1.0 + self.threshold):
            self.stable_count_x += 1
        else:
            self.stable_count_x = 0

        if abs(accel_data['y']) > (1.0 - self.threshold) and abs(accel_data['y']) < (1.0 + self.threshold):
            self.stable_count_y += 1
        else:
            self.stable_count_y
        
        if abs(accel_data['z']) > (1.0 - self.threshold) and abs(accel_data['z']) < (1.0 + self.threshold):
            self.stable_count_z += 1
        else:
            self.stable_count_z = 0
        
        if self.stable_count_z >= self.required_stable_count:
            self.gyro_angle_x = 0.0 #Nullstil roll
            self.gyro_angle_y = 0.0 #Nullstill pitch 
            if accel_data['z'] > 0: # Hvis +1 g på z-aksen
                self.roll_offset = 0.0
                self.pitch_offset = 0.0
            else: #Hvis -1g på z aksen 
                self.roll_offset = 180.0 # sensoren er opp-ned 
                self.pitch_offset = 180.0
                self.gyro_angle_x = accel_angle_x +180
                self.gyro_angle_y = accel_angle_y +180                   
            self.stable_count_z = 0 # Resetter teller
        elif self.stable_count_y >= self.required_stable_count:
            self.gyro_angle_x = accel_angle_x # Nullstill akkumulert roll 
            if accel_data['y'] > 0: # hvis +1g på Y-aksen
                self.roll_offset = 90.0
                self.pitch_offset = 0.0
            else: # -1G på y-aksen 
                self.roll_offset = -90.0
                self.pitch_offset = 0.0
            self.stable_count_y = 0 # Resetter teller
        elif self.stable_count_x >= self.required_stable_count:
            self.gyro_angle_y = accel_angle_y # Nullstill akkumulert pitch 
            if accel_data['x'] < 0: # hvis +1g på Y-aksen 
                self.pitch_offset = -90.0
                self.roll_offset = 0.0
            else: #Hvis -1g på x-aksen
                self.pitch_offset = 90.0
                self.roll_offset = 0.0
            self.stable_count_x = 0 # Resetter teller 
        else:
            pass 

        #Integrer gyro-data for å beregne vinkelendring
        self.gyro_angle_x += gyro_data['x'] * dt
        self.gyro_angle_y += gyro_data['y'] * dt

        #bruk komplementært filter: kombiner gyro og akselerometer
        angle_x =self.alpha * self.gyro_angle_x + (1- self.alpha) * (accel_angle_x - self.roll_offset)
        angle_y =self.alpha * self.gyro_angle_y + (1- self.alpha) * (accel_angle_y - self.pitch_offset)


        return {'roll': angle_x, 'pitch': angle_y}

    def gi_status_aks(self):
        
        mpu = MPU6050_Orientation(0x68) # opretter nytt MPU objekt

        accel_data = mpu.get_accel_data(g=True)
        tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
        
        
        orientation = mpu.get_orientation()
        roll = orientation['roll']
        pitch = orientation['pitch']
        
        data = {tot_G, roll, pitch}

        return data 


        
        

#vi trenger ikkje denne da vi har en egen funksjon for å hente data

if __name__ == "__main__":
    #sensor = mpu6050(0x68)
    mpu = MPU6050_Orientation(0x68)
    



    while True:
        accel_data = mpu.get_accel_data(g=True)
        gyro_data = mpu.get_gyro_data()
        #temp = sensor.get_temp()
        orientation = mpu.get_orientation()

        print("Accelerometer data")
        print("x: " + str(accel_data['x']-mpu.accel_offset['x']))
        print("y: " + str(accel_data['y']-mpu.accel_offset['y']))
        print("z: " + str(accel_data['z']-mpu.accel_offset['z']))

        print("Gyroscope data")
        print("x: " + str(gyro_data['x']- mpu.gyro_offset['x']))
        print("y: " + str(gyro_data['y']- mpu.gyro_offset['y']))
        print("z: " + str(gyro_data['z']- mpu.gyro_offset['z']))

        #print("Temp: " + str(temp) + " C")
    
    
        print(f"Roll: {orientation['roll']:.2f}, Pitch: {orientation['pitch']:.2f}")

        if abs(orientation['pitch']) > 45.0:
            print("svømmer")
        else: 
            print("flyter")

        file_path = "sensor_data.csv"
        file_exists = os.path.exists(file_path)

        with open(file_path, "w", newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(["Accelerometer x", "Accelerometer y", "Accelerometer z", 
                                "Gyroscope x", "Gyroscope y", "Gyroscope z", "role", "pitch"])

            writer.writerow([accel_data['x'], accel_data['y'], accel_data['z'], 
                            gyro_data['x'], gyro_data['y'], gyro_data['z'],orientation['roll'],orientation['pitch']])
        sleep(0.01)
