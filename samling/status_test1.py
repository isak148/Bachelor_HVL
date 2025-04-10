from mpu6050_1.mpu6050_2 import bachelor_akselrasjon_sensor as Akssensor
from ms5837 import Bachelor_Trykksensor as Trykksensor
from ArduinoAD8232ECG import Bachelor_Pulsmåling as pulssensor
from VL6180X.examples import bachelor_pustesenor as pustesensor 
import threading
import time


class Status: 

    def __init__(self):
        self.variabler = {} 
        self.trykk_sensor = Trykksensor.analyse_MS5837()
        self.puls_sensor = pulssensor()
        self.aks_sensor = Akssensor.MPU6050_Orientation()
        self.LFR_sensor = pustesensor.VL6180XAnalyser()
        self.ivann = False
        self.data_aks = {}
        self.data_LFR = {}
        self.data_pulse = {}
        self.data_pressure = {}
        self.oppstart = False
    
    def threding_start(self,verdi):
        if(verdi == True):
            self.oppstart = True
        else:
            self.oppstart = False
        
    def get_data_aks(self):
        # Denne skal retunere: 'total_G': tot_G, 'is_periodic': last_periodicity_status,'retning' : retning
        while(self.oppstart):
            self.data_aks = self.aks_sensor.oppdater_og_vurder_status()
        
            return {"aks": self.data_aks}
    
    def get_data_LFR_Preasure(self):
        while(self.oppstart):
            self.data_LFR = self.LFR_sensor.analyserer_stopp()
            self.data_preasure = self.trykk_sensor.read_sensor_data() 
            
            return {      
                "LFR": self.data_LFR,
                "pressure": self.data_pressure
                }

    def get_data_pulse(self):
        while(self.oppstart):
            self.data_pulse = self.puls_sensor  # her må man få inn en klasse som retunerer noe brukende
            
            return {
                "pulse": self.data_pulse    
                }


    def Svømmer(self):
        # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        if self.data_aks == "Normal Aktivitet":
            status = True
        else:
            status = False
        return status

    def Flyter(self):
         # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        if self.data_aks == "Stille":
            status = True
        else:
            status = False
        return status
    
    def Dykker(self):
        # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        if self.data_trykk == "Økende":
            status = True
        else:
            status = False
        return status
        

    def Svømmer_opp(self):
         # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        if self.data_trykk == "Synkende":
            status = True
        else:
            status = False
        return status
    
    
    def Drukner(self):
        # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false 
        if self.data_aks == "Høy aktivitet"& self.data_LFR == "Puster ikke":
            status = True
        else:
            status = False
        return status
 

    def aktivert(self):
        # Denne skal bestemme om svømmeren er i vann og aktivere status analyse.
        if self.trykk_sensor.read_sensor_data()['I_vann'] == True:
            self.ivann = True
            count = 0
        elif self.trykk_sensor.read_sensor_data()['I_vann'] == False:
            count=+1
            if count > 10:
                self.ivann = False
                count = 0
            else:
                 self.ivann = True
        
        return self.ivann 
        

if __name__ == "__main__":
    while True:
        ivann_aktiv =Status.aktivert() # Denne må være i hovedprogrammet og bestemme om svømmeren er i vann eller ikke.:
       
        if ivann_aktiv == True:
            Status.threding_start(True)
            data_aks = threading.Thread(target=Status.get_data_aks)
            data_LFR_Trykk = threading.Thread(target=Status.get_data_LFR_Preasure)
            data_puls = threading.Thread(target=Status.get_data_pulse)
            aktivering = threading.Thread(target=Status.aktivert)
            
            data_aks.start()
            data_LFR_Trykk.start()
            data_puls.start()
            aktivering.start()

            data_aks.join()
            data_LFR_Trykk.join()
            data_puls.join()
            aktivering.join()
        else:
            Status.threding_start(False)
            
            

       
        while(ivann_aktiv):

            '''
            data_aks = Status.get_data_aks()
            data_LFR_Preasure = Status.get_data_LFR_Preasure()
            data_puls = Status.get_data_pulse()
            '''

            if (True): # Denne må være tidsstyrt hvor ofte vi ønsker og oppdatere status, kanskje 1 gang i sekundet?
                if (Status.Flyter()):
                        print("Flyter")
                
                elif(Status.Svømmer()):
                        print("Svømmer")

                elif(Status.Dykker()):
                        print("Dykker")
                
                elif(Status.Svømmer_opp()):
                        print("Svømmer_opp")
                
                elif(Status.Drukner()):
                        print("Drukner")
                else:
                        print("Uvisst status")
            ivann_aktiv = Status.aktivert() # Denne må være i hovedprogrammet og bestemme om svømmeren er i vann eller ikke.
           
        
        


        
        


            


            


            





  

            

