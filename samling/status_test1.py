from mpu6050_1.mpu6050_2 import bachelor_akselrasjon_sensor as Akssensor
from ms5837 import Bachelor_test1_MS5837 as Trykksensor
from ArduinoAD8232ECG import Bachelor_Pulsmåling as pulssensor
from VL6180X.examples import bachelor_pustesenor as pustesensor 


class status: 

    def __init__(self):
        self.variabler = {} 
        self.trykk_sensor = Trykksensor.analyse_MS5837()
        self.puls_sensor = pulssensor()
        self.aks_sensor = Akssensor.MPU6050_Orientation()
        self.LFR_sensor = pustesensor.VL6180XAnalyser()

    
    def get_data_aks(self):
       # Denne skal retunere: 'total_G': tot_G, 'is_periodic': last_periodicity_status,'retning' : retning
        self.data_aks = self.aks_sensor.oppdater_og_vurder_status()
        
        return {"aks": self.data_aks}
    
    def get_data_LFR_Preasure(self):
         self.data_LFR = self.LFR_sensor.analyserer_stopp()
         self.data_preasure = self.trykk_sensor.read_sensor_data() 
        
         return {      
            "LFR": self.data_LFR
            "pressure": self.data_pressure
            }

    def get_data_pulse(self):
        self.data_pulse = self.puls_sensor.  # her må man få inn en klasse som retunerer noe brukende
        
        return {
            "pulse": self.data_pulse    
            }


    def Svømmer(self, aks, lfr_preasure, pulse):
        # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        pass

    def Flyter(self, aks, lfr_preasure, pulse):
         # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        pass
    
    def Dykker(self, aks, lfr_preasure, pulse):
        # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        pass

    def Svømmer_opp(self, aks, lfr_preasure, pulse):
         # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        pass
    
    def Drukner(self, aks, lfr_preasure, pulse):
        # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false 
        pass 

    def aktivert(selft):
        # Denne skal bestemme om svømmeren er i vann og aktivere status analyse.
        pass 

if __name__ == "__main__":
   
   while(True):
       ivann = aktivert() 

       while(ivann):
            data_aks = thread get data_aks
            data_LFR_Preasure = thread get data
            data_puls = thread get_data 
           

            if (True): # Denne må være tidsstyrt hvor ofte vi ønsker og oppdatere status, kanskje 1 gang i sekundet?
                if (Flyter()):
                        printline("Flyter")
                
                elif(Svømmer()):
                        printline("Svømmer")

                elif(Dykker()):
                        printline("Dykker")
                
                elif(Svømmer_opp()):
                        printline("Svømmer_opp")
                
                elif(Drukner()):
                        printline("Drukner")
                else:
                        printline("Uvisst status")
            
            ivann = aktivert() # Deaktiverer status oppdatering og programm program kall 

            

       
        


       
    
    
       


       


   
   



    

