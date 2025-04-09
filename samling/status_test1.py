from mpu6050_1.mpu6050_2.bachelor_test2 import MPU6050_Orientation
from ms5837 import Bachelor_test1_MS5837 as Trykksensor

class status: 

    def __init__(self):
        self.variabler = {} 
        self.trykk_sensor = Trykksensor.analyse_MS5837() 

    
    def get_data_aks(self):
       # Denne skal retunere: 'total_G': tot_G, 'is_periodic': last_periodicity_status,'retning' : retning
        self.data_aks = MPU6050_Orientation.gi_status_aks()
        
        return {"aks": self.data_aks}
    
    def get_data_LFR_Preasure(self):
         self.data_LFR = 
         self.data_preasure = self.trykk_sensor.read_sensor_data() 
        return {      
            "LFR": self.data_LFR,   
            "pressure": self.data_pressure
            }

    def get_data_pulse(self):
        self.data_pulse
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

            

       
        


       
    
    
       


       


   
   



    

