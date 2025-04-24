from mpu6050_1.mpu6050_2 import bachelor_akselrasjon_sensor as Akssensor
from ms5837 import Bachelor_Trykksensor as Trykksensor
from ArduinoAD8232ECG.Bachelor_Pulsmåling import AnalyseAD8232 as pulssensor
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
        self.data_aks = {} #'total_G': tot_G "float",   'total_Gyro': tot_Gyro "float",     'aks_status': status_fra_G "string",     'gyro_status': status_fra_Gyro "string"
        self.data_LFR = {} # 'pust_status': status_fra_pust, 'pust_frekvens': puste_frekvens "initialiserer, lav, middels, høy"
        self.data_pulse = {} # {'puls': median_bpm "float",   'puls_status': self.puls_status "lav, middels, høy"}
        self.data_preassure = {} # {'status' : self.retningsendring "String",   'Trykk': Trykk "float",   'I_vann': Ivann "bool",  'under_vann': under_vann "bool"} 
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
        
            return {self.data_aks}
    
    def get_data_LFR_Preasure(self):
        while(self.oppstart):
            self.data_LFR = self.LFR_sensor.analyserer_stopp()
            self.data_preassure = self.trykk_sensor.read_sensor_data() 
            
            return {      
                self.data_LFR,
                self.data_pressure
                }

    def get_data_pulse(self):
        while(self.oppstart):
            self.data_pulse = self.puls_sensor.get_data()  # her må man få inn en klasse som retunerer noe brukende
            
            return {
                self.data_pulse    
                }
        
    def get_data_bool(self):
         Retning = self.data_aks['Retning']
         if (Retning == "Opp"):
              Retning_Opp = True
              Retning_Plan = False
              Retning_Ned = False 
         elif (Retning == "Plan"):
              Retning_Opp = False
              Retning_Plan = True
              Retning_Ned = False
         elif (Retning == "Ned"):
              Retning_Opp = False
              Retning_Plan = False
              Retning_Ned = True
         else:
              Retning_Opp = False
              Retning_Plan = False
              Retning_Ned = False
              
         Aks_Status = self.data_aks['Aks_Status']
         if(Aks_Status == "Stille"):
              Aks_Status_Stille = True
              Aks_Status_Moderat = False
              Aks_status_Høy = False

         elif(Aks_Status == "Moderat"):
              Aks_Status_Stille = False
              Aks_Status_Moderat = True
              Aks_status_Høy = False
         elif(Aks_Status == "høy" ):
              Aks_Status_Stille = False
              Aks_Status_Moderat = False
              Aks_status_Høy = True
         else:
              Aks_Status_Stille = False
              Aks_Status_Moderat = False
              Aks_status_Høy = False
               
         Gyro_Status = self.data_aks["Gyro_Status"]  
         if(Gyro_Status == "Stille"):
              Aks_Status_Stille = True
              Aks_Status_Moderat = False
              Aks_status_Høy = False

         elif(Gyro_Status == "Moderat"):
              Aks_Status_Stille = False
              Aks_Status_Moderat = True
              Aks_status_Høy = False
         elif(Gyro_Status == "høy" ):
              Aks_Status_Stille = False
              Aks_Status_Moderat = False
              Aks_status_Høy = True
         else:
              Aks_Status_Stille = False
              Aks_Status_Moderat = False
              Aks_status_Høy = False

         Pust_Status = self.data_LFR['pust_status']
         if (Pust_Status == "Lav" ):
              pass
         elif (Pust_Status == "Normal"):
              pass
         elif (Pust_Status == "Høy"):
              pass
         else
              


         Puste_Frekvens = self.data_LFR['pust_frekvens']
          if (Puste_Frekvens == "Lav" ):
              pass
         elif (Puste_Frekvens == "Normal"):
              pass
         elif (Puste_Frekvens == "Høy"):
              pass
         else:

         Puls_Status = self.data_pulse['puls_status']

         I_vann = self.data_preassure['I_vann']
         Under_vann = self.data_preassure['Under_vann']

         return {
              'Retning': Retning,
              'Aks_Status' : Aks_Status,
              'Gyro_Status': Gyro_Status,
              'Pust_Status': Pust_Status,
              'Puste_frekvens': Puste_Frekvens,
              'Puls_Status': puls_status,
              'I_vann': I_vann,
              'Under_vann': Under_vann
         }



    ''' AKS 
          return {
            'Total_G': tot_G,
            'Total_Gyro': tot_Gyro,
            'Aks_Status': status_fra_G,
            'Gyro_Status': status_fra_Gyro,
            'Retning': retning
        }
    '''     
   
    ''' LFR
     return {        
            'pust_status': status_fra_pust,
            'pust_frekvens': puste_frekvens
        }
    '''

    ''' PULS
    return {'puls': median_bpm,
                    'puls_status': self.puls_status}
    '''
    ''' PUST
     return   {'Retningsendring' : self.retningsendring,
                         'Trykk': Trykk,
                         'I_vann': Ivann,
                         'Under_vann': under_vann} 

    '''




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
    
    def Initialiserer(self):
        # skal retunere initaialiserer viss en sensor returnerer initialiserer eller ubestemt under oppstartsfasen.
         pass

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
           
        
        


        
        


            


            


            





  

            

