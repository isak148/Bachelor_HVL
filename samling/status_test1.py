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
        self.data_preassure = {} # {'status' : self.retningsendring "String",   'Trykk': Trykk "float",   'I_vann': Ivann "bool",  'under_vann': under_vann "bool", 'Under_vann_30s': under_vann_30s "Bool"} 
        self.oppstart = False
        self.count = 0 
    
    def threding_start(self,verdi):
        if(verdi == True):
            self.oppstart = True
        else:
            self.oppstart = False
        
    def get_data_aks(self):
        # Denne skal retunere: 'total_G': tot_G, 'is_periodic': last_periodicity_status,'retning' : retning
        while(self.oppstart):
            self.data_aks = self.aks_sensor.oppdater_og_vurder_status()
        
    
    def get_data_LFR_Preasure(self):
        while(self.oppstart):
            self.data_LFR = self.LFR_sensor.analyserer_stopp()
            self.data_preassure = self.trykk_sensor.read_sensor_data() 
                 

    def get_data_pulse(self):
        while(self.oppstart):
            self.data_pulse = self.puls_sensor.get_data()  # her må man få inn en klasse som retunerer noe brukende
            
         
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
              Gyro_Status_Stille = True
              Gyro_Status_Moderat = False
              Gyro_status_Høy = False
         elif(Gyro_Status == "Moderat"):
              Gyro_Status_Stille = False
              Gyro_Status_Moderat = True
              Gyro_status_Høy = False
         elif(Gyro_Status == "høy" ):
              Gyro_Status_Stille = False
              Gyro_Status_Moderat = False
              Gyro_status_Høy = True
         else:
              Gyro_Status_Stille = False
              Gyro_Status_Moderat = False
              Gyro_status_Høy = False

         Pust_Status = self.data_LFR['Pust_Status']
         if (Pust_Status == "Puster" ):
              Pust_Status_Puster = True
              Pust_Status_Puste_stopp = False
              Pust_Status_Puster_Ikke = False
         elif (Pust_Status == "Puste_Stopp"):
              Pust_Status_Puster = False
              Pust_Status_Puste_stopp = True
              Pust_Status_Puster_Ikke = False
              
         elif (Pust_Status == "Puster_Ikke"):
              Pust_Status_Puster = False
              Pust_Status_Puste_stopp = False
              Pust_Status_Puster_Ikke = True
         else:
              Pust_Status_Puster = False
              Pust_Status_Puste_stopp = False
              Pust_Status_Puster_Ikke = False
              
         Puste_Frekvens = self.data_LFR['Pust_Frekvens']
         if (Puste_Frekvens == "Lav" ):
              Pust_Frekvens_Lav = True
              Pust_Frekvens_Normal = False
              Pust_Frekvens_Høy = False 
         elif (Puste_Frekvens == "Normal"):
              Pust_Frekvens_Lav = True
              Pust_Frekvens_Normal = False
              Pust_Frekvens_Høy = False
         elif (Puste_Frekvens == "Høy"):
              Pust_Frekvens_Lav = True
              Pust_Frekvens_Normal = False
              Pust_Frekvens_Høy = False
         else:
              Pust_Frekvens_Lav = False
              Pust_Frekvens_Normal = False
              Pust_Frekvens_Høy = False

         Puls_Status = self.data_pulse['Puls_Status']
         if (Puls_Status == "Lav"):
              Puls_Status_Lav = True
              Puls_Status_Middel = False
              Puls_Status_Høy = False
         elif (Puls_Status == "Middel"):
              Puls_Status_Lav = False
              Puls_Status_Middel = True
              Puls_Status_Høy = False
         elif (Puls_Status == "Høy"):
              Puls_Status_Lav = False
              Puls_Status_Middel = False
              Puls_Status_Høy = True
         else:
              Puls_Status_Lav = False
              Puls_Status_Middel = False
              Puls_Status_Høy = False
        
         I_vann = self.data_preassure['I_vann'] #Denne returnerer bare True False
         
         Under_vann = self.data_preassure['Under_vann'] #Denne returnerer bare True, False

         Under_Vann_30s = self.data_preassure["Under_Vann_30s"] # Denne returnerer true false 
         
         Retningsendring = self.data_preassure['Retningsendring']
         if (Retningsendring == "Uendret"):
               Retningsendring_Uendret = True
               Retningsendring_Synkende = False
               Retningsendring_Økende = False
         elif (Retningsendring == "Synkende"):
               Retningsendring_Uendret = False
               Retningsendring_Synkende = True
               Retningsendring_Økende = False
         elif (Retningsendring == "Økende"):
               Retningsendring_Uendret = False
               Retningsendring_Synkende = False
               Retningsendring_Økende = True
         else:
               Retningsendring_Uendret = False
               Retningsendring_Synkende = False
               Retningsendring_Økende = False
        
         return {
              'Retning_Opp': Retning_Opp, 
              'Retning_Plan': Retning_Plan,
              'Retning_Ned': Retning_Ned,
              'Aks_Status_Stille': Aks_Status_Stille,
              'Aks_Status_Moderat': Aks_Status_Moderat,
              'Aks_Status_Høy': Aks_status_Høy,
              'Gyro_Status_Stille': Gyro_Status_Stille,
              'Gyro_Status_Moderat': Gyro_Status_Moderat,
              'Gyro_Status_Høy': Gyro_status_Høy,
              'Pust_Status_Puster': Pust_Status_Puster,
              'Pust_Status_Puste_stopp': Pust_Status_Puste_stopp,
              'Pust_Status_Puster_Ikke': Pust_Status_Puster_Ikke,
              'Pust_Frekvens_Lav': Pust_Frekvens_Lav,  
              'Pust_Frekvens_Normal': Pust_Frekvens_Normal,
              'Pust_Frekvens_Høy': Pust_Frekvens_Høy,
              'Puls_Status_Lav': Puls_Status_Lav,
              'Puls_Status_Middel': Puls_Status_Middel,
              'Puls_Status_Høy': Puls_Status_Høy,  
              'I_vann': I_vann,
              'Under_vann': Under_vann,
              'Under_vann_30s': Under_Vann_30s,
              'Retningsendring_Uendret': Retningsendring_Uendret,
              'Retningsendring_Synkende': Retningsendring_Synkende,
              'Retningsendring_Økende': Retningsendring_Økende

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
            'Pust_Status': status_fra_pust,
            'Pust_Frekvens': puste_frekvens
        }
    '''
    ''' PULS
    return {'puls': median_bpm,
                    'Puls_Status': self.puls_status}
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
        Data = self.get_data_bool()

        if ((Data['Retning_Opp'] == True | Data['Retning_Plan'] == True) &
            (Data['Aks_Status_Moderat'] == True | Data['Aks_Status_Høy'] == True) &
            (Data['Gyro_Status_Moderat'] == True | Data['Gyro_Status_Høy'] == True) &
            (Data['Retningsendring_Uendret'] == True) &
            (Data['Under_vann'] == False) &
            (Data['Puls_Status_Middel'] == True | Data['Puls_Status_Høy'] == True) &
            (Data['Pust_Frekvens_Normal'] == True | Data['Pust_Frekvens_Høy'] == True) &
            (Data['Pust_Status_Puster'] == True)):
            status = True
        else:
            status = False
        return status

    def Flyter(self):
        # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        Data = self.get_data_bool()
        if ((Data['Retning_Opp'] == True | Data['Retning_Plan'] == True) &
            (Data['Aks_Status_Stille'] == True) &
            (Data['Gyro_Status_Stille'] == True) &
            (Data['Retningsendring_Uendret'] == True) &
            (Data['Under_vann'] == False) &
            (Data['Puls_Status_Lav'] == True | Data['Puls_Status_Middel'] == True) &
            (Data['Pust_Frekvens_Lav'] == True | Data['Pust_Frekvens_Normal'] == True) &
            (Data['Pust_Status_Puster'] == True)):
            status = True
        else:
            status = False
        return status
    
    def Dykker(self):
        # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        Data = self.get_data_bool()
        if (( Data['Retning_Plan'] == True | Data['Retning_Ned'] == True) &
            (Data['Aks_Status_Moderat'] == True | Data['Aks_Status_Høy'] == True) &
            (Data['Gyro_Status_Moderat'] == True | Data['Gyro_Status_Høy'] == True) &
            (Data['Retningsendring_Uendret'] == True | Data['Retningsendring_Økende'] == True) &
            (Data['Under_vann'] == True) &
            (Data['Puls_Status_Lav'] == True | Data['Puls_Status_Middel'] == True | Data['Puls_Status_Høy'] == True) &
            (Data['Pust_Frekvens_Lav'] == True) &
            (Data['Pust_Status_Puste_stopp'] == True)):
            status = True
        else:
            status = False
       
        return status
        

    def Svømmer_opp(self):
         # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false
        Data = self.get_data_bool()
        if ((Data['Retning_Opp'] == True) &
            (Data['Aks_Status_Moderat'] == True | Data['Aks_Status_Høy'] == True) &
            (Data['Gyro_Status_Moderat'] == True | Data['Gyro_Status_Høy'] == True) &
            (Data['Retningsendring_Synkende'] == True) &
            (Data['Under_vann'] == True) &
            (Data['Puls_Status_Lav'] == True | Data['Puls_Status_Middel'] == True | Data['Puls_Status_Høy'] == True) &
            (Data['Pust_Frekvens_Lav'] == True) &
            (Data['Pust_Status_Puste_stopp'] == True)):
            status = True
        else:
            status = False
        
        return status
    
    
    def Drukner(self):
        # Denne skal mota status fra hovedprogrammet gjennom dict og retunere
        # True eller false 
        Data = self.get_data_bool()
        if ((Data['Retning_Opp'] == True | Data['Retning_Opp'] == True) &
            (Data['Aks_Status_Lav'] == True | Data['Aks_Status_Moderat'] == True | Data['Aks_Status_Høy'] == True) &
            (Data['Gyro_Status_Lav'] == True |Data['Gyro_Status_Moderat'] == True | Data['Gyro_Status_Høy'] == True) &
            (Data['Retningsendring_Uendret'] == True | Data['Retningsendring_Synkende'] == True | Data['Retningsendring_Økende'] == True) &
            (Data['Under_vann'] == True | Data['Under_vann'] == False) &
            (Data['Puls_Status_Lav'] == True | Data['Puls_Status_Middel'] == True | Data['Puls_Status_Høy'] == True) &
            (Data['Pust_Frekvens_Lav'] == True) &
            (Data['Pust_Status_Puste_stopp'] == True) |  # OR om scømmeren ikke har pustet på lenge
            (Data['Pust_Status_Puster_Ikke'] == True) |  # OR om svømmeren har vært under vann for lenge
            (Data['Under_Vann_30s'] == True)):
            status = True
        else:
            status = False 
       
        return status
    
    def Initialiserer(self):
        # skal retunere initaialiserer viss en sensor returnerer initialiserer eller ubestemt under oppstartsfasen.
        Data = self.get_data_bool()
        if ():
              pass
        else:
              pass
        
        return False

        
    def aktivert(self):
        '''Metoden sjekker om trykksensoren registrerer vann når den er i initial tilstanden False. 
         hvis sensoren kommer 2cm under vann vil den starte opp alle sensorene med threads.'''
        # Denne skal bestemme om svømmeren er i vann og aktivere status analyse.
        if self.ivann == False: # Sjekker om 
            I_vann = self.trykk_sensor.read_sensor_data(['I_vann']) 
            if (I_vann == True):
                self.ivann = True
        
        return self.ivann 
    
    def Deaktivert(self):
        '''Kjører hvert sekund og hvis vesten ikke registrere vann på 10 sekunder så vil den deaktiveres'''
        if (self.data_preassure['I_vann'] == False):
            self.count +=1
        else:
            self.count = 0
        
        if(self.count >= 10):
             self.ivann = False 
        
        return self.ivann
             

        

if __name__ == "__main__":
    status = Status()
    while True:
        ivann_aktiv =status.aktivert() # Sjekker om svømmeren er i vann har 0.5s sleep
       
        if (ivann_aktiv == True):
            status.threding_start(True)
            aks_thread = threading.Thread(target=status.get_data_aks, daemon=True)
            trykk_thread = threading.Thread(target=status.get_data_LFR_Preasure, daemon=True)
            puls_thread = threading.Thread(target=status.get_data_pulse, daemon=True)

            aks_thread.start()
            trykk_thread.start()
            puls_thread.start()
        else:
            status.threding_start(False)
            

       
        while(ivann_aktiv):

            '''
            data_aks = Status.get_data_aks()
            data_LFR_Preasure = Status.get_data_LFR_Preasure()
            data_puls = Status.get_data_pulse()
            '''
            time.sleep(1) # Oppdaterer status hvert sekund, utfra oppdatert sensor data. 
            if (True): # Denne må være tidsstyrt hvor ofte vi ønsker og oppdatere status, kanskje 1 gang i sekundet?
                if (status.Flyter()):
                        print("Flyter")
                
                elif(status.Svømmer()):
                        print("Svømmer")

                elif(status.Dykker()):
                        print("Dykker")
                
                elif(status.Svømmer_opp()):
                        print("Svømmer_opp")
                
                elif(status.Drukner()):
                        print("Drukner")
                else:
                        print("Uvisst status / Initialiserer")
            
            ivann_aktiv = status.Deaktivert() # Sjekker om svømmeren fortsatt er i vann.
           
        
        


        
        


            


            


            





  

            

