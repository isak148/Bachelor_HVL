from mpu6050_1.mpu6050_2 import bachelor_akselrasjon_sensor as Akssensor
from ms5837 import Bachelor_Trykksensor as Trykksensor
from ArduinoAD8232ECG.Bachelor_Pulsmåling import AnalyseAD8232 as pulssensor
from VL6180X.examples import bachelor_pustesenor as pustesensor 
import threading
import time
import csv


class Status: 

    def __init__(self): # kjører init, init i sensor programmer vil også bli kjørt før programmet starter. 
        self.variabler = {} 
        self.trykk_sensor = Trykksensor.analyse_MS5837() 
        self.puls_sensor = pulssensor()
        self.aks_sensor = Akssensor.MPU6050_Orientation(0x68)
        self.LFR_sensor = pustesensor.VL6180XAnalyser()
        self.ivann = False
        self.data_aks = {} #'total_G': tot_G "float",   'total_Gyro': tot_Gyro "float",     'aks_status': status_fra_G "string",     'gyro_status': status_fra_Gyro "string"
        self.data_LFR = {} # 'pust_status': status_fra_pust, 'pust_frekvens': puste_frekvens "initialiserer, lav, middels, høy"
        self.data_pulse = {} # {'puls': median_bpm "float",   'puls_status': self.puls_status "lav, middels, høy"}
        self.data_preassure = {} # {'status' : self.retningsendring "String",   'Trykk': Trykk "float",   'I_vann': Ivann "bool",  'under_vann': under_vann "bool", 'Under_vann_30s': under_vann_30s "Bool"} 
        self.oppstart = False
        self.count = 0 
        
   
    def threding_start(self,verdi):
        '''
        Threading_start holder trådene i while løkken slik at de kontinuerlig oppdaterer variablene for sensordata. Kan settes til false i 
        hovedprogrammet av trykk-sensor slik at alle sensorene ikke går kontinuerlig, dette for og optimalisere strømforbruk.
        '''
        if(verdi == True):
            self.oppstart = True
        else:
            self.oppstart = False

      
    def get_data_aks(self):
        '''Kjører program for MPU og oppdaterer aks og gyro variabeler, kontinuerlig'''
        interval = 0.01  # 100 Hz → 10 ms
        next_time = time.perf_counter()
        while(self.oppstart):
            self.data_aks = self.aks_sensor.oppdater_og_vurder_status(Lagre=True, Filnavn="Akselerasjon5") # True for lagre og ønsket filnavn
            next_time += interval
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
        
    
    def get_data_LFR(self): 
        '''Kjører program for LRF og oppdaterer puste variabeler kontinuerlig'''
        interval = 0.05  # 20 Hz → 50 ms
        next_time = time.perf_counter()
        while(self.oppstart):
            self.data_LFR = self.LFR_sensor.analyserer_stopp(Lagre=True, Filnavn="Puste_frekvens5")          # True for lagre og ønsket filnavn
            next_time += interval
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)   

    def get_data_Preasure(self):
        '''Kjører program for Trykksensor og oppdaterer trykk variabeler kontinuerlig''' 
        interval = 0.5 # 20 Hz → 50 ms
        next_time = time.perf_counter()
        while(self.oppstart):
            self.data_preassure = self.trykk_sensor.read_sensor_data(Lagre=True, Filnavn="Trykk5")  # True for lagre og ønsket filnavn
            next_time += interval
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)   


    def get_data_pulse(self):
        '''Kjører program for Puls og oppdaterer puls kontinuerlig'''
        interval = 0.01  # 100 Hz → 10 ms
        next_time = time.perf_counter()
        while(self.oppstart):
            self.data_pulse = self.puls_sensor.get_data(Lagre=True, Filnavn="Puls5")               # True for lagre og ønsket filnavn
            next_time += interval
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)   

            
         
    def get_data_bool(self):
         '''Denne konverterer alle variabler fra sensorer til boolske verdier og returnerer dem som True eller False. Med og
          bruke de felles variablene som blir oppdatert av trådene. '''
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
              Pust_Frekvens_Lav = False
              Pust_Frekvens_Normal = True
              Pust_Frekvens_Høy = False
         elif (Puste_Frekvens == "Høy"):
              Pust_Frekvens_Lav = False
              Pust_Frekvens_Normal = False
              Pust_Frekvens_Høy = True
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
        
         I_vann = self.data_preassure['I_Vann'] #Denne returnerer bare True False
         
         Under_vann = self.data_preassure['Under_Vann'] #Denne returnerer bare True, False

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
              'Under_Vann_30s': Under_Vann_30s,
              'Retningsendring_Uendret': Retningsendring_Uendret,
              'Retningsendring_Synkende': Retningsendring_Synkende,
              'Retningsendring_Økende': Retningsendring_Økende

         }



    def Svømmer(self):
        '''Definerer logikk for status svømmer'''
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
        '''Definerer logikk for status Flyter'''
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
        '''Definerer logikk for status Dykker'''
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
        '''Definerer logikk for status Svømmer_opp'''
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
        '''Definerer logikk for status drukner''' 
        Data = self.get_data_bool()
        # Kritisk overstyring - uansett andre forhold.
        if (Data['Pust_Status_Puster_Ikke'] | Data['Under_Vann_30s']):
            return True 

        if ((Data['Retning_Opp'] == True | Data['Retning_Opp'] == True) &
            (Data['Aks_Status_Stille'] == True | Data['Aks_Status_Moderat'] == True | Data['Aks_Status_Høy'] == True) &
            (Data['Gyro_Status_Stille'] == True |Data['Gyro_Status_Moderat'] == True | Data['Gyro_Status_Høy'] == True) &
            (Data['Retningsendring_Uendret'] == True | Data['Retningsendring_Synkende'] == True | Data['Retningsendring_Økende'] == True) &
            (Data['Under_vann'] == True | Data['Under_vann'] == False) &
            (Data['Puls_Status_Lav'] == True | Data['Puls_Status_Middel'] == True | Data['Puls_Status_Høy'] == True) &
            (Data['Pust_Frekvens_Lav'] == True) &
            (Data['Pust_Status_Puste_stopp'] == True)):     
            status = True
        else:
            status = False 
       
        return status
    
        
    def Aktivert(self):
        '''Metoden sjekker om trykksensoren registrerer vann når den er i initial tilstanden False. 
         hvis sensoren kommer 2cm under vann vil den starte opp alle sensorene med threads.'''
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
    

    def skriv_til_fil(self, filnavn, verdi):
                '''Skriver status til en fil i CSV format'''
                if not filnavn.endswith(".csv"):
                        filnavn += ".csv"  # Legger til .csv hvis det mangler

                with open(filnavn, mode='a', newline='', encoding='utf-8') as fil:
                        writer = csv.writer(fil)
                        writer.writerow([verdi])  # Skriver én tallverdi på ny linje
             

# Hovedprogram som bruker logisk styring med trykksensor       
'''
if __name__ == "__main__":
    status = Status()
    therding_running = False # Sjekker om trådene er startet opp eller ikke.
    while True:
        ivann_aktiv =status.Aktivert() # Sjekker om svømmeren er i vann har 0.5s sleep
       
        if ivann_aktiv and not therding_running:
            status.threding_start(True)
            aks_thread = threading.Thread(target=status.get_data_aks, daemon=True)
            trykk_thread = threading.Thread(target=status.get_data_LFR_Preasure, daemon=True)
            puls_thread = threading.Thread(target=status.get_data_pulse, daemon=True)

            aks_thread.start()
            trykk_thread.start()
            puls_thread.start()
            therding_running = True
            
        elif not ivann_aktiv and therding_running:
            status.threding_start(False)
            therding_running = False

       
        while(ivann_aktiv):

         
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
'''
           
# Test program som kjører i 60 sekunder, trenger ikke aktivering fra trykksensor.        
if __name__ == "__main__":
    status = Status()
    print("starter hovedtråd")

    # Start trådene umiddelbart (ingen sjekk for "i vann")
    status.threding_start(True)
    aks_thread = threading.Thread(target=status.get_data_aks, daemon=True)
    trykk_thread = threading.Thread(target=status.get_data_Preasure, daemon=True)
    puls_thread = threading.Thread(target=status.get_data_pulse, daemon=True)
    pust_thread = threading.Thread(target=status.get_data_LFR, daemon=True)

    aks_thread.start()
    trykk_thread.start()
    puls_thread.start()
    pust_thread.start()

    # Start testklokke
    start_tid = time.time()
    kjøretid = 60  # sekunder
    count = 0

    while count<=kjøretid:
        time.sleep(1)

        if status.Flyter():
            print("Flyter")
            status.skriv_til_fil("status5", "Flyter")
        elif status.Svømmer():
            print("Svømmer")
            status.skriv_til_fil("status5", "svømmer")
        elif status.Dykker():
            print("Dykker")
            status.skriv_til_fil("status5", "Dykker")
        elif status.Svømmer_opp():
            print("Svømmer opp")
            status.skriv_til_fil("status5", "Svømmer opp")
        elif status.Drukner():
            print("Drukner")
            status.skriv_til_fil("status5", "Drukner")
        else:
            print("Uvisst status / Initialiserer")
            status.skriv_til_fil("status5", "Uvisst status / Initialiserer")
        count+=1
    print("Testmodus avsluttet etter 60 sekunder.")     


        
        


            


            


            





  

            

