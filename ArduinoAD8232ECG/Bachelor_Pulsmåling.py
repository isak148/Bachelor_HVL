import time
import statistics
import board
import busio
import adafruit_ads1x15.ads1015 as ADS1015
import adafruit_ads1x15.analog_in as AnalogIn
import csv

class AnalyseAD8232:
    def __init__(self):
        # Initialiserer I2C og ADC
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.adc = ADS1015.ADS1015(self.i2c)
        self.channel = AnalogIn.AnalogIn(self.adc, ADS1015.P0)
        self.threshold = 26000
        self.debounce_time = 0.25  # sekunder
        self.max_history = 5       # antall verdier å ta median av
        self.last_peak_time = None
        self.last_value = 0
        self.bpm_history = []
        self.puls_status = None
        self.median_bpm=0

    def read_sensor(self):
        # Leser av verdien fra ADC-kanalen
        return self.channel.value
    
    def get_data(self, Lagre = False, Filnavn = ""):
            
            value = self.channel.value
            now = time.time()
            

            # Detekter ny topp (går over terskel og var under forrige gang)
            if value > self.threshold and self.last_value <= self.threshold:
                if self.last_peak_time is None:
                    self.last_peak_time = now
                    #print("Første topp registrert")
                else:
                    rr_interval = now - self.last_peak_time
                    if rr_interval > self.debounce_time:
                        bpm = 60 / rr_interval
                        self.bpm_history.append(bpm)
                        if len(self.bpm_history) > self.max_history:
                            self.bpm_history.pop(0)

                        self.median_bpm = statistics.median(self.bpm_history)
                        #print(f"Puls (median av siste {len(self.bpm_history)}): {median_bpm:.1f} BPM")

                        self.last_peak_time = now
                
            if (self.median_bpm <= 100):
                self.puls_status = "Lav"
            elif (100 > self.median_bpm > 150):
                self.puls_status = "Middel"
            else:
                self.puls_status = "Høy"

            self.last_value = value

            if (Lagre == True):
                self.skriv_til_fil(Filnavn, value)

            time.sleep(0.01)
            print(self.puls_status) # Feilsøking: 
            print(self.median_bpm) # Feilsøking
            return {'Puls': self.median_bpm,
                    'Puls_Status': self.puls_status}
    
    def skriv_til_fil(self, filnavn, verdi):
                if not filnavn.endswith(".csv"):
                        filnavn += ".csv"  # Legger til .csv hvis det mangler

                with open(filnavn, mode='a', newline='', encoding='utf-8') as fil:
                        writer = csv.writer(fil)
                        writer.writerow([verdi])  # Skriver én tallverdi på ny linje


if __name__ == "__main__":
   
    sensor = AnalyseAD8232()
    while(True):
        sensor.get_data()
        time.sleep(0.01)