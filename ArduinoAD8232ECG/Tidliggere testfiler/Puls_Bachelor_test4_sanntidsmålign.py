import time
import statistics
import board
import busio
import adafruit_ads1x15.ads1015 as ADS1015
import adafruit_ads1x15.analog_in as AnalogIn
class AnalyseAD8232:
    def __init__(self):
        # Initialiserer I2C og ADC
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.adc = ADS1015.ADS1015(self.i2c)
        self.channel = AnalogIn.AnalogIn(self.adc, ADS1015.P0)
        self.THRESHOLD = 26000
        self.DEBOUNCE_TIME = 0.25  # sekunder
        self.MAX_HISTORY = 5       # antall verdier å ta median av
        self.last_peak_time = None
        self.last_value = 0
        self.bpm_history = []
        self.puls_status = None

    def read_sensor(self):
        # Leser av verdien fra ADC-kanalen
        return self.channel.value
    
    def get_data(self):
            
            value = self.channel.value
            now = time.time()

            # Detekter ny topp (går over terskel og var under forrige gang)
            if value > self.THRESHOLD and last_value <= self.THRESHOLD:
                if last_peak_time is None:
                    last_peak_time = now
                    print("Første topp registrert")
                else:
                    rr_interval = now - last_peak_time
                    if rr_interval > self.DEBOUNCE_TIME:
                        bpm = 60 / rr_interval
                        self.bpm_history.append(bpm)
                        if len(self.bpm_history) > self.MAX_HISTORY:
                            self.bpm_history.pop(0)

                        median_bpm = statistics.median(self.bpm_history)
                        print(f"Puls (median av siste {len(self.bpm_history)}): {median_bpm:.1f} BPM")

                        last_peak_time = now
                
            if (median_bpm <= 100):
                self.puls_status = "lav"
            elif (100 > median_bpm > 150):
                self.puls_status = "middel"
            else:
                self.puls_status = "høy"

            last_value = value
            time.sleep(0.01)

            return {'puls': median_bpm,
                    'puls_status': self.puls_status}

   
