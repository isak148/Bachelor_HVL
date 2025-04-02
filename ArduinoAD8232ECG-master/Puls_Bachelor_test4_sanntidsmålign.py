import time
import statistics
import board
import busio
import adafruit_ads1x15.ads1015 as ADS1015
import adafruit_ads1x15.analog_in as AnalogIn

# --- Konfigurasjon ---
THRESHOLD = 26000
DEBOUNCE_TIME = 0.25  # sekunder
MAX_HISTORY = 5       # antall verdier å ta median av

# --- Init ADC ---
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS1015.ADS1015(i2c)
channel = AnalogIn.AnalogIn(adc, ADS1015.P0)

print("Venter på første topp...")

try:
    last_peak_time = None
    last_value = 0
    bpm_history = []

    while True:
        value = channel.value
        now = time.time()

        # Detekter ny topp (går over terskel og var under forrige gang)
        if value > THRESHOLD and last_value <= THRESHOLD:
            if last_peak_time is None:
                last_peak_time = now
                print("📍 Første topp registrert")
            else:
                rr_interval = now - last_peak_time
                if rr_interval > DEBOUNCE_TIME:
                    bpm = 60 / rr_interval
                    bpm_history.append(bpm)
                    if len(bpm_history) > MAX_HISTORY:
                        bpm_history.pop(0)

                    median_bpm = statistics.median(bpm_history)
                    print(f"✅ Puls (median av siste {len(bpm_history)}): {median_bpm:.1f} BPM")

                    last_peak_time = now

        last_value = value

except KeyboardInterrupt:
    print(" Avslutter måling.")
