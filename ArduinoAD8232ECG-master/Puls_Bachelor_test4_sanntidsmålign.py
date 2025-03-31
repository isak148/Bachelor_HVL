import time
import board
import busio
import adafruit_ads1x15.ads1015 as ADS1015
import adafruit_ads1x15.analog_in as AnalogIn

# --- Konfigurasjon ---
THRESHOLD = 26000
DEBOUNCE_TIME = 0.25  # sekunder for å unngå dobbelttelling (filter)

# --- Init ADC ---
i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS1015.ADS1015(i2c)
channel = AnalogIn.AnalogIn(adc, ADS1015.P0)

print("Venter på første topp...")

try:
    last_peak_time = None
    last_value = 0

    while True:
        value = channel.value
        now = time.time()

        # Enkel toppdeteksjon: verdi går over terskel og var lav før
        if value > THRESHOLD and last_value <= THRESHOLD:
            if last_peak_time is None:
                # Første topp: start timing
                last_peak_time = now
                print("📍 Første topp registrert")
            else:
                # Andre topp: regn puls
                rr_interval = now - last_peak_time
                if rr_interval > DEBOUNCE_TIME:
                    bpm = 60 / rr_interval
                    print(f"✅ Estimert puls: {bpm:.1f} BPM")
                    last_peak_time = now  # start ny måling

        last_value = value

except KeyboardInterrupt:
    print("Avslutter måling.")
