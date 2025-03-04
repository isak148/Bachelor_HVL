import RPi.GPIO as GPIO
import time

data_pin = 17 # GPIO pin connected to Arduino data pin
delay_time = 0.001 #must match arduino delay time.

GPIO.setmode(GPIO.BCM)
GPIO.setup(data_pin, GPIO.IN)

try:
    while True:
        data = 0
        for i in range(10):
            bit = GPIO.input(data_pin)
            data |= (bit << i)
            time.sleep(delay_time)
        print("Sensor Value:", data)
        time.sleep(0.1) #delay between readings.

except KeyboardInterrupt:
    GPIO.cleanup()