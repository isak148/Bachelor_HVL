import subprocess
import threading
import time
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def ny_terminal(script_path):
    try:
        subprocess.Popen(["lxterminal","-e","python3",script_path])
        logging.info(f"Starter {script_path} i ny terminal.")
    except FileNotFoundError:
        logging.error(f"Fant ikke skriptet: {script_path}")
    except Exception as e:
        logging.error(f"Feil ved oppstart av terminal: {e}")

def vl6180x():
    logging.info("Funksjon 1 startet")
    ny_terminal("/home/Bachelor/Bachelor_HVL/VL6180X/examples/vl6180x_continuoustest.py")
    logging.info("Funksjon 1 er ferdig")
    time.sleep(0.1)

def MPU6050():
    logging.info("Funksjon 2 startet")
    ny_terminal("/home/Bachelor/Bachelor_HVL/mpu6050_1/mpu6050_2/Bachelor_test1.py")
    logging.info("Funksjon 2 er ferdig")
    time.sleep(1)

def MS5837():
    logging.info("Funksjon 3 startet")
    ny_terminal("/home/Bachelor/Bachelor_HVL/ms5837/example.py")
    logging.info("Funksjon 3 er ferdig")
    time.sleep(1)

def data_analyse():
    logging.info("Starter dataanalyse")
    ny_terminal("/home/Bachelor/Bachelor_HVL/samling\data_analyse.py")
    logging.info("data analyse er ferdig")
    time.sleep(0.1)

def ads1015():
    logging.info("Funksjon 5 startet")
    ny_terminal("/home/Bachelor/Bachelor_HVL/ArduinoAD8232ECG-master/ads1015.py")
    logging.info("Funksjon 5 er ferdig")
    time.sleep(0.1)

def oppstart():
    tred1 = threading.Thread(target=vl6180x)
    tred2 = threading.Thread(target=MPU6050)
    tred3 = threading.Thread(target=MS5837)
    tred4 = threading.Thread(target=data_analyse)
    tred5 = threading.Thread(target=ads1015)

    tred1.start()
    tred2.start()
    tred3.start()
    tred4.start()

    tred1.join()
    tred2.join()
    tred3.join()
    tred4.join()

    logging.info("Hovedtråd ferdig")

if __name__ == "__main__":
    oppstart()
    
#tanke for oppstart
""" 
def hovud():
    tred3 = threading.Thread(target=funksjon3)
    tred3.start()
    tred3.join()

    if sensor.pressure == 1100:
        oppstart()    
"""    
