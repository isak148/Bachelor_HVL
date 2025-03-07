import subprocess
import threading
import time

def funksjon1():
    print("Funksjon 1 startet")
    subprocess.run(["python3", "/home/Bachelor/Bachelor_HVL/VL6180X/examples/vl6180x_continuoustest.py"])
    print("Funksjon 1 er ferdig")
    time.sleep(0.1)

def funksjon2():
    print("Funksjon 2 startet")
    subprocess.run(["python3", "/home/Bachelor/Bachelor_HVL/mpu6050_1/mpu6050_2/Bachelor_test1.py"])
    print("Funksjon 2 er ferdig")
    time.sleep(1)

def funksjon3():
    print("Funksjon 3 startet")
    subprocess.run(["python3", "/home/Bachelor/Bachelor_HVL/ms5837/example.py"])
    print("Funksjon 3 er ferdig")
    time.sleep(1)

tred1 = threading.Thread(target=funksjon1)
tred2 = threading.Thread(target=funksjon2)
tred3 = threading.Thread(target=funksjon3)

tred1.start()
tred3.start()
tred2.start()

tred1.join()
tred2.join()
tred3.join()

print("Hovedtråd ferdig")