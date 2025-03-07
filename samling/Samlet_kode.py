import subprocess
import threading
import time 

def funksjon1():
    subprocess.run(["python3", "/home/Bachelor/Bachelor_HVL/VL6180X/examples/vl6180x_continuoustest.py"])
    print("er ferdig")
    time.sleep(0.1)

def funksjon2():
    subprocess.run(["python3", "/home/Bachelor/Bachelor_HVL/mpu6050_1/mpu6050_2/Bachelor_test1.py"])  
    print("er ferdig")
    time.sleep(1)

def funksjon3():
    subprocess.run(["python3", "/home/Bachelor/Bachelor_HVL/ms5837/example.py"])
    
    print("er ferdig")
    time.sleep(1)

tred1 = threading.Thread(target=funksjon1)
tred2 = threading.Thread(target=funksjon2)
tred3 = threading.Thread(target=funksjon3)

tred1.start
tred3.start
tred2.start

tred1.join
tred2.join
tred3.join 

print("hovud tråd ferdig")

