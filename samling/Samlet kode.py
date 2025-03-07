import VL6180X as VL6180X
import mpu6050_1 as mpu6050
import ms5837 as ms5837
import subprocess

subprocess.run(["python3", "VL6180X.py"])
subprocess.run(["python3", "mpu6050.py"])  
subprocess.run(["python3", "ms5837.py"])
