import serial
import time

ser = serial.Serial('COM3', 921600)

filename = "adaptive_off.txt"

start_time = time.time()

with open(filename, "w") as f:
    while time.time() - start_time < 120:  # 2 minutes
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(line)
            f.write(line + "\n")

ser.close()
