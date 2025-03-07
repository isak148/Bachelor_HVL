# SPDX-FileCopyrightText: 2018 Jonas Schatz
# SPDX-License-Identifier: MIT

# Demo of reading the range from the VL6180x distance sensor in
# continuous mode and plotting in real time

import adafruit_vl6180x
import time
import csv
import os
import board
import busio
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Create I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Create sensor instance.
sensor = adafruit_vl6180x.VL6180X(i2c)

# Starting continuous mode
print("Starting continuous mode")
sensor.start_range_continuous(20)

# Initialize plot
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

def animate(i, xs, ys):
    # Read the range in millimeters.
    range_mm = sensor.range
    print("Range: {0}mm".format(range_mm))

    # Append new data to lists.
    xs.append(time.time())
    ys.append(range_mm)

    # Limit x and y lists to 20 values.
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists.
    ax.clear()
    ax.plot(xs, ys)

    # Format plot.
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('VL6180x Range Over Time')
    plt.ylabel('Range (mm)')

    file_path = "sensor_data_VL6180x.csv"
    file_exists = os.path.exists(file_path)

    with open(file_path, "a", newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(["Range", "Time"])
        writer.writerow([range_mm, time.time()])

# Set up plot to call animate() function periodically.
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=100) #interval is in milliseconds.

plt.show()

# Stop continuous mode.
sensor.stop_range_continuous()