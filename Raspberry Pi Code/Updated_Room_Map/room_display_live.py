import serial
import matplotlib.pyplot as plt

ser = serial.Serial('COM3', 115200)  # replace COM3 with your USB port
plt.ion()
fig, ax = plt.subplots()
sc = ax.scatter([], [], c=[], cmap='coolwarm', vmin=0, vmax=1000, s=10)
ax.set_xlabel("Pan PWM")
ax.set_ylabel("Tilt PWM")
plt.title("Live LIDAR scan")

pan_vals, tilt_vals, dist_vals = [], [], []

while True:
    line = ser.readline().decode().strip()
    try:
        pan, tilt, dist = map(int, line.split(','))
        pan_vals.append(pan)
        tilt_vals.append(tilt)
        dist_vals.append(dist)

        ax.clear()
        sc = ax.scatter(pan_vals, tilt_vals, c=dist_vals, cmap='coolwarm', vmin=0, vmax=1000, s=10)
        plt.pause(0.01)
    except:
        continue
