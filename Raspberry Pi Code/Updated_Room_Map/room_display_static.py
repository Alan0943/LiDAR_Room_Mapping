import matplotlib.pyplot as plt
import pandas as pd

# Load the raw data: pan, tilt, distance
data = pd.read_csv("scan_results.txt", sep="\t", header=None, names=["pan", "tilt", "dist"])

# Extract columns
pan = data["pan"]
tilt = data["tilt"]
dist = data["dist"]

# Plot using pan and tilt as axes, distance as color
plt.figure(figsize=(10,8))
scatter = plt.scatter(pan, tilt, c=dist, cmap="coolwarm", s=10)  # red = far, blue = close
plt.colorbar(scatter, label="Distance (cm)")
plt.xlabel("Pan PWM")
plt.ylabel("Tilt PWM")
plt.title("Raw LIDAR Scan (Distance as Color)")
plt.grid(True)
plt.show()
