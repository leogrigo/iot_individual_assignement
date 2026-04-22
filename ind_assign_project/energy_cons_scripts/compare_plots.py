import numpy as np
import matplotlib.pyplot as plt

# =========================
# CONFIG
# =========================
file_off = "adaptive_off.txt"
file_on = "adaptive_on.txt"

smooth_window = 10   # set to 1 to disable smoothing

# =========================
# LOAD FUNCTION
# =========================
def load_run(filename):
    time_ms = []
    current_mA = []
    power_mW = []

    with open(filename, "r") as f:
        for line in f:
            parts = line.strip().split()

            if len(parts) != 3:
                continue

            try:
                t = float(parts[0])
                c = float(parts[1])
                p = float(parts[2])
            except ValueError:
                continue

            time_ms.append(t)
            current_mA.append(c)
            power_mW.append(p)

    if len(time_ms) < 2:
        raise ValueError(f"Insufficient data in {filename}")

    time_s = np.array(time_ms) / 1000.0
    current_mA = np.array(current_mA)
    power_mW = np.array(power_mW)

    # relative time starting from 0
    time_s = time_s - time_s[0]

    return time_s, current_mA, power_mW

# =========================
# SMOOTHING
# =========================
def smooth_signal(x, window):
    if window <= 1:
        return x
    kernel = np.ones(window) / window
    return np.convolve(x, kernel, mode="same")

# =========================
# LOAD DATA
# =========================
time_off, current_off, power_off = load_run(file_off)
time_on, current_on, power_on = load_run(file_on)

current_off_s = smooth_signal(current_off, smooth_window)
power_off_s = smooth_signal(power_off, smooth_window)

current_on_s = smooth_signal(current_on, smooth_window)
power_on_s = smooth_signal(power_on, smooth_window)

# =========================
# PLOT POWER
# =========================
plt.figure(figsize=(11, 5))
plt.plot(time_off, power_off_s, label="Adaptive OFF")
plt.plot(time_on, power_on_s, label="Adaptive ON")
plt.xlabel("Time (s)")
plt.ylabel("Power (mW)")
plt.title("Power Comparison: Adaptive OFF vs ON")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("comparison_power.png", dpi=300)
plt.show()

# =========================
# PLOT CURRENT
# =========================
plt.figure(figsize=(11, 5))
plt.plot(time_off, current_off_s, label="Adaptive OFF")
plt.plot(time_on, current_on_s, label="Adaptive ON")
plt.xlabel("Time (s)")
plt.ylabel("Current (mA)")
plt.title("Current Comparison: Adaptive OFF vs ON")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("comparison_current.png", dpi=300)
plt.show()
