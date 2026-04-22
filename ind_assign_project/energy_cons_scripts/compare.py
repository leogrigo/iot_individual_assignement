import numpy as np

# =========================
# CONFIG
# =========================
file_off = "adaptive_off.txt"
file_on = "adaptive_on.txt"

# If you want to ignore the initial boot/transient seconds
skip_initial_seconds = 0.0

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

    # Remove the initial seconds if requested
    mask = time_s >= skip_initial_seconds
    time_s = time_s[mask]
    current_mA = current_mA[mask]
    power_mW = power_mW[mask]

    if len(time_s) < 2:
        raise ValueError(f"Insufficient data after the initial trim in {filename}")

    # Make time relative to the actual start
    time_s = time_s - time_s[0]

    return time_s, current_mA, power_mW

# =========================
# METRICS
# =========================
def compute_metrics(filename):
    time_s, current_mA, power_mW = load_run(filename)

    duration_s = time_s[-1] - time_s[0]

    avg_current_mA = np.mean(current_mA)
    avg_power_mW = np.mean(power_mW)

    # Energy = integral of power over time
    # mW * s = mJ
    energy_mJ = np.trapz(power_mW, time_s)

    # optional: mWh
    energy_mWh = energy_mJ / 3600.0

    return {
        "filename": filename,
        "duration_s": duration_s,
        "avg_current_mA": avg_current_mA,
        "avg_power_mW": avg_power_mW,
        "energy_mJ": energy_mJ,
        "energy_mWh": energy_mWh,
    }

def savings_percent(reference, new):
    return 100.0 * (reference - new) / reference

# =========================
# MAIN
# =========================
off = compute_metrics(file_off)
on = compute_metrics(file_on)

print("=== ADAPTIVE OFF ===")
print(f"File:              {off['filename']}")
print(f"Duration (s):      {off['duration_s']:.3f}")
print(f"Avg current (mA):  {off['avg_current_mA']:.3f}")
print(f"Avg power (mW):    {off['avg_power_mW']:.3f}")
print(f"Energy (mJ):       {off['energy_mJ']:.3f}")
print(f"Energy (mWh):      {off['energy_mWh']:.6f}")
print()

print("=== ADAPTIVE ON ===")
print(f"File:              {on['filename']}")
print(f"Duration (s):      {on['duration_s']:.3f}")
print(f"Avg current (mA):  {on['avg_current_mA']:.3f}")
print(f"Avg power (mW):    {on['avg_power_mW']:.3f}")
print(f"Energy (mJ):       {on['energy_mJ']:.3f}")
print(f"Energy (mWh):      {on['energy_mWh']:.6f}")
print()

current_saving = savings_percent(off["avg_current_mA"], on["avg_current_mA"])
power_saving = savings_percent(off["avg_power_mW"], on["avg_power_mW"])
energy_saving = savings_percent(off["energy_mJ"], on["energy_mJ"])

print("=== COMPARISON (ON vs OFF) ===")
print(f"Current saving (%): {current_saving:.2f}")
print(f"Power saving (%):   {power_saving:.2f}")
print(f"Energy saving (%):  {energy_saving:.2f}")
print()

if energy_saving > 0:
    print("Result: adaptive ON consumed LESS than adaptive OFF.")
elif energy_saving < 0:
    print("Result: adaptive ON consumed MORE than adaptive OFF.")
else:
    print("Result: no difference.")
