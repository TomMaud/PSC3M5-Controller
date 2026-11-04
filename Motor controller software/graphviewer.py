import json
import tkinter as tk
from tkinter import filedialog, messagebox

import matplotlib.pyplot as plt


def load_json_file():
    """Open a file picker and load motor readings."""

    root = tk.Tk()
    root.withdraw()

    filename = filedialog.askopenfilename(
        title="Select Motor Readings",
        filetypes=[("JSON files", "*.json")]
    )

    root.destroy()

    if not filename:
        return None

    try:
        with open(filename, "r", encoding="utf-8") as file:
            return json.load(file)

    except (OSError, json.JSONDecodeError) as error:
        messagebox.showerror("File Error", str(error))
        return None


def plot_currents(data):
    """Plot U, V and W phase currents against elapsed time."""

    if not data:
        print("No readings found.")
        return

    # Use the first timestamp as the time origin.
    start_time = data[0]["time_ms"]

    times = []
    current_u = []
    current_v = []
    current_w = []
    voltage_u = []
    voltage_v = []
    voltage_w = []
    speed = []
    requestedspeed = []

    for reading in data:

        # Convert milliseconds since epoch to elapsed seconds.
        elapsed_time = (
            reading["time_ms"] - start_time
        ) / 1000.0

        times.append(elapsed_time)

        current_u.append(reading.get("current_u"))
        current_v.append(reading.get("current_v"))
        current_w.append(reading.get("current_w"))

        voltage_u.append(reading.get("voltage_u"))
        voltage_v.append(reading.get("voltage_v"))
        voltage_w.append(reading.get("voltage_w"))
        speed.append(reading.get("speed_rpm"))
        requestedspeed.append(reading.get("requested"))

    fig, ax1 = plt.subplots(figsize=(12, 6))

    # Left axis: phase currents
    ax1.plot(times, current_u, label="Phase U")
    ax1.plot(times, current_v, label="Phase V")
    ax1.plot(times, current_w, label="Phase W")

    ax1.set_xlabel("Time (seconds)")
    ax1.set_ylabel("Current (A)")
    ax1.grid(True)

    # Right axis: motor speed
    ax2 = ax1.twinx()
    ax2.plot(times, speed, label="Motor Speed", linestyle="--")
    ax2.plot(times, requestedspeed, label="Requested Motor Speed", linestyle="-")

    ax2.set_ylabel("Speed (rpm)")

    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()

    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

    plt.title("PMSM Phase Currents and Motor Speed")
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(12, 6))

    plt.plot(times, voltage_u, label="Voltage U")
    plt.plot(times, voltage_v, label="Voltage V")
    plt.plot(times, voltage_w, label="Voltage W")

    plt.xlabel("Time (seconds)")
    plt.ylabel("Voltage (V)")
    plt.title("PMSM Phase Voltage Against Time")

    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()


if __name__ == "__main__":

    readings = load_json_file()

    if readings is not None:
        plot_currents(readings)
