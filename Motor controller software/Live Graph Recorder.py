
import serial
import json
import time
import os

import tkinter as tk
from tkinter import filedialog, messagebox

import matplotlib.pyplot as plt

from datetime import datetime

# ============================================================
# CONFIGURATION
# ============================================================

PORT = "/dev/cu.usbmodem1101"
BAUD_RATE = 115200

# ============================================================
# RECORD MOTOR DATA
# ============================================================

def record_data():

    print(f"Connecting to {PORT}...")

    try:
        ser = serial.Serial(
            PORT,
            BAUD_RATE,
            timeout=1
        )

        time.sleep(2)
        ser.reset_input_buffer()

        print("Connected successfully.")

    except serial.SerialException as error:
        print(f"Serial connection error: {error}")
        return None

    readings = []

    first_timestamp = None
    last_timestamp = None
    timestamp_offset = 0

    total_samples = 0

    print("\nRecording motor currents...")
    print("Press CTRL+C to stop.\n")

    try:

        while True:

            raw_line = ser.readline()

            if not raw_line:
                continue

            try:

                line = raw_line.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                parts = line.split(",")

                if len(parts) != 5 or parts[0] != "I":
                    continue

                timestamp_ms = int(parts[1])

                # Convert mA to A
                iu = int(parts[2]) / 1000.0
                iv = int(parts[3]) / 1000.0
                iw = int(parts[4]) / 1000.0

                # Handle timestamp rollover
                if (
                    last_timestamp is not None
                    and timestamp_ms < last_timestamp
                ):
                    timestamp_offset += 2**32

                last_timestamp = timestamp_ms

                adjusted_timestamp = (
                    timestamp_ms + timestamp_offset
                )

                if first_timestamp is None:
                    first_timestamp = adjusted_timestamp

                elapsed_time = (
                    adjusted_timestamp - first_timestamp
                ) / 1000.0

                # Store as a Python dictionary
                reading = {
                    "time_ms": adjusted_timestamp,
                    "elapsed_time_s": elapsed_time,
                    "current_u": iu,
                    "current_v": iv,
                    "current_w": iw
                }

                readings.append(reading)

                total_samples += 1

                if total_samples % 10 == 0:

                    print(
                        f"Samples: {total_samples} | "
                        f"Time: {elapsed_time:.2f}s | "
                        f"U: {iu:.3f}A | "
                        f"V: {iv:.3f}A | "
                        f"W: {iw:.3f}A"
                    )

            except (ValueError, IndexError) as error:
                print(f"Invalid message: {error}")

    except KeyboardInterrupt:
        print("\nRecording stopped.")

    finally:
        ser.close()

    print(f"\nTotal samples: {total_samples}")

    return readings


# ============================================================
# SAVE JSON
# ============================================================

def save_json_file(readings):

    if not readings:
        print("No readings to save.")
        return None

    filename = (
        f"motor_readings_"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )

    try:

        with open(filename, "w", encoding="utf-8") as file:

            json.dump(
                readings,
                file,
                indent=4,
                allow_nan=False
            )

        print(f"JSON saved successfully: {filename}")

        return filename

    except (OSError, TypeError, ValueError) as error:

        print(f"JSON save error: {error}")
        return None


# ============================================================
# LOAD JSON FILE
# ============================================================

def load_json_file():

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
            data = json.load(file)

        if not isinstance(data, list):
            raise ValueError("Expected a JSON list of readings.")

        print(f"Loaded {len(data)} readings.")

        return data

    except (OSError, json.JSONDecodeError, ValueError) as error:

        messagebox.showerror("File Error", str(error))
        return None


# ============================================================
# PLOT MOTOR CURRENTS
# ============================================================

def plot_currents(data):

    if not data:
        print("No readings found.")
        return

    times = [
        reading["elapsed_time_s"]
        for reading in data
    ]

    current_u = [
        reading.get("current_u")
        for reading in data
    ]

    current_v = [
        reading.get("current_v")
        for reading in data
    ]

    current_w = [
        reading.get("current_w")
        for reading in data
    ]

    fig, ax = plt.subplots(figsize=(12, 6))

    ax.plot(times, current_u, label="Phase U")
    ax.plot(times, current_v, label="Phase V")
    ax.plot(times, current_w, label="Phase W")

    ax.set_xlabel("Time (seconds)")
    ax.set_ylabel("Current (A)")

    ax.set_title("PMSM Phase Currents")

    ax.grid(True)
    ax.legend()

    plt.tight_layout()
    plt.show()


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    print("1. Record new motor data")
    print("2. Load existing JSON data")

    choice = input("\nSelect option: ").strip()

    if choice == "1":

        readings = record_data()

        if readings:

            filename = save_json_file(readings)

            if filename:
                plot_currents(readings)

    elif choice == "2":

        readings = load_json_file()

        if readings is not None:
            plot_currents(readings)

    else:

        print("Invalid selection.")
