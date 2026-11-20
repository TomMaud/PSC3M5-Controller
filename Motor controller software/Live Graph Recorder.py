
import serial
import json
import time
import threading
import re
import queue
import tkinter as tk

from tkinter import filedialog, messagebox
from datetime import datetime

import matplotlib.pyplot as plt


# ============================================================
# CONFIGURATION
# ============================================================

PORT = "/dev/cu.usbmodem1101"
BAUD_RATE = 115200

SERIAL_TIMEOUT = 0.2

# ============================================================
# GLOBAL DATA
# ============================================================

readings = []

recording = False
serial_thread = None
stop_event = threading.Event()

message_queue = queue.Queue()


# ============================================================
# TELEMETRY PARSER
# ============================================================

def parse_telemetry_line(line):

    if "[Telemetry]" not in line:
        return None

    def get_value(pattern, conversion=float):

        match = re.search(pattern, line)

        if not match:
            return None

        try:
            return conversion(match.group(1))
        except (ValueError, TypeError):
            return None

    data = {
        "state": get_value(r"State:\s*(\d+)", int),

        "enabled": get_value(r"En:\s*(\d+)", int),

        "rpm": get_value(
            r"Speed:\s*(-?\d+(?:\.\d+)?)"
        ),

        "requested_rpm": get_value(
            r"ReqRPM:\s*(-?\d+(?:\.\d+)?)"
        ),

        "faults": get_value(
            r"Faults:\s*(0x[0-9A-Fa-f]+)",
            lambda x: int(x, 16)
        ),

        "hall": get_value(
            r"Hall:\s*(\d+)",
            int
        ),

        "time_ms": get_value(
            r"Time:\s*(\d+)",
            int
        ),

        # PSoC transmits currents in milliamps.
        # Convert to amps for recording.

        "current_u": get_value(
            r"Iu:\s*(-?\d+)",
            lambda x: int(x) / 1000.0
        ),

        "current_v": get_value(
            r"Iv:\s*(-?\d+)",
            lambda x: int(x) / 1000.0
        ),

        "current_w": get_value(
            r"Iw:\s*(-?\d+)",
            lambda x: int(x) / 1000.0
        ),
    }

    required_fields = [
        "time_ms",
        "current_u",
        "current_v",
        "current_w"
    ]

    if any(data[field] is None for field in required_fields):
        return None

    return data


# ============================================================
# SERIAL RECORDING
# ============================================================

def serial_recording():

    global readings

    ser = None
    start_timestamp = None

    try:

        ser = serial.Serial(
            PORT,
            BAUD_RATE,
            timeout=SERIAL_TIMEOUT
        )

        message_queue.put(("status", "Connected to " + PORT))

        # Allow the serial connection to settle.
        time.sleep(1)

        ser.reset_input_buffer()

        while not stop_event.is_set():

            raw_line = ser.readline()

            if not raw_line:
                continue

            line = raw_line.decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if not line:
                continue

            data = parse_telemetry_line(line)

            if data is None:
                continue

            timestamp = data["time_ms"]

            if start_timestamp is None:
                start_timestamp = timestamp

            # Handle 32-bit millisecond timestamp rollover.
            elapsed_ms = (
                timestamp - start_timestamp
            ) & 0xFFFFFFFF

            elapsed_time = elapsed_ms / 1000.0

            record = {
                "time_ms": timestamp,
                "elapsed_time_s": elapsed_time,

                "current_u": data["current_u"],
                "current_v": data["current_v"],
                "current_w": data["current_w"],

                "rpm": data["rpm"],
                "requested_rpm": data["requested_rpm"],

                "state": data["state"],
                "enabled": data["enabled"],
                "faults": data["faults"],
                "hall": data["hall"],
            }

            readings.append(record)

            message_queue.put(("reading", record))

    except serial.SerialException as error:

        message_queue.put(
            ("error", f"Serial error: {error}")
        )

    except Exception as error:

        message_queue.put(
            ("error", f"Recording error: {error}")
        )

    finally:

        if ser is not None and ser.is_open:
            ser.close()

        message_queue.put(("finished", None))


# ============================================================
# START RECORDING
# ============================================================

def start_recording():

    global recording
    global serial_thread
    global readings

    if recording:
        return

    readings = []

    stop_event.clear()

    recording = True

    start_button.config(state=tk.DISABLED)
    stop_button.config(state=tk.NORMAL)

    status_label.config(text="Connecting...")

    serial_thread = threading.Thread(
        target=serial_recording,
        daemon=True
    )

    serial_thread.start()


# ============================================================
# STOP RECORDING
# ============================================================

def stop_recording():

    global recording

    if not recording:
        return

    stop_event.set()

    status_label.config(text="Stopping...")


# ============================================================
# SAVE JSON
# ============================================================

def save_json_file():

    if not readings:

        messagebox.showwarning(
            "No data",
            "There are no motor readings to save."
        )

        return

    filename = filedialog.asksaveasfilename(
        defaultextension=".json",
        filetypes=[
            ("JSON files", "*.json")
        ],
        initialfile=(
            "motor_data_"
            + datetime.now().strftime("%Y%m%d_%H%M%S")
            + ".json"
        )
    )

    if not filename:
        return

    try:

        with open(filename, "w") as file:

            json.dump(
                readings,
                file,
                indent=4
            )

        messagebox.showinfo(
            "Saved",
            f"Motor data saved successfully:\n{filename}"
        )

    except Exception as error:

        messagebox.showerror(
            "Save error",
            str(error)
        )


# ============================================================
# LOAD JSON
# ============================================================

def load_json_file():

    global readings

    filename = filedialog.askopenfilename(
        filetypes=[
            ("JSON files", "*.json")
        ]
    )

    if not filename:
        return

    try:

        with open(filename, "r") as file:
            loaded_data = json.load(file)

        if not isinstance(loaded_data, list):
            raise ValueError("The JSON file must contain a list of readings.")

        readings = loaded_data

        status_label.config(
            text=f"Loaded {len(readings)} readings"
        )

        messagebox.showinfo(
            "Loaded",
            f"Loaded {len(readings)} motor readings."
        )

    except Exception as error:

        messagebox.showerror(
            "Load error",
            str(error)
        )


# ============================================================
# PLOT CURRENT GRAPHS
# ============================================================

def plot_current_graphs():

    if not readings:

        messagebox.showwarning(
            "No data",
            "Record or load motor data first."
        )

        return

    times = [
        item["elapsed_time_s"]
        for item in readings
    ]

    current_u = [
        item["current_u"]
        for item in readings
    ]

    current_v = [
        item["current_v"]
        for item in readings
    ]

    current_w = [
        item["current_w"]
        for item in readings
    ]

    plt.figure(figsize=(12, 6))

    plt.plot(times, current_u, label="Phase U")
    plt.plot(times, current_v, label="Phase V")
    plt.plot(times, current_w, label="Phase W")

    plt.title("Motor Phase Currents")

    plt.xlabel("Time (seconds)")
    plt.ylabel("Current (A)")

    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()


# ============================================================
# PLOT MOTOR SPEED
# ============================================================

def plot_speed_graph():

    if not readings:

        messagebox.showwarning(
            "No data",
            "Record or load motor data first."
        )

        return

    times = [
        item["elapsed_time_s"]
        for item in readings
    ]

    actual_speed = [
        item.get("rpm", 0)
        for item in readings
    ]

    requested_speed = [
        item.get("requested_rpm", 0)
        for item in readings
    ]

    plt.figure(figsize=(12, 6))

    plt.plot(
        times,
        actual_speed,
        label="Actual RPM"
    )

    plt.plot(
        times,
        requested_speed,
        label="Requested RPM",
        linestyle="--"
    )

    plt.title("Motor Speed")

    plt.xlabel("Time (seconds)")
    plt.ylabel("Speed (RPM)")

    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()


# ============================================================
# PLOT HALL POSITION
# ============================================================

def plot_hall_graph():

    if not readings:

        messagebox.showwarning(
            "No data",
            "Record or load motor data first."
        )

        return

    times = [
        item["elapsed_time_s"]
        for item in readings
    ]

    hall = [
        item.get("hall", 0)
        for item in readings
    ]

    plt.figure(figsize=(12, 5))

    plt.step(
        times,
        hall,
        where="post",
        label="Hall State"
    )

    plt.title("Motor Hall Sensor Position")

    plt.xlabel("Time (seconds)")
    plt.ylabel("Hall State")

    plt.yticks(range(8))
    plt.ylim(-0.5, 7.5)

    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()


# ============================================================
# PROCESS SERIAL THREAD MESSAGES
# ============================================================

def process_messages():

    global recording

    try:

        while True:

            message_type, data = message_queue.get_nowait()

            if message_type == "status":

                status_label.config(text=data)

            elif message_type == "reading":

                status_label.config(
                    text=(
                        f"Recording | "
                        f"RPM: {data['rpm']} | "
                        f"Hall: {data['hall']} | "
                        f"Samples: {len(readings)}"
                    )
                )

            elif message_type == "error":

                status_label.config(text=data)

                messagebox.showerror(
                    "Recording error",
                    data
                )

            elif message_type == "finished":

                recording = False

                start_button.config(state=tk.NORMAL)
                stop_button.config(state=tk.DISABLED)

                if not stop_event.is_set():
                    status_label.config(
                        text="Recording stopped"
                    )
                else:
                    status_label.config(
                        text=f"Stopped | {len(readings)} readings recorded"
                    )

    except queue.Empty:
        pass

    root.after(100, process_messages)


# ============================================================
# TKINTER INTERFACE
# ============================================================

root = tk.Tk()

root.title("Motor Telemetry Monitor")
root.geometry("520x420")

title_label = tk.Label(
    root,
    text="Motor Telemetry Monitor",
    font=("Arial", 18, "bold")
)

title_label.pack(pady=15)


status_label = tk.Label(
    root,
    text="Not connected",
    font=("Arial", 11)
)

status_label.pack(pady=10)


# Recording controls

control_frame = tk.Frame(root)
control_frame.pack(pady=10)

start_button = tk.Button(
    control_frame,
    text="Start Recording",
    width=18,
    command=start_recording
)

start_button.grid(row=0, column=0, padx=5)


stop_button = tk.Button(
    control_frame,
    text="Stop Recording",
    width=18,
    command=stop_recording,
    state=tk.DISABLED
)

stop_button.grid(row=0, column=1, padx=5)


# File controls

file_frame = tk.Frame(root)
file_frame.pack(pady=10)

save_button = tk.Button(
    file_frame,
    text="Save JSON",
    width=18,
    command=save_json_file
)

save_button.grid(row=0, column=0, padx=5)


load_button = tk.Button(
    file_frame,
    text="Load JSON",
    width=18,
    command=load_json_file
)

load_button.grid(row=0, column=1, padx=5)


# Graph controls

graph_frame = tk.LabelFrame(
    root,
    text="Motor Graphs",
    padx=10,
    pady=10
)

graph_frame.pack(padx=20, pady=15, fill="x")


current_button = tk.Button(
    graph_frame,
    text="Phase Current Graph",
    command=plot_current_graphs
)

current_button.pack(fill="x", pady=3)


speed_button = tk.Button(
    graph_frame,
    text="Motor Speed Graph",
    command=plot_speed_graph
)

speed_button.pack(fill="x", pady=3)


hall_button = tk.Button(
    graph_frame,
    text="Hall Position Graph",
    command=plot_hall_graph
)

hall_button.pack(fill="x", pady=3)


# Sample count

sample_label = tk.Label(
    root,
    text="Samples: 0"
)

sample_label.pack(pady=5)


def update_sample_count():

    sample_label.config(
        text=f"Samples: {len(readings)}"
    )

    root.after(500, update_sample_count)


# ============================================================
# START APPLICATION
# ============================================================

process_messages()
update_sample_count()

root.mainloop()
