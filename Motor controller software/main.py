
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import re
import time
import serial
from serial.tools import list_ports


# ---------------------------------------------------------
# Settings
# ---------------------------------------------------------

BAUD_RATE = 115200

HEARTBEAT_BYTE = b"H"
HEARTBEAT_PERIOD_S = 0.25

MIN_RPM = 300
MAX_RPM = 6000
RPM_STEP = 10


# ---------------------------------------------------------
# Serial port discovery
# ---------------------------------------------------------

def find_motor_port():
    """Find the J-Link virtual COM port."""

    ports = list_ports.comports()

    for port in ports:
        if "J-Link" in port.description:
            return port.device

    return None


# ---------------------------------------------------------
# Main application
# ---------------------------------------------------------

class MotorControlGUI:

    def __init__(self, root):

        self.root = root
        self.root.title("PMSM FOC Motor Controller")
        self.root.geometry("650x600")

        # Serial connection
        self.serial_connection = None
        self.serial_lock = threading.Lock()

        # Thread control
        self.stop_event = threading.Event()
        self.rx_thread = None
        self.heartbeat_thread = None

        # GUI message queue
        self.log_queue = queue.Queue()

        # RPM controls
        self.rpm_var = tk.IntVar(value=0)
        self.reverse_var = tk.BooleanVar(value=False)

        # -------------------------------------------------
        # Connection controls
        # -------------------------------------------------

        connection_frame = ttk.LabelFrame(
            root,
            text="Connection",
            padding=10
        )
        connection_frame.pack(fill="x", padx=10, pady=10)

        ttk.Label(
            connection_frame,
            text="Serial Port:"
        ).grid(row=0, column=0, padx=5, pady=5)

        self.port_var = tk.StringVar()

        self.port_dropdown = ttk.Combobox(
            connection_frame,
            textvariable=self.port_var,
            width=25,
            state="readonly"
        )
        self.port_dropdown.grid(
            row=0, column=1, padx=5, pady=5
        )

        self.refresh_button = ttk.Button(
            connection_frame,
            text="Refresh",
            command=self.refresh_ports
        )
        self.refresh_button.grid(
            row=0, column=2, padx=5, pady=5
        )

        self.connect_button = ttk.Button(
            connection_frame,
            text="Connect",
            command=self.connect
        )
        self.connect_button.grid(
            row=0, column=3, padx=5, pady=5
        )

        self.disconnect_button = ttk.Button(
            connection_frame,
            text="Disconnect",
            command=self.disconnect,
            state="disabled"
        )
        self.disconnect_button.grid(
            row=0, column=4, padx=5, pady=5
        )

        self.connection_status = ttk.Label(
            connection_frame,
            text="Disconnected"
        )
        self.connection_status.grid(
            row=1, column=0, columnspan=5, pady=5
        )

        # -------------------------------------------------
        # Motor speed controls
        # -------------------------------------------------

        speed_frame = ttk.LabelFrame(
            root,
            text="Motor Speed Control",
            padding=15
        )
        speed_frame.pack(fill="x", padx=10, pady=10)

        self.rpm_label = ttk.Label(
            speed_frame,
            text="Requested Speed: 0 RPM",
            font=("Arial", 14, "bold")
        )
        self.rpm_label.pack(pady=5)

        self.rpm_slider = tk.Scale(
            speed_frame,
            from_=MIN_RPM,
            to=MAX_RPM,
            resolution=RPM_STEP,
            orient=tk.HORIZONTAL,
            variable=self.rpm_var,
            length=450,
            command=self.update_rpm_label
        )
        self.rpm_slider.pack(pady=10)

        # Direction switch
        self.reverse_switch = ttk.Checkbutton(
            speed_frame,
            text="Reverse Direction",
            variable=self.reverse_var
        )
        self.reverse_switch.pack(pady=5)

        # Speed buttons
        button_frame = ttk.Frame(speed_frame)
        button_frame.pack(pady=10)

        self.set_rpm_button = ttk.Button(
            button_frame,
            text="Set RPM",
            command=self.set_rpm
        )
        self.set_rpm_button.grid(
            row=0, column=0, padx=10
        )

        self.stop_motor_button = ttk.Button(
            button_frame,
            text="Set 0 RPM",
            command=self.stop_motor
        )
        self.stop_motor_button.grid(
            row=0, column=1, padx=10
        )

        # -------------------------------------------------
        # Telemetry / log display
        # -------------------------------------------------

        telemetry_frame = ttk.LabelFrame(
            root,
            text="Motor Telemetry",
            padding=10
        )
        telemetry_frame.pack(
            fill="both",
            expand=True,
            padx=10,
            pady=10
        )

        self.log_text = tk.Text(
            telemetry_frame,
            height=15,
            width=75,
            state="disabled",
            font=("Courier", 9)
        )
        self.log_text.pack(
            fill="both",
            expand=True
        )

        scrollbar = ttk.Scrollbar(
            self.log_text,
            command=self.log_text.yview
        )
        scrollbar.pack(side="right", fill="y")

        self.log_text.configure(
            yscrollcommand=scrollbar.set
        )

        # -------------------------------------------------
        # Initialisation
        # -------------------------------------------------

        self.refresh_ports()

        self.root.after(100, self.process_log_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # -----------------------------------------------------
    # Logging
    # -----------------------------------------------------

    def log(self, message):
        self.log_queue.put(message)

    def process_log_queue(self):

        while not self.log_queue.empty():

            message = self.log_queue.get()

            self.log_text.configure(state="normal")

            self.log_text.insert(
                tk.END,
                message + "\n"
            )

            self.log_text.see(tk.END)

            self.log_text.configure(state="disabled")

        self.root.after(100, self.process_log_queue)

    # -----------------------------------------------------
    # Port selection
    # -----------------------------------------------------

    def refresh_ports(self):

        ports = list_ports.comports()

        port_names = [
            port.device for port in ports
        ]

        self.port_dropdown["values"] = port_names

        detected_port = find_motor_port()

        if detected_port:
            self.port_var.set(detected_port)

        elif port_names and not self.port_var.get():
            self.port_var.set(port_names[0])

        if not port_names:
            self.port_var.set("")
            self.log("No serial ports detected.")

    # -----------------------------------------------------
    # Connection
    # -----------------------------------------------------

    def connect(self):

        port = self.port_var.get()

        if not port:
            messagebox.showerror(
                "Connection Error",
                "Please select a serial port."
            )
            return

        try:

            self.serial_connection = serial.Serial(
                port=port,
                baudrate=BAUD_RATE,
                timeout=0.1,
                write_timeout=1
            )

            self.stop_event.clear()

            # Start receiver thread
            self.rx_thread = threading.Thread(
                target=self.receive_data,
                daemon=True
            )
            self.rx_thread.start()

            # Start heartbeat thread
            self.heartbeat_thread = threading.Thread(
                target=self.send_heartbeat,
                daemon=True
            )
            self.heartbeat_thread.start()

            self.connection_status.config(
                text=f"Connected to {port}"
            )

            self.connect_button.config(state="disabled")
            self.disconnect_button.config(state="normal")

            self.log(f"Connected to {port}")
            self.log("Heartbeat started.")

        except serial.SerialException as error:

            messagebox.showerror(
                "Serial Error",
                str(error)
            )

    def disconnect(self):

        self.stop_event.set()

        with self.serial_lock:

            if self.serial_connection:
                try:
                    self.serial_connection.close()
                except serial.SerialException:
                    pass

                self.serial_connection = None

        self.connection_status.config(
            text="Disconnected"
        )

        self.connect_button.config(state="normal")
        self.disconnect_button.config(state="disabled")

        self.log("Disconnected from motor controller.")

    # -----------------------------------------------------
    # Serial transmission
    # -----------------------------------------------------

    def send_serial(self, data):

        with self.serial_lock:

            if (
                self.serial_connection is None
                or not self.serial_connection.is_open
            ):
                self.log("Cannot send command: not connected.")
                return False

            try:
                self.serial_connection.write(data)
                return True

            except serial.SerialException as error:
                self.log(f"Serial write error: {error}")
                return False

    # -----------------------------------------------------
    # Heartbeat
    # -----------------------------------------------------

    def send_heartbeat(self):

        while not self.stop_event.is_set():

            self.send_serial(HEARTBEAT_BYTE)

            self.stop_event.wait(HEARTBEAT_PERIOD_S)

    # -----------------------------------------------------
    # Receive telemetry
    # -----------------------------------------------------

    def receive_data(self):

        ansi_escape = re.compile(
            r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])"
        )

        while not self.stop_event.is_set():

            try:

                if self.serial_connection is None:
                    break

                line = self.serial_connection.readline()

                if not line:
                    continue

                decoded = line.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                decoded = ansi_escape.sub("", decoded)

                if decoded:
                    self.log(decoded)

            except serial.SerialException as error:

                if not self.stop_event.is_set():
                    self.log(f"Serial receive error: {error}")

                break

    # -----------------------------------------------------
    # RPM slider
    # -----------------------------------------------------

    def update_rpm_label(self, value):

        rpm = int(float(value))

        self.rpm_label.config(
            text=f"Requested Speed: {rpm} RPM"
        )

    # -----------------------------------------------------
    # Set motor speed
    # -----------------------------------------------------

    def set_rpm(self):

        rpm = int(self.rpm_var.get())

        # Apply direction switch
        if self.reverse_var.get():
            rpm = -rpm

        command = f"R{rpm}\n".encode("ascii")

        if self.send_serial(command):

            direction = (
                "Reverse" if rpm < 0 else "Forward"
            )

            self.log(
                f"Sent speed command: {abs(rpm)} RPM ({direction})"
            )

    # -----------------------------------------------------
    # Stop motor
    # -----------------------------------------------------

    def stop_motor(self):

        self.rpm_var.set(0)
        self.update_rpm_label(0)

        if self.send_serial(b"R0\n"):
            self.log("Sent motor stop command.")

    # -----------------------------------------------------
    # Application shutdown
    # -----------------------------------------------------

    def close(self):

        self.disconnect()
        self.root.destroy()


# ---------------------------------------------------------
# Run application
# ---------------------------------------------------------

if __name__ == "__main__":

    root = tk.Tk()

    app = MotorControlGUI(root)

    root.mainloop()
