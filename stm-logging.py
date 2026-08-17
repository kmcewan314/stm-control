import time
import sys
import csv
import serial
import atexit
from collections import deque
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# --- configuration ---
port_name = "COM3"
baud = 115200
filename = "stm_data_" + time.strftime("%Y_%m_%d-%H_%M_%S") + ".csv"
filepath = "C:\\Users\\mitmu\\Documents\\spacetime-modulator\\stm_motor_recordings\\"
sample_rate = 5 # set in arduino, informs program of sample rate in milliseconds
window_size = 2000 # how many data points to show at once
speed_average_samples = 50 # how many data points to average for speed calculations

# --- set up serial port ---
try:
    ser = serial.Serial(port_name, baud, timeout=0.05)
except serial.SerialException:
    print("SerialException: serial port could not be opened")

# --- csv file setup ---
fullname = "".join([filepath, filename])

csv_file = open(fullname, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Timestamp", "Object_Angle", "Mirror_Angle", "Trigger_Sent"])

# --- rolling window buffers ---
obj_history = deque(maxlen=window_size)
mir_history = deque(maxlen=window_size)
obj_short_hist = deque(maxlen=speed_average_samples)
mir_short_hist = deque(maxlen=speed_average_samples)
time_short_hist = deque(maxlen=speed_average_samples)

# -- pyqt gui setup ---
app = QtWidgets.QApplication(sys.argv)

main_window = QtWidgets.QMainWindow()
main_window.setWindowTitle("motor speed control & live plotter")
main_window.resize(1000, 700)

central_widget = QtWidgets.QWidget()
main_layout = QtWidgets.QVBoxLayout()
central_widget.setLayout(main_layout)
main_window.setCentralWidget(central_widget)

# control pane layout
control_layout = QtWidgets.QHBoxLayout()

btn_start = QtWidgets.QPushButton("START (send sync)")
btn_start.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold;")

btn_stop = QtWidgets.QPushButton("STOP")
btn_stop.setStyleSheet("background-color: #c62828; color: white; font-weight: bold;")

obj_rpm_label = QtWidgets.QLabel("target object RPM:")
obj_rpm_input = QtWidgets.QLineEdit("60")
obj_rpm_input.setFixedWidth(80)
btn_set_obj_rpm = QtWidgets.QPushButton("set speed")

mir_rpm_label = QtWidgets.QLabel("target mirror RPM:")
mir_rpm_input = QtWidgets.QLineEdit("40")
mir_rpm_input.setFixedWidth(80)
btn_set_mir_rpm = QtWidgets.QPushButton("set speed")

control_layout.addWidget(btn_start)
control_layout.addWidget(btn_stop)
control_layout.addSpacing(20)
control_layout.addWidget(obj_rpm_label)
control_layout.addWidget(obj_rpm_input)
control_layout.addWidget(btn_set_obj_rpm)
control_layout.addSpacing(20)
control_layout.addWidget(mir_rpm_label)
control_layout.addWidget(mir_rpm_input)
control_layout.addWidget(btn_set_mir_rpm)
control_layout.addStretch()

main_layout.addLayout(control_layout)

# status bar
status_bar = QtWidgets.QStatusBar()
actual_obj_rpm = QtWidgets.QLabel("actual object speed: 0 RPM")
actual_mir_rpm = QtWidgets.QLabel("actual mirror speed: 0 RPM")
status_bar.addPermanentWidget(actual_obj_rpm)
status_bar.addPermanentWidget(actual_mir_rpm)

main_window.setStatusBar(status_bar)

# pyqtgraph layout
plot_widget = pg.GraphicsLayoutWidget()
main_layout.addWidget(plot_widget)

plot = plot_widget.addPlot(title="motor angle (°)")
plot.setYRange(0,360, padding=0.05)
plot.setLabel("bottom", "samples")
plot.setLabel("left", "angle (°)")
plot.addLegend()

curve_obj = plot.plot(pen=pg.mkPen("r", width=2), name="object motor")
curve_mir = plot.plot(pen=pg.mkPen("b", width=2), name="mirror motor")


# --- serial command handlers ---
def send_start():
    if ser.is_open:
        ser.write(b"START\n")

def send_stop():
    if ser.is_open:
        ser.write(b"STOP\n")

def send_obj_rpm():
    if ser.is_open:
        rpm_val = obj_rpm_input.text().strip()
        if rpm_val.isdigit() and int(rpm_val) > 0:
            cmd = f"OBJ:{rpm_val}\n".encode("utf-8")
            ser.write(cmd)

def send_mir_rpm():
    if ser.is_open:
        rpm_val = mir_rpm_input.text().strip()
        if rpm_val.isdigit() and int(rpm_val) > 0:
            cmd = f"MIR:{rpm_val}\n".encode("utf-8")
            ser.write(cmd)

btn_start.clicked.connect(send_start)
btn_stop.clicked.connect(send_stop)
btn_set_obj_rpm.clicked.connect(send_obj_rpm)
obj_rpm_input.returnPressed.connect(send_obj_rpm)
btn_set_mir_rpm.clicked.connect(send_mir_rpm)
mir_rpm_input.returnPressed.connect(send_mir_rpm)

# --- speed calculations ---
def calcSpeed(angle_window, time_window):
    if len(angle_window) < 2:
        # if only 1 sample in window, can't calculate
        return 0.0

    distance = angle_window[-1] - angle_window[0] # get degrees traveled durign the window
    if distance < 0:
        # if object has wrapped around from 360 back to 0, it's negative so add 360
        distance += 360

    # time in seconds = time delta / 1 million microseconds per second
    time_sec = (time_window[-1] - time_window[0]) / 1000000.0
    if not time_sec:
        # can't divide by 0
        return 0.0

    # degrees per second = (degrees traveled)/(time in seconds)
    dps = (distance/time_sec)
    # rev per minute = (degrees per s) * (60 seconds per minute) / (360 degrees per rev)
    return int(dps/6.0)

# --- main loop ---
def update():
    updated = False
    # read everything in the serial buffer
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line or line.startswith("SYSTEM") or line.startswith("SUCCESS"):
                continue

            parts = line.split(",")
            timestamp, obj_val, mir_val, trig_sent = None, None, None, None

            if len(parts) == 4:
                timestamp = int(parts[0])
                obj_val = float(parts[1])
                mir_val = float(parts[2])
                trig_sent = bool(int(parts[3]))

            if None not in {timestamp, obj_val, mir_val, trig_sent}:
                updated = True
                # write to csv
                csv_writer.writerow([timestamp, obj_val, mir_val, trig_sent])
                csv_file.flush()

                # append to rolling buffers
                obj_history.append(obj_val)
                mir_history.append(mir_val)
                obj_short_hist.append(obj_val)
                mir_short_hist.append(mir_val)
                time_short_hist.append(timestamp)

        except (ValueError, IndexError):
            # skip unparseable lines
            pass
    if updated:
        # update plot data
        curve_obj.setData(list(obj_history))
        curve_mir.setData(list(mir_history))

        obj_speed = calcSpeed(obj_short_hist, time_short_hist)
        mir_speed = calcSpeed(mir_short_hist, time_short_hist)
        actual_obj_rpm.setText(f"actual object speed: {obj_speed} RPM")
        actual_mir_rpm.setText(f"actual mirror speed: {mir_speed} RPM")

# start timer and cleanup
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20) # run update() every 20ms (50Hz/FPS update rate)

def cleanup():
    timer.stop()
    if ser.is_open:
        ser.write(b"STOP\n")
        ser.close()
    if not csv_file.closed:
        csv_file.close()

atexit.register(cleanup)
app.aboutToQuit.connect(cleanup)

main_window.show()

# run GUI
try:
    sys.exit(app.exec())
finally:
    cleanup()
