import time
import sys
import csv
import serial
import atexit
from collections import deque
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# --- configuration ---
port_name = "/dev/cu.debug-console"
baud = 115200
filename = "stm_data_" + time.strftime("%Y_%m_%d-%H_%M_%S") + ".csv"
filepath = "C:\\Users\\mitmu\\Documents\\stm-control\\stm_motor_recordings\\" # change on each computer
sample_rate = 1 # set in arduino, informs program of sample rate in milliseconds
window_size = 2000 # how many data points to show at once
speed_average_time = 500 # how many data points to average for speed calculations

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
obj_short_hist = deque(maxlen=speed_average_time)
mir_short_hist = deque(maxlen=speed_average_time)

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

rpm_label = QtWidgets.QLabel("target RPM:")
rpm_input = QtWidgets.QLineEdit("60")
rpm_input.setFixedWidth(80)
btn_set_rpm = QtWidgets.QPushButton("set speed")

control_layout.addWidget(btn_start)
control_layout.addWidget(btn_stop)
control_layout.addSpacing(20)
control_layout.addWidget(rpm_label)
control_layout.addWidget(rpm_input)
control_layout.addWidget(btn_set_rpm)
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

def send_rpm():
    if ser.is_open:
        rpm_val = rpm_input.text().strip()
        if rpm_val.isdigit() and int(rpm_val) > 0:
            cmd = f"{rpm_val}\n".encode("utf-8")
            ser.write(cmd)

btn_start.clicked.connect(send_start)
btn_stop.clicked.connect(send_stop)
btn_set_rpm.clicked.connect(send_rpm)
rpm_input.returnPressed.connect(send_rpm)

# --- speed calculations ---
def calcSpeed(window):
    distance = window[-1] - window[0]
    if distance < 0:
        distance += 360
    time = speed_average_time * sample_rate * 0.001
    return round((distance / time), 1)

# --- main loop ---
def update():
    updated = False
    # read everything in the serial buffer
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line or line.startswith("Target") or line.startswith("SUCCESS") or line.startswith("SYSTEM"):
                continue

            parts = line.split(",")
            timestamp, obj_val, mir_val, trig_sent = None, None, None, None

            for part in parts:
                if "Timestamp:" in part:
                    timestamp = int(part.split(":")[1])
                elif "Object_Angle:" in part:
                    obj_val = float(part.split(":")[1])
                elif "Mirror_Angle:" in part:
                    mir_val = float(part.split(":")[1])
                elif "Trigger_Sent:" in part:
                    trig_sent = bool(int(part.split(":")[1]))

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

        except (ValueError, IndexError):
            # skip unparseable lines
            pass
    if updated:
        # update plot data
        curve_obj.setData(list(obj_history))
        curve_mir.setData(list(mir_history))

        obj_speed = calcSpeed(obj_short_hist)
        mir_speed = calcSpeed(mir_short_hist)
        actual_obj_rpm.setText(f"actual object speed: {obj_speed} RPM")
        actual_mir_rpm.setText(f"actual object speed: {mir_speed} RPM")

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
