import serial
import time
import re
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox
from matplotlib.widgets import RadioButtons

# Serial config
PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0)
ser.reset_input_buffer()
ser.write(b"MATLAB ON\n")
ser.flush()
time.sleep(0.2)

running = True

# Store recent records
# Keep enough history so larger X spans have actual data to render.
MAX_RECORDS = 50000
HISTORY_SECONDS = 180.0

records = deque(maxlen=MAX_RECORDS)

FIELD_INDEX = {
    "pulses": 3,
    "rpm": 4,
    "omega": 5,
    "current": 6,
    "pwm": 1,
    "dir": 2,
}

selected_field = "pulses"
last_status = "Type a command and press Enter."
x_window_sec = 10.0
y_span = None

# Create figure
fig = plt.figure(figsize=(12, 8), constrained_layout=True)
gs = fig.add_gridspec(4, 2, width_ratios=[4.5, 1.2], height_ratios=[5, 0.8, 0.8, 0.8])

ax = fig.add_subplot(gs[0, 0])
command_ax = fig.add_subplot(gs[1, 0])
xspan_ax = fig.add_subplot(gs[2, 0])
yspan_ax = fig.add_subplot(gs[3, 0])
radio_ax = fig.add_subplot(gs[:, 1])
command_ax.set_xticks([])
command_ax.set_yticks([])
command_ax.set_facecolor("#f4f4f4")
xspan_ax.set_xticks([])
xspan_ax.set_yticks([])
xspan_ax.set_facecolor("#f4f4f4")
yspan_ax.set_xticks([])
yspan_ax.set_yticks([])
yspan_ax.set_facecolor("#f4f4f4")
radio_ax.set_facecolor("#fafafa")

line1, = ax.plot([], [], label="Selected variable")

ax.set_title("Real-Time Serial Telemetry from CSV")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Value")
ax.legend()
ax.grid(True, alpha=0.3)

command_box = TextBox(command_ax, "ESP32 command", initial="")
xspan_box = TextBox(xspan_ax, "X span [s]", initial=str(x_window_sec))
yspan_box = TextBox(yspan_ax, "Y span [units|auto]", initial="auto")
radio = RadioButtons(radio_ax, tuple(FIELD_INDEX.keys()), active=0)

status_text = fig.text(0.15, 0.01, last_status, fontsize=10)

CSV_COLUMNS = 7
TIME_INDEX = 0
NUMBER_RE = re.compile(r"[-+]?\d*\.?\d+")


def get_selected_index():
    return FIELD_INDEX[selected_field]


def update_labels():
    if selected_field == "dir":
        ax.set_ylabel("Direction")
    elif selected_field == "current":
        ax.set_ylabel("Current [A]")
    elif selected_field == "rpm":
        ax.set_ylabel("Speed [RPM]")
    elif selected_field == "omega":
        ax.set_ylabel("Speed [rad/s]")
    elif selected_field == "pwm":
        ax.set_ylabel("PWM")
    else:
        ax.set_ylabel("Position [pulses]")

    ax.set_title(f"Real-Time Serial Telemetry from CSV: {selected_field}")
    fig.canvas.draw_idle()


def on_field_change(label):
    global selected_field
    selected_field = label
    update_labels()


radio.on_clicked(on_field_change)


def set_x_span(value):
    global x_window_sec
    global last_status

    text = value.strip().lower()
    if text == "auto":
        x_window_sec = None
        last_status = "X axis span: auto"
        status_text.set_text(last_status)
        fig.canvas.draw_idle()
        return

    try:
        span = float(text)
        if span <= 0:
            raise ValueError
    except ValueError:
        last_status = "Invalid X span. Use positive number or 'auto'."
        status_text.set_text(last_status)
        fig.canvas.draw_idle()
        return

    x_window_sec = span
    last_status = f"X axis span set to {span:.3f} s"
    status_text.set_text(last_status)
    fig.canvas.draw_idle()


def set_y_span(value):
    global y_span
    global last_status

    text = value.strip().lower()
    if text == "auto":
        y_span = None
        last_status = "Y axis span: auto"
        status_text.set_text(last_status)
        fig.canvas.draw_idle()
        return

    try:
        span = float(text)
        if span <= 0:
            raise ValueError
    except ValueError:
        last_status = "Invalid Y span. Use positive number or 'auto'."
        status_text.set_text(last_status)
        fig.canvas.draw_idle()
        return

    y_span = span
    last_status = f"Y axis span set to {span:.3f}"
    status_text.set_text(last_status)
    fig.canvas.draw_idle()


xspan_box.on_submit(set_x_span)
yspan_box.on_submit(set_y_span)


def send_command(command):
    global running
    global last_status

    command = command.strip()
    if not command:
        last_status = "Enter a command before sending."
        status_text.set_text(last_status)
        fig.canvas.draw_idle()
        return

    if command.lower() in {"exit", "quit"}:
        running = False
        try:
            ser.write(b"STOP\n")
        except Exception:
            pass
        plt.close(fig)
        return

    try:
        ser.write((command + "\n").encode("utf-8"))
        ser.flush()
        last_status = f"Sent: {command}"
        status_text.set_text(last_status)
    except Exception as exc:
        last_status = f"Error sending command: {exc}"
        status_text.set_text(last_status)

    command_box.set_val("")
    fig.canvas.draw_idle()


command_box.on_submit(send_command)


def parse_csv_row(line):
    values = line.split(",")
    if len(values) == CSV_COLUMNS:
        try:
            return tuple(float(item) for item in values)
        except ValueError:
            return None

    # Fallback for mixed serial lines with stray prefixes/suffixes.
    nums = NUMBER_RE.findall(line)
    if len(nums) < CSV_COLUMNS:
        return None

    try:
        return tuple(float(item) for item in nums[:CSV_COLUMNS])
    except ValueError:
        return None

def update(frame):
    global last_status
    try:
        parsed_count = 0
        last_line = ""
        # Drain buffered serial lines to stay synchronized with live telemetry.
        while ser.in_waiting > 0:
            raw = ser.readline()
            if not raw:
                break

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            last_line = line
            row = parse_csv_row(line)
            if row is not None:
                records.append(row)
                parsed_count += 1

        if parsed_count == 0:
            if last_line and "," not in last_line:
                last_status = last_line[:120]
                status_text.set_text(last_status)
            return line1,

        if records:
            newest_time_s = records[-1][TIME_INDEX] / 1000.0
            while records and ((newest_time_s - (records[0][TIME_INDEX] / 1000.0)) > HISTORY_SECONDS):
                records.popleft()

        if records:
            times = [row[TIME_INDEX] / 1000.0 for row in records]
            values_y = [row[get_selected_index()] for row in records]
            line1.set_data(times, values_y)

            latest_value = values_y[-1]

            if x_window_sec is None:
                ax.relim()
                ax.autoscale_view(scalex=True, scaley=False)
            else:
                end_x = times[-1]
                start_x = end_x - x_window_sec
                ax.set_xlim(start_x, end_x)

            if y_span is None:
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)
            else:
                half_span = y_span / 2.0
                ax.set_ylim(latest_value - half_span, latest_value + half_span)

            last_status = f"Plot: {selected_field} | latest={latest_value:.4f}"
            status_text.set_text(last_status)

        if selected_field == "dir":
            line1.set_drawstyle("steps-post")
        else:
            line1.set_drawstyle("default")

    except Exception as e:
        last_status = f"Serial error: {e}"
        status_text.set_text(last_status)

    return line1,

ani = FuncAnimation(fig, update, interval=20, cache_frame_data=False)

update_labels()

try:
    plt.show()
finally:
    running = False
    ser.close()
