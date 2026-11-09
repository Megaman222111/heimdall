"""All tunables for the Pi radar + vision tracker."""

# ---------------- Web ----------------
WEB_HOST = "0.0.0.0"
WEB_PORT = 80

# ---------------- Radar (HLK-LD2450 on UART3: GPIO4 TX, GPIO5 RX) ----------------
RADAR_UART_ADDR = "fe201600"  # UART3 on the Pi 4; used to find its /dev/ttyAMA* name
RADAR_BAUD = 256000
RADAR_X_SIGN = 1              # -1 if radar left/right is mirrored vs the camera
RADAR_MAX_RANGE_MM = 6000

# ---------------- Camera + detection ----------------
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_HFLIP = False
CAM_VFLIP = False
CAM_HFOV_DEG = 53.5           # OV5647 horizontal field of view
CAM_VFOV_DEG = 41.4
CAM_X_SIGN = 1                # -1 if camera boxes are mirrored
MODEL_PATH = "models/yolo11n-320.onnx"
MODEL_SIZE = 320
MODEL_THREADS = 3             # leaves one core for radar, servo and web
VISION_MIN_SCORE = 0.40
VISION_NMS_IOU = 0.45
JPEG_QUALITY = 70
PERSON_HEIGHT_M = 1.70        # estimates range for camera-only targets

# ---------------- Fusion / tracking ----------------
MAX_TRACKS = 3
MATCH_DEG = 8.0               # radar <-> camera bearing match gate
TRACK_GATE_DEG = 12.0
TRACK_GATE_MM = 900
TRACK_TIMEOUT_MS = 900
VISION_STALE_MS = 450
RADAR_STALE_MS = 400

# ---------------- Servo (hardware PWM on GPIO18) ----------------
PWM_CHIP = "/sys/class/pwm/pwmchip0"
PWM_CHANNEL = 0
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_CENTER_DEG = 90         # servo angle where the head points straight ahead
PAN_MIN_DEG = -90             # narrow these if the servo buzzes at its end stops
PAN_MAX_DEG = 90
SERVO_INVERT = False          # True if a positive command turns the head the wrong way
PAN_GAIN = 0.35               # fraction of error corrected per 50 Hz tick
PAN_MAX_DEG_S = 60.0
PAN_DEADBAND_DEG = 2.0
FOLLOW_RADAR_ONLY = True      # turn toward radar-only targets when the camera sees nobody
IDLE_RETURN_MS = 8000         # with sweep off: return to centre after this long (0 = hold)

# ---------------- Sweep (search when nobody is tracked) ----------------
SWEEP_ENABLED = True
SWEEP_START_MS = 1500
SWEEP_DEG_S = 25.0
