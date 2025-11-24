# plotter_main.py
# MicroPython program for SED1115 Lab 7 pen plotter
# Uses servo_translator.translate to generate safe duty_u16 values.

import time
from machine import Pin, PWM
from servo_translator import translate

# --- CONFIG / CALIBRATION ---
# GPIO pins are defined by the expansion board mapping (J1->GPIO0, J2->GPIO1, J3->GPIO2)
SHOULDER_PIN = 0  # J1
ELBOW_PIN    = 1  # J2
WRIST_PIN    = 2  # J3

PWM_FREQ = 50  # typical servo frequency

# Safety: wrist up/down angles (lab recommends 0 degrees for up, ~30 for down)
WRIST_UP_ANGLE = 0.0
WRIST_DOWN_ANGLE = 30.0

# Optional calibration offsets (in degrees) if your physical hardware needs them
SHOULDER_OFFSET = 0.0
ELBOW_OFFSET = 0.0
WRIST_OFFSET = 0.0

# Movement parameters
STEP_DEG = 1.0        # degrees per interpolation step (smaller -> smoother)
STEP_DELAY = 0.01     # seconds between interpolation steps

# Translate function defaults (change if your servos need different min/max pulse widths)
MIN_ANGLE = 0.0
MAX_ANGLE = 180.0
MIN_PULSE_US = 1000.0
MAX_PULSE_US = 2000.0

# --- SETUP PWMs ---
shoulder_pwm = PWM(Pin(SHOULDER_PIN))
elbow_pwm    = PWM(Pin(ELBOW_PIN))
wrist_pwm    = PWM(Pin(WRIST_PIN))

shoulder_pwm.freq(PWM_FREQ)
elbow_pwm.freq(PWM_FREQ)
wrist_pwm.freq(PWM_FREQ)

# track current positions (initialize to a safe mid position)
current_shoulder = 90.0
current_elbow = 90.0
current_wrist = WRIST_UP_ANGLE

def set_servo_angles(s_angle=None, e_angle=None, w_angle=None, immediate=False):
    """
    s_angle, e_angle, w_angle are absolute angles in degrees (or None to not change).
    If immediate is False, the function will interpolate from current angles to target.
    """
    global current_shoulder, current_elbow, current_wrist

    # apply offsets
    if s_angle is not None:
        s_angle += SHOULDER_OFFSET
    if e_angle is not None:
        e_angle += ELBOW_OFFSET
    if w_angle is not None:
        w_angle += WRIST_OFFSET

    # Build list of target angles and start interpolation
    target_s = current_shoulder if s_angle is None else s_angle
    target_e = current_elbow if e_angle is None else e_angle
    target_w = current_wrist if w_angle is None else w_angle

    # If immediate, just set values
    if immediate:
        _write_servo(shoulder_pwm, target_s)
        _write_servo(elbow_pwm, target_e)
        _write_servo(wrist_pwm, target_w)
        current_shoulder, current_elbow, current_wrist = target_s, target_e, target_w
        return

    # Compute number of steps by largest angular change
    max_delta = max(abs(target_s - current_shoulder),
                    abs(target_e - current_elbow),
                    abs(target_w - current_wrist))
    if max_delta < 0.0001:
        return

    steps = int(max_delta / STEP_DEG) + 1

    for i in range(1, steps+1):
        frac = i / steps
        s_val = current_shoulder + frac * (target_s - current_shoulder)
        e_val = current_elbow  + frac * (target_e - current_elbow)
        w_val = current_wrist  + frac * (target_w - current_wrist)

        _write_servo(shoulder_pwm, s_val)
        _write_servo(elbow_pwm, e_val)
        _write_servo(wrist_pwm, w_val)
        time.sleep(STEP_DELAY)

    # finalize
    current_shoulder, current_elbow, current_wrist = target_s, target_e, target_w

def _write_servo(pwm_obj, angle):
    duty = translate(angle,
                     min_angle=MIN_ANGLE, max_angle=MAX_ANGLE,
                     min_pulse_us=MIN_PULSE_US, max_pulse_us=MAX_PULSE_US,
                     pwm_freq=PWM_FREQ)
    pwm_obj.duty_u16(duty)

# Disable servos by setting duty to 0 (M18). Some setups prefer turning off PWM entirely.
def disable_servos():
    shoulder_pwm.deinit()
    elbow_pwm.deinit()
    wrist_pwm.deinit()
    print("All servos disabled (M18).")

# --- G-CODE PARSER / INTERPRETER ---
def parse_gcode_line(line):
    """
    Returns a dict with the parsed command:
      {'cmd': 'G1', 'S': value, 'E': value}
      or {'cmd': 'M3'}, {'cmd':'M5'}, {'cmd':'M18'}
    Unknown/empty lines return None.
    """
    s = line.strip()
    if not s:
        return None
    # ignore comments (anything after ';' or '(' ) - simple approach
    if s.startswith(';') or s.startswith('('):
        return None

    parts = s.split()
    if len(parts) == 0:
        return None
    cmd = parts[0].upper()
    result = {'cmd': cmd}
    # parse parameters like S47.847 E57.273
    for token in parts[1:]:
        token = token.strip()
        if len(token) < 2:
            continue
        letter = token[0].upper()
        try:
            value = float(token[1:])
            result[letter] = value
        except Exception:
            # ignore parse errors for now
            pass
    return result

def run_gcode_file(path):
    """
    Read the file line-by-line and interpret commands.
    """
    print("Running G-Code file:", path)
    try:
        with open(path, "r") as f:
            for raw in f:
                line = raw.strip()
                if not line:
                    continue
                parsed = parse_gcode_line(line)
                if parsed is None:
                    continue

                cmd = parsed.get('cmd')
                # G1 command: move shoulder/elbow
                if cmd == 'G1':
                    s_val = parsed.get('S', None)
                    e_val = parsed.get('E', None)
                    # If neither present, do nothing
                    if s_val is None and e_val is None:
                        continue
                    # Move to those angles (interpolated)
                    set_servo_angles(s_angle=s_val, e_angle=e_val)
                    print("G1 -> S:", s_val, "E:", e_val)

                elif cmd == 'M3':
                    # wrist down (pen down)
                    set_servo_angles(w_angle=WRIST_DOWN_ANGLE)
                    print("M3 -> wrist down")

                elif cmd == 'M5':
                    # wrist up (pen up)
                    set_servo_angles(w_angle=WRIST_UP_ANGLE)
                    print("M5 -> wrist up")

                elif cmd == 'M18':
                    # disable all servos
                    print("M18 -> disable servos")
                    disable_servos()
                    break

                else:
                    print("Unknown command:", cmd, "line:", line)
        print("Finished running file.")
    except Exception as e:
        print("Error opening/reading file:", e)

# --- MAIN ---
def main():
    # Optionally set an initial safe pose (mid angles)
    set_servo_angles(s_angle=90.0, e_angle=90.0, w_angle=WRIST_UP_ANGLE, immediate=True)
    # Name of the G-code file (upload this to the Pico root)
    gcode_filename = "line.gcode"
    run_gcode_file(gcode_filename)

if __name__ == "__main__":
    main()
