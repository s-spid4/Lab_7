# servo_translator.py
# MicroPython helper to convert a servo angle (deg) to duty_u16 for Pico PWM (50 Hz)
# Defaults provide a safe 1000-2000 µs pulse mapping (approx 0° -> 1000µs, 180° -> 2000µs).
# Adjust min_pulse_us / max_pulse_us to calibrate for your servos.

def translate(angle,
              min_angle=0.0, max_angle=180.0,
              min_pulse_us=1000.0, max_pulse_us=2000.0,
              pwm_freq=50):
    """
    Convert servo angle (degrees) to duty_u16 for Raspberry Pi Pico PWM.

    angle: desired servo angle in degrees (float)
    min_angle/max_angle: expected servo angle range
    min_pulse_us/max_pulse_us: pulse width corresponding to min_angle/max_angle in microseconds
    pwm_freq: PWM frequency in Hz (50 is typical for hobby servos)

    Returns: integer duty_u16 (0..65535)
    """
    # clamp angle
    if angle < min_angle:
        angle = min_angle
    if angle > max_angle:
        angle = max_angle

    # linear interpolation for pulse width
    frac = (angle - min_angle) / (max_angle - min_angle)
    pulse_us = min_pulse_us + frac * (max_pulse_us - min_pulse_us)

    # period in microseconds
    period_us = 1_000_000.0 / pwm_freq

    # duty fraction to duty_u16
    duty_frac = pulse_us / period_us
    duty_u16 = int(duty_frac * 65535.0)

    # clamp
    if duty_u16 < 0:
        duty_u16 = 0
    if duty_u16 > 65535:
        duty_u16 = 65535

    return duty_u16
