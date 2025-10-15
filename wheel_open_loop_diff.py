#!/usr/bin/env python3
"""
Inputs:
y: forward, 0 is stop, 1 is full speed
t: turning, -1 is left, 1 is right, 0 is straight but these both scale depending on input

These formulas are used for the diff, allow inputs to be mixed. I just looked up this formula on a forum so it might need changing but seems to work
 - left_target = y * (1.0 + t) / 2.0
 - right_target = y * (1.0 - t) / 2.0

Outputs are scaled to duty cycle using BASE_DUTY and MAX_DUTY.
Ramping uses a slew limiter so PWM duty changes smoothly. I did this coz it turns out PID loops would require more hardware to work properly, but this does the same thing
"""
import time
import RPi.GPIO as GPIO

# GPIO pins
LEFT_DIR_PIN  = 17   # AIN1 (just as reminder of what pins to connect to motor driver)
LEFT_PWM_PIN  = 18   # PWMA
RIGHT_DIR_PIN = 23   # BIN1
RIGHT_PWM_PIN = 24   # PWMB

# PWM and the ramp values
PWM_FREQ = 1000
MAX_DUTY = 90.0
MIN_DUTY = 0.0
BASE_DUTY = 80.0
RAMP_STEP = 2.0
RAMP_INTERVAL = 0.04

ZERO_DEAD = 0.02    # I'm just ignoring super small values here, you can raise or lower this value if you need :)

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(LEFT_DIR_PIN,  GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(RIGHT_DIR_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(LEFT_PWM_PIN,  GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(RIGHT_PWM_PIN, GPIO.OUT, initial=GPIO.LOW)

left_pwm = GPIO.PWM(LEFT_PWM_PIN, PWM_FREQ)
right_pwm = GPIO.PWM(RIGHT_PWM_PIN, PWM_FREQ)
left_pwm.start(0.0)
right_pwm.start(0.0)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def ramp_step(current, target):
    if abs(target - current) <= RAMP_STEP:
        return target
    return current + RAMP_STEP * (1 if target > current else -1)

def is_zero(v):
    return abs(v) <= ZERO_DEAD

def compute_mixed_targets(y: float, t: float):

    # reset inputs or you get problems and stuff :/
    y_s = clamp(y, 0.0, 1.0)
    t_s = clamp(t, -1.0, 1.0)
    if is_zero(y_s):
        return 0.0, 0.0
    # differential mixing formulas I used, but you can change to what you need:
    left_cmd = y_s * (1.0 + t_s) / 2.0
    right_cmd = y_s * (1.0 - t_s) / 2.0
    # scale it
    left_duty  = clamp(BASE_DUTY * left_cmd, MIN_DUTY, MAX_DUTY)
    right_duty = clamp(BASE_DUTY * right_cmd, MIN_DUTY, MAX_DUTY)
    return left_duty, right_duty

def set_pwms(duty_l, duty_r):
    left_pwm.ChangeDutyCycle(duty_l)
    right_pwm.ChangeDutyCycle(duty_r)

def main_loop(inputs_iterator):

    cur_l = 0.0
    cur_r = 0.0
    try:
        for item in inputs_iterator:
            if len(item) == 3:
                y, t, duration = item
                end = time.time() + duration
                while time.time() < end:
                    tgt_l, tgt_r = compute_mixed_targets(y, t)
                    cur_l = clamp(ramp_step(cur_l, tgt_l), MIN_DUTY, MAX_DUTY)
                    cur_r = clamp(ramp_step(cur_r, tgt_r), MIN_DUTY, MAX_DUTY)
                    set_pwms(cur_l, cur_r)
                    time.sleep(RAMP_INTERVAL)
            else:
                y, t = item
                tgt_l, tgt_r = compute_mixed_targets(y, t)
                cur_l = clamp(ramp_step(cur_l, tgt_l), MIN_DUTY, MAX_DUTY)
                cur_r = clamp(ramp_step(cur_r, tgt_r), MIN_DUTY, MAX_DUTY)
                set_pwms(cur_l, cur_r)
                time.sleep(RAMP_INTERVAL)
    except KeyboardInterrupt:
        pass
    finally:
        # this ramps the motors down to zero safely on exit
        while cur_l > 0.0 or cur_r > 0.0:
            cur_l = clamp(ramp_step(cur_l, 0.0), MIN_DUTY, MAX_DUTY)
            cur_r = clamp(ramp_step(cur_r, 0.0), MIN_DUTY, MAX_DUTY)
            set_pwms(cur_l, cur_r)
            time.sleep(RAMP_INTERVAL)
        left_pwm.ChangeDutyCycle(0.0)
        right_pwm.ChangeDutyCycle(0.0)
        left_pwm.stop()
        right_pwm.stop()
        GPIO.cleanup()

# Here's a demo of the motors, you can change this to better simulate the real values but this just tests the different configurations I could think of
if __name__ == "__main__":
    demo = [
        (0.6, 0.0, 4.0),   # forward 60% speed for 4s
        (0.8, 0.5, 4.0),   # forward 80% with right turn
        (0.5, -0.7, 4.0),  # forward 50% with left turn
        (1.0, 0.0, 4.0),   # full speed forward
        (0.7, 0.8, 4.0),   # forward 70% with right turn
        (0.0, 0.0, 2.0),   # stop :)
    ]
    main_loop(iter(demo))
