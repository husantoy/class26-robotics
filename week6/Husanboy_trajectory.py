"""Lab 6: draw 39 using encoder-threshold motion, without PID.
Run in TRIK Studio Python mode; M3/E3 left, M4/E4 right.
Odometry observes the motion; x, y and heading never steer the motors.
"""
import math
import time

RADIUS = 0.028  # m: measure on your robot
TRACK = 0.154   # m: wheel-to-wheel distance
PERIOD_MS = 10
DRIVE_POWER = 18
TURN_POWER = 12
WHEEL_TOLERANCE = 0.0001  # m
ACTION_TIMEOUT = 30.0   # seconds
LOG_FILE = "Husanboy_odometry_data.txt"
# Starts at the top-left of 3, heading +x. +y is left.
# False lifts the simulator pen during the gap between the digits.
STROKES = [
    (0.40, 0.00, True), (0.40, -0.30, True),
    (0.00, -0.30, True), (0.40, -0.30, True),
    (0.40, -0.60, True), (0.00, -0.60, True),
    (0.65, 0.00, False),
    (1.05, 0.00, True), (1.05, -0.30, True),
    (0.65, -0.30, True), (0.65, 0.00, True),
    (1.05, 0.00, True), (1.05, -0.60, True),
    (0.65, -0.60, True),  # final bottom stroke of 9
]


def motion_plan():
    """Return (signed left metres, signed right metres, pen, power)."""
    actions = []
    px = py = heading = 0.0
    for x, y, draw in STROKES:
        dx, dy = x - px, y - py
        bearing = math.atan2(dy, dx)
        turn = (bearing - heading + math.pi) % (2 * math.pi) - math.pi
        arc = turn * TRACK / 2
        actions.append((-arc, arc, False, TURN_POWER))
        distance = math.hypot(dx, dy)
        actions.append((distance, distance, draw, DRIVE_POWER))
        px, py, heading = x, y, bearing
    return actions


def integrate(x, y, theta, delta_left_deg, delta_right_deg):
    dl = math.radians(delta_left_deg) * RADIUS
    dr = math.radians(delta_right_deg) * RADIUS
    ds = (dl + dr) / 2
    da = (dr - dl) / TRACK
    return (x + ds * math.cos(theta + da / 2),
            y + ds * math.sin(theta + da / 2), theta + da)


def main():
    clock = getattr(time, "monotonic", time.time)
    start = clock()
    x = y = theta = 0.0
    old_l = brick.encoder("E3").read()
    old_r = brick.encoder("E4").read()
    marker = None
    try:
        marker = brick.marker()
    except AttributeError:
        pass  # Real hardware can have no simulator marker.
    try:
        with open(LOG_FILE, "w") as log:
            log.write("0.000000,0.000000,0\n")
            for left_m, right_m, draw, power in motion_plan():
                if marker is not None:
                    if draw:
                        marker.down("blue")
                    else:
                        marker.up()
                base_l = brick.encoder("E3").read()
                base_r = brick.encoder("E4").read()
                action_start = clock()
                while True:
                    enc_l = brick.encoder("E3").read()
                    enc_r = brick.encoder("E4").read()
                    x, y, theta = integrate(x, y, theta, enc_l-old_l, enc_r-old_r)
                    old_l, old_r = enc_l, enc_r
                    now = clock()
                    # Initial sample is already logged. Each later step waits 10 ms.
                    if now - start >= 0.001:
                        log.write("%.6f,%.6f,%d\n" % (x, y, int((now-start)*1000)))
                        log.flush()
                    travelled_l = math.radians(enc_l-base_l) * RADIUS
                    travelled_r = math.radians(enc_r-base_r) * RADIUS
                    sign_l = (left_m > 0) - (left_m < 0)
                    sign_r = (right_m > 0) - (right_m < 0)
                    done_l = sign_l * travelled_l >= abs(left_m) - WHEEL_TOLERANCE
                    done_r = sign_r * travelled_r >= abs(right_m) - WHEEL_TOLERANCE
                    brick.motor("M3").setPower(0 if done_l else sign_l * power)
                    brick.motor("M4").setPower(0 if done_r else sign_r * power)
                    if done_l and done_r:
                        break
                    if now - action_start > ACTION_TIMEOUT:
                        raise RuntimeError("Encoder motion timeout: check wiring, wheel geometry or obstruction")
                    script.wait(PERIOD_MS)
                # Stop and record settling before starting the next action.
                for _ in range(10):
                    script.wait(PERIOD_MS)
                    enc_l = brick.encoder("E3").read()
                    enc_r = brick.encoder("E4").read()
                    x, y, theta = integrate(x, y, theta, enc_l-old_l, enc_r-old_r)
                    old_l, old_r = enc_l, enc_r
                    log.write("%.6f,%.6f,%d\n" % (x, y, int((clock()-start)*1000)))
    finally:
        brick.motor("M3").setPower(0)
        brick.motor("M4").setPower(0)
        if marker is not None:
            marker.up()


if __name__ == "__main__":
    main()
