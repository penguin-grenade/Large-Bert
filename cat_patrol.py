# -*- coding: utf-8 -*-
"""Yard patrol: watch for the orange intruder cat and shoo it away, humanely.

The robot watches (from one spot or while roaming). When a cat shows up it
stops and studies it. Only once it is sure the cat is the orange intruder,
and no person or one of your cats is in view, does it flash its lights, chirp
and walk toward the cat. It never gets closer than STANDOFF_CM, backs off
instead of cornering a cat that won't leave, gives up after CHASE_SECONDS and
then rests for COOLDOWN_SECONDS.

Run from Code/Server (stop main.py first, they share the hardware):
    python3 cat_patrol.py                 # patrol for real
    python3 cat_patrol.py --dry-run       # watch and log only, never move or beep
    python3 cat_patrol.py --test-hardware # check turn and pan directions
    python3 cat_patrol.py --video clip.mp4 --show   # test detection on a PC
"""
import argparse
import random
import threading
import time

import patrol_config as C
from cat_vision import CatDetector, CatIdentifier, PiCameraSource, VideoSource, Vision, annotate

RED, BLUE, GREEN, AMBER, OFF = (255, 0, 0), (0, 0, 255), (0, 120, 0), (60, 30, 0), (0, 0, 0)


def log(message):
    print(time.strftime("%H:%M:%S"), message, flush=True)


def clamp(value, low, high):
    return max(low, min(high, value))


class Robot:
    """Thin wrapper around the Freenove hardware classes."""
    def __init__(self):
        from adc import ADC
        from buzzer import Buzzer
        from control import Control
        from led import Led
        from ultrasonic import Ultrasonic
        self.control = Control()
        self.servo = self.control.servo
        self.buzzer = Buzzer()
        self.led = Led()
        self.sonar = Ultrasonic()
        self.adc = ADC()
        self._chirping = False

    def power_on(self):
        """Switch the servo power back on (relax cuts it) before moving anything."""
        if self.control.servo_power_disable.value:
            self.control.servo_power_disable.off()
            time.sleep(0.1)

    def power_off(self):
        """Cut power to all servos, head included. Digital servos hold position without it."""
        self.control.servo_power_disable.on()

    def walk(self, forward=0, sideways=0, turn=0, speed=C.PATROL_SPEED, lift=40):
        """One gait cycle. turn is -10..10 degrees, positive = left after TURN_SIGN is set.
        lift is how high each foot rises, in mm (Freenove default 40)."""
        self.power_on()
        self.control.run_gait(["CMD_MOVE", "1", str(int(sideways)), str(int(forward)),
                               str(int(speed)), str(int(clamp(turn, -10, 10) * C.TURN_SIGN))], Z=int(lift))

    def stand(self):
        self.walk()

    def turn_by(self, degrees):
        """Rotate the body roughly `degrees` (positive = left)."""
        cycles = int(round(abs(degrees) / C.DEG_PER_TURN_CYCLE))
        for _ in range(cycles):
            self.walk(turn=10 if degrees > 0 else -10)

    def look(self, pan=C.PAN_CENTER, tilt=C.TILT_CENTER):
        self.power_on()
        self.servo.set_servo_angle(C.HEAD_PAN_CHANNEL, clamp(pan, 50, 180))
        self.servo.set_servo_angle(C.HEAD_TILT_CHANNEL, clamp(tilt, 50, 180))

    def distance(self):
        return self.sonar.get_distance()

    def lights(self, colour):
        if self.led.is_support_led_function:
            self.led.led_index(0x7f, *colour)

    def chirp(self, beeps=3):
        """Short beeps on a background thread so walking isn't held up."""
        if self._chirping or in_quiet_hours():
            return
        def run():
            self._chirping = True
            for _ in range(beeps):
                self.buzzer.set_state(True)
                time.sleep(0.08)
                self.buzzer.set_state(False)
                time.sleep(0.12)
            self._chirping = False
        threading.Thread(target=run, daemon=True).start()

    def battery_ok(self):
        load, pi = self.adc.read_battery_voltage()
        return load >= C.MIN_LOAD_BATTERY_V and pi >= C.MIN_PI_BATTERY_V, (load, pi)

    def rest(self):
        self.buzzer.set_state(False)
        self.lights(OFF)
        self.control.relax(True)
        self.power_off()


class DryRunRobot:
    """Stands in for the hardware: logs what the robot would do."""
    def walk(self, forward=0, sideways=0, turn=0, speed=C.PATROL_SPEED):
        if forward or sideways or turn:
            log("  [dry-run] walk forward=%s sideways=%s turn=%s speed=%s" % (forward, sideways, turn, speed))
        time.sleep(0.3)

    def stand(self):
        pass

    def turn_by(self, degrees):
        if int(round(abs(degrees) / C.DEG_PER_TURN_CYCLE)):
            log("  [dry-run] turn %+d degrees" % degrees)

    def look(self, pan=C.PAN_CENTER, tilt=C.TILT_CENTER):
        pass

    def distance(self):
        return None

    def lights(self, colour):
        pass

    def chirp(self, beeps=3):
        log("  [dry-run] chirp")

    def battery_ok(self):
        return True, (None, None)

    def rest(self):
        pass


def in_quiet_hours():
    if not C.QUIET_HOURS:
        return False
    start, end = C.QUIET_HOURS
    hour = time.localtime().tm_hour
    return start <= hour < end if start < end else (hour >= start or hour < end)


class Patrol:
    def __init__(self, robot, vision, show=False):
        self.robot = robot
        self.vision = vision
        self.show = show
        self.pan = C.PAN_CENTER
        self.scan_index = 0
        self.steps_since_scan = 0
        self.last_battery_check = 0.0
        self.encounters = []
        self.running = True

    # ---- helpers --------------------------------------------------------
    def look(self):
        intruder, my_cat, person, detections = self.vision.look()
        if self.show and self.vision.last_frame is not None:
            import cv2
            cv2.imshow("cat patrol", annotate(self.vision.last_frame, detections))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.running = False
        if getattr(self.vision.source, "ended", False):
            self.running = False
        return intruder, my_cat, person, detections

    def heading_to(self, detection):
        """Degrees the body must turn (positive = left) to face a detection."""
        head_offset = (self.pan - C.PAN_CENTER) * C.PAN_SIGN
        return head_offset - detection.cx * C.CAMERA_HFOV_DEG / 2

    def face(self, detection):
        self.robot.turn_by(self.heading_to(detection))
        self.pan = C.PAN_CENTER
        self.robot.look(self.pan)

    def check_battery(self):
        if time.time() - self.last_battery_check < C.BATTERY_CHECK_SECONDS:
            return True
        self.last_battery_check = time.time()
        ok, volts = self.robot.battery_ok()
        if not ok:
            log("Battery low %s, sitting down. Charge me!" % (volts,))
            self.robot.chirp(5)
            time.sleep(1)
            self.robot.rest()
        return ok

    # ---- states ---------------------------------------------------------
    def run(self):
        log("Patrol started in %s mode." % C.MODE)
        self.robot.stand()
        self.robot.look()
        while self.running and self.check_battery():
            cat = self.watch()
            if cat is None:
                continue
            log("Cat spotted (%s, %.0f%% orange). Taking a closer look." % (cat.verdict, cat.orange * 100))
            self.face(cat)
            verdict = self.investigate()
            if verdict == "intruder":
                self.encounters = [t for t in self.encounters if time.time() - t < 3600] + [time.time()]
                outcome = self.deter()
                rest = C.COOLDOWN_SECONDS
                if len(self.encounters) >= C.MAX_ENCOUNTERS_PER_HOUR:
                    rest = max(rest, int(3600 - (time.time() - self.encounters[0])))
                log("Encounter over: %s. Resting %ds." % (outcome, rest))
                self.cooldown(rest)
            elif verdict == "mine":
                log("That's one of ours. Carrying on.")
                self.robot.lights(GREEN)
                self.cooldown(C.IGNORE_SECONDS)
            elif verdict == "cat gone":
                log("Cat went out of view.")
            else:
                log("Leaving it alone (%s)." % verdict)
                self.cooldown(C.IGNORE_SECONDS)
        self.robot.rest()

    def watch(self):
        """Sweep or roam until any cat is seen. Returns the first detection, or None."""
        if C.MODE == "roam" and self.steps_since_scan < C.ROAM_STEPS_BETWEEN_SCANS:
            self.roam_step()
        else:
            self.pan = C.SCAN_ANGLES[self.scan_index % len(C.SCAN_ANGLES)]
            self.scan_index += 1
            self.robot.look(self.pan)
            if self.scan_index % len(C.SCAN_ANGLES) == 0:
                self.steps_since_scan = 0
            deadline = time.time() + C.SCAN_DWELL_SECONDS
            while time.time() < deadline and self.running:
                found = self.first_cat(self.look()[3])
                if found:
                    return found
            return None
        return self.first_cat(self.look()[3])

    @staticmethod
    def first_cat(detections):
        cats = [d for d in detections if d.label == "cat"]
        return max(cats, key=lambda d: d.area) if cats else None

    def roam_step(self):
        self.steps_since_scan += 1
        self.pan = C.PAN_CENTER
        self.robot.look()
        distance = self.robot.distance()
        if distance is not None and distance < C.OBSTACLE_CM:
            log("Obstacle at %.0f cm, turning." % distance)
            self.robot.turn_by(random.choice([-1, 1]) * random.randint(60, 120))
        else:
            self.robot.walk(forward=C.STEP_LENGTH, speed=C.PATROL_SPEED)

    def investigate(self):
        """Stand still and look several times before deciding anything."""
        self.robot.stand()
        self.robot.lights(AMBER)
        self.vision.reset_votes()
        deadline = time.time() + C.INVESTIGATE_SECONDS
        last_seen = time.time()
        while time.time() < deadline and self.running:
            intruder, my_cat, person, detections = self.look()
            if self.first_cat(detections):
                last_seen = time.time()
            if self.vision.confirmed_intruder():
                if person and C.HOLD_OFF_FOR_PEOPLE:
                    return "person nearby"
                if my_cat and C.HOLD_OFF_FOR_MY_CATS:
                    return "my cat nearby"
                return "intruder"
            if list(self.vision.votes).count("mine") >= C.VOTES_NEEDED:
                return "mine"
            if time.time() - last_seen > C.LOST_SECONDS:
                self.robot.lights(OFF)
                return "cat gone"
        self.robot.lights(OFF)
        return "unsure"

    def deter(self):
        log("Intruder confirmed! Shooing it away.")
        start = time.time()
        last_seen = time.time()
        last_chirp = 0.0
        too_close_since = None
        flash = False
        while self.running:
            now = time.time()
            if now - start > C.CHASE_SECONDS:
                return "time limit reached"
            intruder, my_cat, person, _ = self.look()
            if person and C.HOLD_OFF_FOR_PEOPLE:
                self.robot.stand()
                return "person in view, stopped"
            if my_cat and C.HOLD_OFF_FOR_MY_CATS:
                self.robot.stand()
                return "one of our cats in view, stopped"

            flash = not flash
            self.robot.lights(RED if flash else BLUE)
            if now - last_chirp > C.CHIRP_EVERY_SECONDS:
                self.robot.chirp()
                last_chirp = now

            if intruder is None:
                if now - last_seen > C.LOST_SECONDS:
                    self.robot.stand()
                    return "intruder left"
                continue
            last_seen = now

            if abs(intruder.cx) > 0.25:
                self.robot.walk(turn=clamp(-intruder.cx * C.CAMERA_HFOV_DEG / 2, -10, 10),
                                speed=C.APPROACH_SPEED)
                continue

            distance = self.robot.distance()
            close = intruder.area >= C.CLOSE_AREA_FRACTION or (distance is not None and distance < C.STANDOFF_CM)
            if close:
                self.robot.stand()
                too_close_since = too_close_since or now
                if now - too_close_since > C.CORNER_SECONDS:
                    self.back_off()
                    return "intruder stood its ground, backed off"
            else:
                too_close_since = None
                self.robot.walk(forward=C.STEP_LENGTH, speed=C.APPROACH_SPEED)
        return "stopped"

    def back_off(self):
        self.robot.lights(OFF)
        for _ in range(3):
            self.robot.walk(forward=-C.STEP_LENGTH, speed=C.PATROL_SPEED)
        self.robot.stand()

    def cooldown(self, seconds):
        self.robot.stand()
        self.robot.look()
        self.pan = C.PAN_CENTER
        self.vision.reset_votes()
        end = time.time() + seconds
        while time.time() < end and self.running:
            self.look()  # keep logging what we see, but don't act
        self.robot.lights(OFF)


def test_hardware(robot):
    print("Watch the robot and note which way it goes.")
    print("1) Two steps forward")
    for _ in range(2):
        robot.walk(forward=C.STEP_LENGTH)
    print("2) Turning LEFT about 30 degrees. If it turned right, set TURN_SIGN = -1.")
    robot.turn_by(30)
    robot.stand()
    time.sleep(1)
    print("3) Camera looks to the robot's LEFT. If it looked right, set PAN_SIGN = -1.")
    robot.look(C.PAN_CENTER + 40)
    time.sleep(1.5)
    robot.look()
    print("4) Lights and chirp")
    for colour in (RED, BLUE, OFF):
        robot.lights(colour)
        time.sleep(0.5)
    robot._chirping = False
    robot.chirp()
    time.sleep(1)
    print("5) Distance ahead: %s cm" % robot.distance())
    print("6) Battery: %s" % (robot.battery_ok()[1],))
    robot.rest()


def main():
    parser = argparse.ArgumentParser(description="Yard patrol that shoos away the orange intruder cat.")
    parser.add_argument("--dry-run", action="store_true", help="watch and log only, never move or beep")
    parser.add_argument("--video", help="video file or webcam number to use instead of the Pi camera (implies --dry-run)")
    parser.add_argument("--show", action="store_true", help="show the camera view with boxes (needs a screen)")
    parser.add_argument("--test-hardware", action="store_true", help="check walking, turning, head, lights, buzzer")
    parser.add_argument("--mode", choices=["sentry", "roam"], help="override MODE in patrol_config.py")
    args = parser.parse_args()
    if args.mode:
        C.MODE = args.mode

    if args.test_hardware:
        test_hardware(Robot())
        return

    dry_run = args.dry_run or args.video is not None
    robot = DryRunRobot() if dry_run else Robot()
    source = VideoSource(args.video) if args.video is not None else PiCameraSource()
    identifier = CatIdentifier()
    log("Reference photos: %d intruder, %d mine. %s" % (
        len(identifier.refs["intruder"]), len(identifier.refs["mine"]),
        "Using photo matching." if identifier.use_refs else
        "Photo matching OFF (needs %s and photos in both folders), using the strict orange check." % C.EMBEDDER_FILE))
    patrol = Patrol(robot, Vision(source, CatDetector(), identifier), show=args.show)
    try:
        patrol.run()
    except KeyboardInterrupt:
        log("Stopping.")
    finally:
        robot.rest()
        source.close()


if __name__ == "__main__":
    main()
