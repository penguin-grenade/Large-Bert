# -*- coding: utf-8 -*-
"""Faster servo writes for Freenove's control.py.

Every gait frame, control.set_leg_angles() sets 18 leg servos. Freenove's
pca9685.set_pwm sends each one as 4 separate one-byte I2C writes, so a frame
costs 72 I2C transactions: about 26-31 ms at the Pi's default 100 kHz, most of
a ~37 ms frame (the rest is a 10 ms sleep; the leg maths takes 0.4 ms).

install(control) makes set_leg_angles collect the 18 positions and send them
as block writes instead: the PCA9685's auto-increment lets one transaction
fill a run of up to 8 neighbouring channels (32 bytes, the smbus block limit).
Bert's leg channels form 4 runs (board 0x41: 8-15; board 0x40: 0-7, 11, 15),
so a frame takes 4 transactions. Everything else (head, relax) still goes
through Freenove's normal path, and if a block write fails, that run falls
back to the normal one-byte writes.
"""

MODE1 = 0x00
AUTO_INCREMENT = 0x20
LED0_ON_L = 0x06
MAX_RUN = 8                      # 8 channels x 4 bytes = 32 bytes, smbus's block limit


def duty_for(angle):
    """Same maths as Freenove's servo.set_servo_angle: 0..180 deg -> 500..2500 us -> 12-bit at 50 Hz."""
    pulse = (2500 - 500) * angle / 180.0 + 500
    return int(4095 * pulse / 20000.0)


def runs(channels):
    """Split sorted channel numbers into runs of neighbours, at most MAX_RUN long."""
    out, run = [], []
    for ch in sorted(channels):
        if run and (ch != run[-1] + 1 or len(run) == MAX_RUN):
            out.append(run)
            run = []
        run.append(ch)
    if run:
        out.append(run)
    return out


class FastServo:
    def __init__(self, control, log=print):
        self.control = control
        self.servo = control.servo
        self.log = log
        self.enabled = True
        self.batch = None
        self.block_writes = 0
        self.fallbacks = 0
        self._orig_set_angle = self.servo.set_servo_angle
        self._orig_set_legs = control.set_leg_angles
        for pwm in (self.servo.pwm_40, self.servo.pwm_41):
            mode = pwm.read(MODE1)
            pwm.write(MODE1, (mode & 0x7F) | AUTO_INCREMENT)   # bit 7 is RESTART: writing 0 leaves it alone
        self.servo.set_servo_angle = self._set_angle
        control.set_leg_angles = self._set_legs

    def _set_angle(self, channel, angle):
        if self.batch is None:
            return self._orig_set_angle(channel, angle)
        if channel < 16:
            self.batch[(self.servo.pwm_41, channel)] = angle
        else:
            self.batch[(self.servo.pwm_40, channel - 16)] = angle

    def _set_legs(self):
        if not self.enabled:
            return self._orig_set_legs()
        self.batch = {}
        try:
            self._orig_set_legs()
            batch = self.batch
        finally:
            self.batch = None
        self._flush(batch)

    def _flush(self, batch):
        by_board = {}
        for (pwm, ch), angle in batch.items():
            by_board.setdefault(pwm, {})[ch] = angle
        for pwm, chans in by_board.items():
            for run in runs(chans):
                data = []
                for ch in run:
                    off = duty_for(chans[ch])
                    data += [0, 0, off & 0xFF, off >> 8]
                try:
                    pwm.bus.write_i2c_block_data(pwm.address, LED0_ON_L + 4 * run[0], data)
                    self.block_writes += 1
                except OSError as exc:
                    self.fallbacks += 1
                    if self.fallbacks <= 3:
                        self.log("Servo block write failed (%s); using single writes for this run" % exc)
                    for ch in run:
                        pwm.set_pwm(ch, 0, duty_for(chans[ch]))


def install(control, log=print):
    """Patch a control.Control so set_leg_angles uses block writes. Returns the FastServo (set .enabled to switch)."""
    existing = getattr(control, "_fast_servo", None)
    if existing is not None:
        return existing
    fast = FastServo(control, log)
    control._fast_servo = fast
    return fast
