"""Measured-person supervisor; has no access to scripted pause times.

Hold on growing separation plus sustained low observed person speed.
The separation gate avoids waiting just because the robot is turning.
All thresholds are experimental, not validated assistive-device settings.
"""
import numpy as np


class GuideSupervisor:
    def __init__(self):
        self.waiting = False
        self.stopped_s = 0.
        self.hold_xy = None
        self.hold_yaw = None

    def update(self, robot_xy, robot_yaw, person_speed, gap, dt):
        if self.waiting:
            if person_speed > .035 and gap < 1.25:
                self.waiting = False
                self.stopped_s = 0.
                return 'resume'
        else:
            self.stopped_s = self.stopped_s + dt if person_speed < .025 and gap > 1.08 else 0.
            if self.stopped_s > .30 or gap > 1.35:
                self.waiting = True
                self.hold_xy = np.array(robot_xy).copy()
                self.hold_yaw = float(robot_yaw)
                return 'wait'
        return None
