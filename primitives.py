# -*- coding: utf-8 -*-
import math
from collections import defaultdict

import pypot.primitive


# The constant may be replaced by dict for each motor if needed.
LOAD_LIMIT = 60  # empiric value, can be changed
SPEED_LIMIT = 4*math.pi  # empiric value too (rad/s)
TEMPERATURE_LIMIT = 60  # under 60 degrees, the motors should be safe


class LimitOverLoad(pypot.primitive.LoopPrimitive):
    """
    This primitive avoid the torque to be to intense, therefore potentially damaging to the robot.

    use: robot.attach_primitive(LimitOverLoad(robot, 50), 'limit_overload')
    """
    def __init__(self, robot, freq):
        pypot.primitive.LoopPrimitive.__init__(self, robot, freq)
        self._robot = robot

    def setup(self):
        # before the run loop
        pass

    def update(self):
        # called at a predefined frequency
        overloaded_motors = defaultdict()

        for m in self._robot.motors:
            if abs(m.present_load) > LOAD_LIMIT and not m.compliant:
                overloaded_motors[m.name] = m.present_load

        if overloaded_motors:
            print("overloaded motors: {}".format(overloaded_motors))  # todo replace with logger, add load
            for m in self._robot.motors:
                m.compliant = True

        # todo also stop if the (absolute) load is going up too fast & may be over LOAD_LIMIT soon.

    def teardown(self):
        # after the run loop
        pass


class LimitSpeed(pypot.primitive.LoopPrimitive):
    """
    This primitive avoid the speed to be to fast, therefore potentially damaging to the robot.

    use: robot.attach_primitive(LimitSpeed(robot, 50), 'limit_speed')
    """
    def __init__(self, robot, freq):
        pypot.primitive.LoopPrimitive.__init__(self, robot, freq)
        self._robot = robot
        self._freq = freq
        self.prev_position = {}

    def setup(self):
        # before the run loop
        for m in self._robot.motors:
            self.prev_position[m.name] = [m.present_position]

    def update(self):
        # called at a predefined frequency
        overspeeding_motors = defaultdict()

        for m in self._robot.motors:
            if m.model in ['MX-28', 'MX-64']:
                resolution = 0.088  # degree
            elif m.model in ['AX-12']:
                resolution = 0.29  # degree
            else:
                raise ValueError("unknown motor model.")
            # unfiltered, instantaneous speed
            speed = (m.present_position - self.prev_position[m.name][0]) * resolution / self._freq  # deg/s
            speed = math.radians(speed)  # rad/s
            if abs(speed) > SPEED_LIMIT:
                overspeeding_motors[m.name] = speed

        if overspeeding_motors:
            print("overspeeding motors: {}".format(overspeeding_motors))  # todo replace with logger, add speed
            for m in self._robot.motors:
                m.compliant = True

        # todo also stop if the (absolute) speed is going up too fast & may be over SPEED_LIMIT soon.

    def teardown(self):
        # after the run loop
        pass


class LimitTemperature(pypot.primitive.LoopPrimitive):
    """
    This primitive avoid the temperature to be to high, therefore potentially damaging to the robot.

    use: robot.attach_primitive(LimitTemperature(robot, 50), 'limit_overheat')
    """
    def __init__(self, robot, freq):
        pypot.primitive.LoopPrimitive.__init__(self, robot, freq)
        self._robot = robot

    def setup(self):
        # before the run loop
        pass

    def update(self):
        # called at a predefined frequency
        overheating_motors = defaultdict()

        for m in self._robot.motors:
            if m.present_temperature > TEMPERATURE_LIMIT:
                overheating_motors[m.name] = m.present_temperature

        if overheating_motors:
            print("overheated motors: {}".format(overheating_motors))  # todo replace with logger, add temperature
            for m in self._robot.motors:
                m.compliant = True

        # todo also stop if the temperature is going up too fast & may be over TEMPERATURE_LIMIT soon.

    def teardown(self):
        # after the run loop
        pass


def attach_safety_primitives(robot, start=False, frequency=50):
    """
    Attach all the safety primitives to the Poppy robot.

    :param robot: The poppy robot to protect.
    :param start: Whether or not to start the primitives (default: False).
    :param frequency: Frequency of the primitives (default: 50 Hz).
    :return:
    """
    # todo add LimitSpeed primitive
    robot.attach_primitive(LimitOverLoad(robot, frequency), 'limit_overload')
    robot.attach_primitive(LimitSpeed(robot, frequency), 'limit_speed')
    robot.attach_primitive(LimitTemperature(robot, frequency), 'limit_overheat')

    if start:
        robot.limit_overload.start()
        robot.limit_speed.start()
        robot.limit_overheat.start()
