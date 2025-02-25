from typing import Any, Callable

import csv
import glob
import signal
import sys
import time
from math import sqrt

import serial

PRECISION_OF_SLEEP = 0.0001


class LoopKiller:

    """
    Soft Realtime Loop---a class designed to allow clean exits from infinite loops
    with the potential for post-loop cleanup operations executing.

    The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
    when it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
    Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

    the function_in_loop argument to the Soft Realtime Loop's blocking_loop method is the function to be run every loop.
    A typical usage would set function_in_loop to be a method of an object, so that the object could store program state.
    See the 'ifmain' for two examples.

    # This library will soon be hosted as a PIP module and added as a python dependency.
    # https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/main/SoftRealtimeLoop.py

    Author: Gray C. Thomas, Ph.D
    https://github.com/GrayThomas, https://graythomas.github.io

    """

    def __init__(self, fade_time=0.0):
        signal.signal(signal.SIGTERM, self.handle_signal)
        signal.signal(signal.SIGINT, self.handle_signal)
        signal.signal(signal.SIGHUP, self.handle_signal)
        self._fade_time = fade_time
        self._soft_kill_time = None

    def handle_signal(self, signum, frame):
        self.kill_now = True

    def get_fade(self):
        # interpolates from 1 to zero with soft fade out
        if self._kill_soon:
            t = time.time() - self._soft_kill_time
            if t >= self._fade_time:
                return 0.0
            return 1.0 - (t / self._fade_time)
        return 1.0

    _kill_now = False
    _kill_soon = False

    @property
    def kill_now(self):
        if self._kill_now:
            return True
        if self._kill_soon:
            t = time.time() - self._soft_kill_time
            if t > self._fade_time:
                self._kill_now = True
        return self._kill_now

    @kill_now.setter
    def kill_now(self, val):
        if val:
            if self._kill_soon:  # if you kill twice, then it becomes immediate
                self._kill_now = True
            else:
                if self._fade_time > 0.0:
                    self._kill_soon = True
                    self._soft_kill_time = time.time()
                else:
                    self._kill_now = True
        else:
            self._kill_now = False
            self._kill_soon = False
            self._soft_kill_time = None


class SoftRealtimeLoop:
    """
    Soft Realtime Loop---a class designed to allow clean exits from infinite loops
    with the potential for post-loop cleanup operations executing.

    The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
    when it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
    Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

    the function_in_loop argument to the Soft Realtime Loop's blocking_loop method is the function to be run every loop.
    A typical usage would set function_in_loop to be a method of an object, so that the object could store program state.
    See the 'ifmain' for two examples.

    This library will soon be hosted as a PIP module and added as a python dependency.
    https://github.com/UM-LoCoLab/NeuroLocoMiddleware/blob/main/SoftRealtimeLoop.py

    # Author: Gray C. Thomas, Ph.D
    # https://github.com/GrayThomas, https://graythomas.github.io

    """

    def __init__(self, dt=0.001, report=False, fade=0.0):
        self.t0 = self.t1 = time.time()
        self.killer = LoopKiller(fade_time=fade)
        self.dt = dt
        self.ttarg = None
        self.sum_err = 0.0
        self.sum_var = 0.0
        self.sleep_t_agg = 0.0
        self.n: int = 0
        self.report = report

    def __del__(self):
        if self.report:
            print("In %d cycles at %.2f Hz:" % (self.n, 1.0 / self.dt))
            print("\tavg error: %.3f milliseconds" % (1e3 * self.sum_err / self.n))
            print(
                "\tstddev error: %.3f milliseconds"
                % (
                    1e3
                    * sqrt((self.sum_var - self.sum_err**2 / self.n) / (self.n - 1))
                )
            )
            print(
                "\tpercent of time sleeping: %.1f %%"
                % (self.sleep_t_agg / self.time() * 100.0)
            )

    @property
    def fade(self):
        return self.killer.get_fade()

    def run(self, function_in_loop, dt=None):
        if dt is None:
            dt = self.dt
        self.t0 = self.t1 = time.time() + dt
        while not self.killer.kill_now:
            ret = function_in_loop()
            if ret == 0:
                self.stop()
            while time.time() < self.t1 and not self.killer.kill_now:
                if signal.sigtimedwait(
                    [signal.SIGTERM, signal.SIGINT, signal.SIGHUP], 0
                ):
                    self.stop()
            self.t1 += dt

    def stop(self):
        self.killer.kill_now = True

    def time(self):
        return time.time() - self.t0

    def time_since(self):
        return time.time() - self.t1

    def __iter__(self):
        self.t0 = self.t1 = time.time() + self.dt
        return self

    def __next__(self) -> tuple[int, float]:
        if self.killer.kill_now:
            raise StopIteration

        while (
            time.time() < self.t1 - 2 * PRECISION_OF_SLEEP and not self.killer.kill_now
        ):
            t_pre_sleep = time.time()
            time.sleep(
                max(PRECISION_OF_SLEEP, self.t1 - time.time() - PRECISION_OF_SLEEP)
            )
            self.sleep_t_agg += time.time() - t_pre_sleep

        while time.time() < self.t1 and not self.killer.kill_now:
            try:
                if signal.sigtimedwait(
                    [signal.SIGTERM, signal.SIGINT, signal.SIGHUP], 0
                ):
                    self.stop()
            except AttributeError:
                pass

        if self.killer.kill_now:
            raise StopIteration
        self.t1 += self.dt
        if self.ttarg is None:
            # inits ttarg on first call
            self.ttarg = time.time() + self.dt
            # then skips the first loop
            return (int(self.n), (self.t1 - self.t0))
        error = time.time() - self.ttarg  # seconds
        self.sum_err += error
        self.sum_var += error**2
        self.n += 1
        self.ttarg += self.dt
        return (int(self.n), (self.t1 - self.t0))


class CSVLog:
    """
    Logging class to make writing to a CSV file easier.
    See if __name__ == "__main__" for an example.
    At instantiation, pass a list of lists corresponding to the variable names you wish to log, as well as the name of their containers.
    The container name is prepended in the log so you know which object the variable came from.
    These variables should live as attributes within some object accessible to the main loop.
    To update the log, simply call log.update((obj1, obj2, ...)).

    Author: Kevin Best
    https://github.com/tkevinbest
    """

    def __init__(self, file_name, variable_name, container_names) -> None:
        """
        Args:
            file_name (_type_): _description_
            variable_name (_type_): _description_
            container_names (_type_): _description_
        """
        self.file_name = file_name
        self.var_names = variable_name
        column_headers = [
            cont_name + "_" + item
            for (sublist, cont_name) in zip(variable_name, container_names)
            for item in sublist
        ]
        self._write_row(column_headers)

    def update(self, data_containers_in):
        row = []
        for (var_group, container) in zip(self.var_names, data_containers_in):
            if type(container) is dict:
                for var in var_group:
                    row.append(container.get(var))
            else:
                for var in var_group:
                    row.append(getattr(container, var))
        self._write_row(row)

    def _write_row(self, val_list):
        with open(self.file_name, "a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(val_list)


class EdgeDetector:
    """
    Used to calculate rising and falling edges of a digital signal in real time.
    Call edgeDetector.update(digitalSignal) to update the detector.
    Then read edgeDetector.rising_edge or falling edge to know if the event occurred.

    Author: Kevin Best
    https://github.com/tkevinbest
    """

    def __init__(self, bool_in):
        self.cur_state = bool_in
        self.rising_edge = False
        self.falling_edge = False

    def update(self, bool_in):
        self.rising_edge = bool_in and not self.cur_state
        self.falling_edge = not bool_in and self.cur_state
        self.cur_state = bool_in


class SaturatingRamp:
    """
    Creates a signal that ramps between 0 and 1 at the specified rate.
    Looks like a trapezoid in the time domain
    Used to slowly enable joint torque for smooth switching at startup.
    Call saturatingRamp.update() to update the value of the ramp and return the value.
    Can also access saturatingRamp.value without updating.

    Example usage:
        ramp = saturatingRamp(100, 1.0)

        # In loop
            torque = torque * ramp.update(enable_ramp)

    Author: Kevin Best
    https://github.com/tkevinbest
    """

    def __init__(self, loop_frequency=100, ramp_time=1.0) -> None:
        """
        Args:
            loop_frequency (int, optional): Rate in Hz (default 100 Hz). Defaults to 100.
            ramp_time (float, optional): Time to complete the ramp. Defaults to 1.0.
        """
        self.delta_per_update = 1.0 / (loop_frequency * ramp_time)
        self.value = 0.0

    def update(self, enable_ramp=False):
        """
        Updates the ramp value and returns it as a float.
        If enable_ramp is true, ramp value increases
        Otherwise decreases.

        Example usage:
            torque = torque * ramp.update(enable_ramp)

        Args:
            enable_ramp (bool, optional): If enable_ramp is true, ramp value increases. Defaults to False.

        Returns:
            value (float): Scalar between 0 and 1.
        """
        if enable_ramp:
            delta = self.delta_per_update
        else:
            delta = -1 * self.delta_per_update
        self.value += delta

        self.value = min(max(self.value, 0), 1)
        return self.value


def get_active_ports():
    """
    Lists active serial ports.
    """
    if sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        ports = glob.glob("/dev/tty[A-Za-z]C*")
    elif sys.platform.startswith("darwin"):
        ports = glob.glob("/dev/tty.*")
    elif sys.platform.startswith("win"):
        ports = ["COM%s" % (i + 1) for i in range(256)]
    else:
        raise OSError("Unsupported platform.")

    serial_ports = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            serial_ports.append(port)
        except (OSError, serial.SerialException):
            pass

    return serial_ports


def from_twos_compliment(value: int, bit_length: int) -> int:
    """Converts a 2's compliment integer to a signed integer

    Args:
        value (int): 2's compliment integer
        bit_length (int): Number of bits of 2's compliment representation

    Returns:
        int: Signed integer


    """
    assert type(value) == int
    assert value >= 0
    assert type(bit_length) == int
    assert bit_length >= 0
    assert value.bit_length() <= bit_length
    if value >= 2 ** (bit_length - 1):
        return int(value - (2**bit_length))
    else:
        return int(value)


def to_twos_compliment(value: int, bit_length: int) -> int:
    """Converts a signed integer to 2's compliment for of a defined number of bits
    as an unsigned integer

    Args:
        value (int): Signed integer to convert
        bits (int): Number of bits of 2's compliment representation

    Returns:
        int: Unsigned integer 2's compliment
    """
    assert value >= -(
        2 ** (bit_length - 1)
    ), f"Value {value} is too small for {bit_length} bits"
    assert value < 2 ** (
        bit_length - 1
    ), f"Value {value} is too large for {bit_length} bits"
    if value >= 0:
        return value
    return value + 2**bit_length


def value_to_bit_count(
    value: float, bit_length: int, min_value: float, max_value: float
):
    """_summary_

    Args:
        value (float): _description_
        bit_length (_type_): _description_
        min_value (_type_): _description_
        max_value (_type_): _description_

    Returns:
        _type_: _description_
    """
    assert type(bit_length) == int
    assert bit_length > 0
    assert min_value < max_value
    assert min_value <= value <= max_value

    if value < 0:
        new_value = (max_value - min_value) + value
    else:
        new_value = value

    return int(new_value / (max_value - min_value) * (2**bit_length - 1))


def nested_dict_update(d: dict, u: dict) -> dict:
    """Updated values in a nested dictionary

    Does not add new keys or remove old keys.
    Preserves the structure of the original dictionary.

    Args:
        d (dict): Original dictionary
        u (dict): Update dictionary

    Returns:
        dict: A reference to the original dictionary with updated values
    """

    def get_operation(d: dict, k: str, v: Any) -> Callable[[], None]:
        def operation():
            d[k] = v

        return operation

    def nested_update(d: dict, u: dict) -> list[Callable[[], None]]:
        operations = []
        for k, v in u.items():
            if k not in d.keys():
                raise KeyError(f"Key {k} not in dictionary")
            elif isinstance(v, dict):
                operations.extend(nested_update(d.get(k), v))
            else:
                operations.append(get_operation(d, k, v))
        return operations

    for op in nested_update(d, u):
        op()
    return d
