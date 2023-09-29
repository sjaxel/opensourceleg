from typing import Callable, ClassVar, Dict, Optional

import time
from dataclasses import dataclass, field


class TimerError(Exception):
    """A custom exception used to report errors in use of Timer class"""


@dataclass
class OSLTimer:
    timers: ClassVar[dict[str, dict[str, float]]] = {}
    name: Optional[str] = None
    text: str = "Elapsed time: {:0.4f} seconds"
    logger: Optional[Callable[[str], None]] = print
    _start_time: Optional[float] = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        """Add timer to dict of timers after initialization"""
        if self.name is not None:
            self.timers.setdefault(self.name, {"time": 0.0, "max": 0, "count": 0})

    def __enter__(self):
        """Start a new timer as a context manager"""
        self.start()
        return self

    def __exit__(self, *exc_info):
        """Stop the context manager timer"""
        self.stop()

    def start(self) -> None:
        """Start a new timer"""
        if self._start_time is not None:
            raise TimerError(f"Timer is running. Use .stop() to stop it")

        self._start_time = time.perf_counter()

    def stop(self) -> float:
        """Stop the timer, and report the elapsed time"""
        if self._start_time is None:
            raise TimerError(f"Timer is not running. Use .start() to start it")

        # Calculate elapsed time
        elapsed_time = time.perf_counter() - self._start_time
        self._start_time = None

        # Report elapsed time
        if self.logger:
            self.logger(self.text.format(elapsed_time))
        if self.name:
            self.timers[self.name]["time"] += elapsed_time
            if elapsed_time > self.timers[self.name]["max"]:
                self.timers[self.name]["max"] = elapsed_time
            self.timers[self.name]["count"] += 1

        return elapsed_time

    def __str__(self) -> str:
        stats = self.timers[self.name]
        n = stats["count"]
        if n == 0:
            return f"Timer {self.name}: No data"
        return f"Timer {self.name}: avg={(stats['time'] / stats['count']):.4f}s, max {stats['max']:.4f}s"


if __name__ == "__main__":
    t = OSLTimer(name="test")
    print(t.timers)

    with t:
        time.sleep(0.7)
    print(t.timers)

    with t:
        time.sleep(0.7)
    print(t.timers)
