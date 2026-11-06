"""One `ros2 launch` child process: start it, read its output, stop it without blocking.

Stopping is driven from poll() so the agent's loop never waits on it, because an
abort has to keep talking to the hub while a launch shuts down.
"""
import collections
import os
import signal
import subprocess
import threading
import time

# Long enough for launch to pass SIGINT on and let its nodes exit.
INTERRUPT_GRACE_S = 10.0
TERMINATE_GRACE_S = 5.0
OUTPUT_LINES = 2000


class LaunchProcess:
    def __init__(self, argv, now=time.monotonic):
        self.argv = argv
        self.now = now
        self.process = None
        self.lines = collections.deque(maxlen=OUTPUT_LINES)
        self.stop_requested_at = None
        self.terminate_sent = False
        self.kill_sent = False

    def start(self):
        # Its own session, so a signal to the agent doesn't reach the launch
        # directly and stop() can signal everything it started.
        self.process = subprocess.Popen(
            self.argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, errors='replace', bufsize=1, start_new_session=True)
        threading.Thread(target=self._read_output, daemon=True).start()

    def _read_output(self):
        for line in self.process.stdout:
            self.lines.append(line)

    def running(self):
        return self.process is not None and self.process.poll() is None

    @property
    def returncode(self):
        return None if self.process is None else self.process.poll()

    def stop(self):
        """Ask the launch to stop. Safe to call more than once."""
        if self.running() and self.stop_requested_at is None:
            self.stop_requested_at = self.now()
            # The whole group, as Ctrl-C in a terminal would. Launch assumes its children got
            # that SIGINT too, so it forwards nothing and waits 5 s before escalating. Signalling
            # only launch left the flight node running for those 5 s, still able to arm.
            self._signal_group(signal.SIGINT)

    def poll(self):
        """Escalate a stop that isn't finishing. Call this regularly."""
        if self.stop_requested_at is None or not self.running():
            return
        waited = self.now() - self.stop_requested_at
        if waited > INTERRUPT_GRACE_S and not self.terminate_sent:
            self.terminate_sent = True
            self._signal_group(signal.SIGTERM)
        elif waited > INTERRUPT_GRACE_S + TERMINATE_GRACE_S and not self.kill_sent:
            self.kill_sent = True
            self._signal_group(signal.SIGKILL)

    def drain_lines(self):
        lines = []
        while self.lines:
            lines.append(self.lines.popleft())
        return lines

    def _signal_group(self, signal_number):
        try:
            os.killpg(self.process.pid, signal_number)
        except ProcessLookupError:
            pass
