"""Tests for the launch child process, using small Python scripts in place of ros2 launch."""
import sys
import time

from swarm_launcher import launch_process
from swarm_launcher.launch_process import LaunchProcess


def script(tmp_path, body):
    path = tmp_path / 'child.py'
    path.write_text(body)
    return [sys.executable, '-u', str(path)]


def wait_until(condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.02)
    return False


def test_output_is_collected(tmp_path):
    process = LaunchProcess(script(tmp_path, "print('hello')\nprint('there')\n"))
    process.start()
    assert wait_until(lambda: not process.running())
    time.sleep(0.05)
    assert process.drain_lines() == ['hello\n', 'there\n']
    assert process.returncode == 0


def test_exit_code_is_reported(tmp_path):
    process = LaunchProcess(script(tmp_path, 'import sys\nsys.exit(3)\n'))
    process.start()
    assert wait_until(lambda: not process.running())
    assert process.returncode == 3


def test_stop_sends_an_interrupt_so_launch_can_shut_down_cleanly(tmp_path):
    process = LaunchProcess(script(tmp_path, (
        'import signal, sys, time\n'
        "signal.signal(signal.SIGINT, lambda *a: (print('clean stop'), sys.exit(0)))\n"
        "print('ready')\n"
        'time.sleep(30)\n')))
    process.start()
    assert wait_until(lambda: process.lines)
    process.stop()
    process.stop()  # Safe to repeat.
    assert wait_until(lambda: not process.running())
    time.sleep(0.05)
    assert 'clean stop\n' in process.drain_lines()
    assert process.returncode == 0


def test_stop_reaches_what_the_launch_started(tmp_path):
    """Seen in SITL. ros2 launch doesn't forward a SIGINT, because it assumes a terminal sent
    its children one too. Signalling only launch left the flight node running for 5 s, in which
    it armed a drone that had just been aborted.
    """
    child = tmp_path / 'node.py'
    marker = tmp_path / 'stopped'
    child.write_text(
        'import signal, sys, time\n'
        f"signal.signal(signal.SIGINT, lambda *a: (open({str(marker)!r}, 'w').close(), sys.exit(0)))\n"
        "print('node up', flush=True)\n"
        'time.sleep(30)\n')
    process = LaunchProcess(script(tmp_path, (
        'import signal, subprocess, sys\n'
        'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'  # Like launch: does not forward it.
        f'node = subprocess.Popen([sys.executable, "-u", {str(child)!r}], preexec_fn=lambda: signal.signal(signal.SIGINT, signal.default_int_handler))\n'
        'node.wait()\n')))
    process.start()
    assert wait_until(lambda: process.lines)
    process.stop()
    assert wait_until(marker.exists, timeout=2.0), 'the node never got the signal'
    assert wait_until(lambda: not process.running())


def test_stop_escalates_when_a_launch_ignores_the_interrupt(tmp_path, clock):
    process = LaunchProcess(script(tmp_path, (
        'import signal, time\n'
        'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
        'signal.signal(signal.SIGTERM, signal.SIG_IGN)\n'
        "print('ready')\n"
        'time.sleep(30)\n')), now=clock)
    process.start()
    assert wait_until(lambda: process.lines)
    process.stop()
    process.poll()
    assert process.running()
    clock.advance(launch_process.INTERRUPT_GRACE_S + 1)
    process.poll()
    assert process.terminate_sent and process.running()
    clock.advance(launch_process.TERMINATE_GRACE_S + 1)
    process.poll()
    assert process.kill_sent
    assert wait_until(lambda: not process.running())


def test_stopping_a_finished_process_is_harmless(tmp_path):
    process = LaunchProcess(script(tmp_path, 'pass\n'))
    process.start()
    assert wait_until(lambda: not process.running())
    process.stop()
    process.poll()
    assert process.stop_requested_at is None
