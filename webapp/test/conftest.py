"""Shared test helpers. These tests need no ROS install, and neither does preflight.py."""
import sys
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parent.parent
MILESTONE1 = PACKAGE.parent / 'src' / 'milestone1_singlevehicle'
for path in (PACKAGE, MILESTONE1):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


@pytest.fixture
def clock():
    return FakeClock()
