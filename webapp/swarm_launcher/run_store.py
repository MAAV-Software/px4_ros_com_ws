"""The files a run leaves behind on the hub: run.json, events.jsonl and log.jsonl.

A full disk must never interfere with a mission, so write errors are reported
once and then ignored.
"""
import json
import sys
import time
from pathlib import Path


class RunStore:
    def __init__(self, root):
        self.root = Path(root).expanduser()
        self.folder = None
        self.failed = False

    def begin(self, run_id, mission_id):
        self.folder = self.root / run_id
        self._write('run.json', json.dumps({
            'run_id': run_id, 'mission_id': mission_id, 'started': time.time()}), 'w')

    def event(self, **fields):
        self._append('events.jsonl', {'time': time.time(), **fields})

    def log(self, line):
        self._append('log.jsonl', {'time': time.time(), **line})

    def _append(self, name, record):
        if self.folder is not None:
            self._write(name, json.dumps(record) + '\n', 'a')

    def _write(self, name, text, mode):
        try:
            self.folder.mkdir(parents=True, exist_ok=True)
            with open(self.folder / name, mode) as handle:
                handle.write(text)
        except OSError as error:
            if not self.failed:
                print(f'run files not being saved: {error}', file=sys.stderr)
            self.failed = True
