"""Records the glasses' AprilTag registrations for offline diagnosis (``replay_registration``).

One directory per registration attempt::

    <root>/<YYYYmmdd-HHMMSS>/
      session.json     bridge config + robot anchor (T_map_tag) at the start
      frames.jsonl     one line per processed glasses frame: header, intrinsics, result, file
      frame_NNNN.jpg   the frame as the Lens sent it
      events.jsonl     diagnosis lines, anchor changes, commit / failure / stop

About 100 KB per frame, <= 1 frame/s from the Lens. Recording never breaks the bridge: disk
errors are reported once and then ignored.
"""
import dataclasses
import json
import os
import threading
import time

import numpy as np


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


class RegistrationRecorder:
    def __init__(self, root, log=print):
        self.root = os.path.expanduser(root)
        self.log = log
        self.dir = None
        self.count = 0
        self.lock = threading.Lock()
        self.failed = False

    def _write(self, name, text, mode="a"):
        if self.dir is None or self.failed:
            return
        try:
            with open(os.path.join(self.dir, name), mode) as f:
                f.write(text)
        except OSError as exc:
            self._fail(exc)

    def _fail(self, exc):
        self.failed = True
        self.log(f"[ar_bridge] registration recording disabled: {exc}")

    def start(self, config, anchor):
        with self.lock:
            self.count = 0
            name = time.strftime("%Y%m%d-%H%M%S")
            path = os.path.join(self.root, name)
            try:
                os.makedirs(path, exist_ok=True)
            except OSError as exc:
                self.dir = None
                self._fail(exc)
                return
            self.dir, self.failed = path, False
            cfg = dataclasses.asdict(config) if dataclasses.is_dataclass(config) else dict(config)
            self._write("session.json", json.dumps(_jsonable({
                "started": time.strftime("%Y-%m-%d %H:%M:%S"), "config": cfg,
                "anchor_T_map_tag": anchor}), indent=1) + "\n", mode="w")
        self.log(f"[ar_bridge] recording this registration to {path}")

    def frame(self, header, jpeg, cam, result, anchor):
        with self.lock:
            if self.dir is None or self.failed:
                return
            self.count += 1
            name = f"frame_{self.count:04d}.jpg"
            try:
                with open(os.path.join(self.dir, name), "wb") as f:
                    f.write(jpeg)
            except OSError as exc:
                self._fail(exc)
                return
            K, width, height = cam
            self._write("frames.jsonl", json.dumps(_jsonable({
                "file": name, "t": time.time(), "header": header, "K": K, "width": width,
                "height": height, "result": result, "anchor_T_map_tag": anchor})) + "\n")

    def event(self, kind, **data):
        with self.lock:
            self._write("events.jsonl", json.dumps(_jsonable(dict(
                data, event=kind, t=time.time()))) + "\n")

    def stop(self):
        with self.lock:
            self.dir = None
