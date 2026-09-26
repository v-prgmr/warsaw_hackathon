"""Registration recording (record_dir) and offline replay (replay_registration)."""
import json
import os

import pytest
from test_server import TAG, camera_info, run, session, tag_registration

from g1_ar_bridge import replay_registration
from g1_ar_bridge.server import BridgeConfig
from g1_ar_bridge.world import SimWorld

pytest.importorskip("websockets")


def record(tmp_path):
    world = SimWorld()
    cfg = BridgeConfig(tag_black_size_m=TAG, record_dir=str(tmp_path))

    async def t(lens, _bridge):
        await camera_info(lens)
        return await tag_registration(lens, world)

    statuses = run(session(t, world, cfg))
    assert statuses[-1]["state"] == "succeeded"
    # the Lens overlay text stays visible until the commit: progress < 80 before it
    assert all(s["progress"] < 80 for s in statuses[:-1])
    (folder,) = [os.path.join(tmp_path, d) for d in os.listdir(tmp_path)]
    return folder, len(statuses)


def test_registration_is_recorded(tmp_path):
    folder, n = record(tmp_path)
    frames = [json.loads(line) for line in open(os.path.join(folder, "frames.jsonl"))]
    assert len(frames) == n
    assert all(os.path.getsize(os.path.join(folder, f["file"])) > 1000 for f in frames)
    assert frames[0]["anchor_T_map_tag"] is not None and frames[0]["width"] == 1008
    session_info = json.load(open(os.path.join(folder, "session.json")))
    assert session_info["config"]["tag_black_size_m"] == TAG
    events = [json.loads(line)["event"] for line in open(os.path.join(folder, "events.jsonl"))]
    assert events[-1] == "registered" and "diagnosis" in events


def test_replay_reproduces_the_outcome_and_explains_a_stricter_one(tmp_path, capsys):
    folder, _ = record(tmp_path)
    replay_registration.main([str(tmp_path), "--overlays", str(tmp_path / "ov")])
    out = capsys.readouterr().out
    assert "RESULT: registers" in out
    assert len(os.listdir(tmp_path / "ov")) > 0

    replay_registration.main([folder, "--min-views", "50"])
    out = capsys.readouterr().out
    assert "RESULT: does NOT register" in out and "consistent views" in out
