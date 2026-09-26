"""Tie a node's lifetime to the process that started it (Linux).

If `ros2 launch` is killed with SIGKILL, its children are orphaned and keep running: a leftover
node went on publishing onto the robot's network during a live test (2026-09-26). With
PR_SET_PDEATHSIG the kernel sends SIGTERM to the node when its parent dies, and rclpy turns that
into a normal shutdown.
"""
import ctypes
import os
import signal

_PR_SET_PDEATHSIG = 1


def exit_with_parent():
    """Call before rclpy.init(). No-op where prctl is unavailable."""
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
    except OSError:
        return
    if libc.prctl(_PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0) != 0:
        return
    if os.getppid() == 1:  # the parent died before prctl took effect
        os.kill(os.getpid(), signal.SIGTERM)
