"""lowstate_to_joint_states: joint mapping, robot-clock stamping, rate limiting."""
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu

unitree_hg = pytest.importorskip("unitree_hg.msg")
from g1_sensors.lowstate_to_joint_states import LowStateToJointStates  # noqa: E402

NAMES = [f"j{i}" for i in range(29)]
S = 1_000_000_000


class Capture:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class FakeClock:
    def __init__(self, ns):
        self.ns = ns

    def now(self):
        return rclpy.time.Time(nanoseconds=self.ns)


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def build(ros, **params):
    """Construct the node with parameters, a fake clock, and a captured publisher."""
    import g1_sensors.lowstate_to_joint_states as mod
    orig = rclpy.node.Node.__init__

    values = {"joint_names": NAMES, **params}

    def init(self, name, **kw):
        kw["parameter_overrides"] = [Parameter(k, value=v) for k, v in values.items()]
        orig(self, name, **kw)
    mod.Node.__init__ = init
    try:
        n = LowStateToJointStates()
    finally:
        mod.Node.__init__ = orig
    n.clock = FakeClock(1000 * S)
    n.get_clock = lambda: n.clock
    n.pub = Capture()
    return n


def lowstate(q0=0.0):
    """Raw CDR bytes, as the node receives them (raw subscription)."""
    msg = unitree_hg.LowState()
    for i in range(29):
        msg.motor_state[i].q = q0 + i * 0.01
        msg.motor_state[i].dq = -i * 0.1
        msg.motor_state[i].tau_est = i * 1.0
    return serialize_message(msg)


def imu(stamp_ns):
    m = Imu()
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp_ns, S)
    return serialize_message(m)


def test_waits_for_robot_clock_then_stamps_on_it(ros):
    n = build(ros, publish_rate=0.0)
    n.on_lowstate(lowstate())
    assert n.pub.msgs == []  # no clock reference yet
    offset = -73_300_000_000  # laptop 73.3 s ahead of the robot
    for k in range(5):
        n.clock.ns += 10_000_000
        n.on_reference(imu(n.clock.ns + offset + (k % 2) * 1_000_000))  # 1 ms jitter
    n.on_lowstate(lowstate())
    (js,) = n.pub.msgs
    stamp = js.header.stamp.sec * S + js.header.stamp.nanosec
    assert abs(stamp - (n.clock.ns + offset)) <= 1_000_000
    assert js.name == NAMES
    assert js.position[28] == pytest.approx(0.28)
    assert js.velocity[3] == pytest.approx(-0.3) and js.effort[5] == pytest.approx(5.0)
    n.destroy_node()


def test_receive_stamp_source(ros):
    n = build(ros, stamp_source="receive", publish_rate=0.0)
    n.on_lowstate(lowstate())
    (js,) = n.pub.msgs
    assert js.header.stamp.sec * S + js.header.stamp.nanosec == n.clock.ns
    n.destroy_node()


def test_rate_limit_and_time_reset(ros):
    n = build(ros, stamp_source="receive", publish_rate=50.0)
    for _ in range(100):  # 1 kHz for 0.1 s -> 5 messages at 50 Hz
        n.on_lowstate(lowstate())
        n.clock.ns += 1_000_000
    assert len(n.pub.msgs) == 5
    n.clock.ns -= 10 * S  # bag restarted: time jumped back, must publish again
    n.on_lowstate(lowstate())
    assert len(n.pub.msgs) == 6
    n.destroy_node()


def test_offset_window_drops_old_samples_and_resets_on_time_jump(ros):
    n = build(ros, publish_rate=0.0, clock_window=2.0)
    n.on_reference(imu(n.clock.ns - 5 * S))
    n.clock.ns += 3 * S
    n.on_reference(imu(n.clock.ns + 1 * S))
    assert [o for _, o in n.offsets] == [1 * S]
    n.clock.ns -= 100 * S
    n.on_reference(imu(n.clock.ns))
    assert [o for _, o in n.offsets] == [0]
    n.destroy_node()


@pytest.mark.parametrize("params", [{"stamp_source": "gps"}, {"joint_names": [""]}])
def test_rejects_bad_parameters(ros, params):
    with pytest.raises(RuntimeError):
        build(ros, **params)
