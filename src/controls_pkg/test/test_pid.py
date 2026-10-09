import math

from controls_pkg.pid import PID


def test_p_only_is_gain_times_error():
    pid = PID(kp=2.0)
    assert pid.update(0.1, 0.05) == 0.2
    assert pid.update(-0.3, 0.05) == -0.6


def test_first_step_has_no_derivative_kick():
    pid = PID(kp=1.0, kd=5.0)
    assert pid.update(0.4, 0.05) == 0.4      # only the P term
    assert pid.d_term == 0.0


def test_unfiltered_derivative_is_slope():
    pid = PID(kd=1.0)
    pid.update(0.0, 0.1)
    assert math.isclose(pid.update(0.2, 0.1), 2.0)      # 0.2 / 0.1


def test_derivative_filter_smooths_a_step():
    raw, filt = PID(kd=1.0), PID(kd=1.0, d_filter_tau=0.3)
    for pid in (raw, filt):
        pid.update(0.0, 0.1)
    assert abs(filt.update(1.0, 0.1)) < abs(raw.update(1.0, 0.1))
    # dt / (tau + dt) = 0.25 of the raw slope of 10
    assert math.isclose(filt.d_term, 2.5)


def test_measured_rate_replaces_the_difference():
    pid = PID(kd=2.0)
    assert math.isclose(pid.update(0.0, 0.1, rate=0.3), 0.6)   # used on the first call too
    assert math.isclose(pid.update(5.0, 0.1, rate=-0.1), -0.2)  # the jump in e is ignored


def test_integral_is_sum_of_error_times_dt():
    pid = PID(ki=1.0)
    for _ in range(5):
        out = pid.update(0.1, 0.1)
    assert math.isclose(out, 0.05)
    assert math.isclose(pid.integral, 0.05)


def test_non_positive_dt_changes_nothing():
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    pid.update(0.1, 0.1)
    before = (pid.integral, pid.derivative)
    pid.update(0.5, 0.0)
    pid.update(0.5, -0.1)
    assert (pid.integral, pid.derivative) == before


def test_output_is_limited():
    pid = PID(kp=10.0, out_limit=0.5)
    assert pid.update(1.0, 0.1) == 0.5
    assert pid.update(-1.0, 0.1) == -0.5
    assert pid.saturated


def test_anti_windup_freezes_integral_while_saturated():
    on = PID(kp=1.0, ki=1.0, out_limit=0.5)
    off = PID(kp=1.0, ki=1.0, out_limit=0.5, anti_windup=False)
    for _ in range(100):
        on.update(2.0, 0.1)
        off.update(2.0, 0.1)
    assert on.integral == 0.0
    assert math.isclose(off.integral, 20.0)
    # the error changes sign: the wound-up integral keeps the output pinned
    assert on.update(-0.1, 0.1) < 0.0
    assert off.update(-0.1, 0.1) == 0.5


def test_integral_still_unwinds_when_saturated():
    pid = PID(kp=0.0, ki=1.0, out_limit=0.5)
    pid.integral = 1.0              # output pinned at the 0.5 limit
    pid.update(-1.0, 0.1)           # error pulls back toward the limit
    assert math.isclose(pid.integral, 0.9)


def test_reset_clears_history():
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    pid.update(0.3, 0.1)
    pid.update(0.5, 0.1)
    pid.reset()
    assert pid.integral == 0.0 and pid.derivative == 0.0
    assert math.isclose(pid.update(0.2, 0.1), 0.22)   # P + first integral slice
