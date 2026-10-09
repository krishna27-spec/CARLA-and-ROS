"""Discrete PID controller (plain Python, no ROS).

For an error e sampled every dt seconds the output is

    u = kp * e  +  ki * I  +  kd * D

    I += e * dt                                  (integral: running sum of e dt)
    D += dt / (tau + dt) * (e_dot - D)           (filtered derivative)

where e_dot is either the difference (e - e_prev) / dt or, when the caller has
one, a direct measurement of the error rate (`rate` argument of `update`).

Three things separate a PID that works on a real actuator from the textbook
one, and each can be switched off here to see what it does:

* Derivative filter (`d_filter_tau`). A raw difference (e - e_prev) / dt
  divides sensor noise by a small dt, so the D term mostly amplifies noise.
  A first-order low-pass with time constant tau keeps the slow trend and
  drops the jitter. tau = 0 gives the raw difference.

* Measured rate (`rate`). Better than filtering a bad derivative is not
  differencing at all: if another sensor gives the rate of change of the error
  (a gyro for an angle; for cross-track error, speed * sin(heading error)),
  pass it in and the D term uses it instead of the difference.

* Anti-windup (`anti_windup`). The actuator saturates at `out_limit`. While it
  is saturated the output cannot grow, but a plain integral keeps growing and
  must be "unwound" later, which shows up as a large overshoot. Here the
  integral is frozen whenever the output is saturated and the error would
  push it further into saturation (conditional integration / clamping).

P, PD and PI are this same class with the unused gains set to zero.
"""


class PID:

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, d_filter_tau=0.0,
                 out_limit=None, anti_windup=True):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.d_filter_tau = d_filter_tau
        self.out_limit = out_limit
        self.anti_windup = anti_windup
        self.reset()

    def reset(self):
        """Forget all history (call when the reference changes abruptly)."""
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = None
        self.p_term = self.i_term = self.d_term = 0.0
        self.saturated = False

    def _limit(self, u):
        if self.out_limit is None:
            return u
        return max(-self.out_limit, min(self.out_limit, u))

    def update(self, error, dt, rate=None):
        """Return the control output for this sample.

        `rate` is an optional direct measurement of d(error)/dt; without it
        the derivative is the difference of successive errors. In that case
        the first call has no previous error, so its derivative is zero
        (starting from e_prev = 0 would give a spike of e / dt). A sample with
        dt <= 0 (duplicate or out-of-order timestamp) leaves the integral and
        derivative as they are.
        """
        integral = self.integral
        if dt > 0.0:
            if rate is None and self.prev_error is not None:
                rate = (error - self.prev_error) / dt
            if rate is not None:
                alpha = dt / (self.d_filter_tau + dt)
                self.derivative += alpha * (rate - self.derivative)
            integral += error * dt
        self.prev_error = error

        self.p_term = self.kp * error
        self.d_term = self.kd * self.derivative
        u = self.p_term + self.ki * integral + self.d_term
        out = self._limit(u)
        self.saturated = out != u
        if self.saturated and self.anti_windup and u * error > 0.0:
            # saturated and the error pushes further out: do not integrate
            integral = self.integral
            out = self._limit(self.p_term + self.ki * integral + self.d_term)
        self.integral = integral
        self.i_term = self.ki * integral
        return out
