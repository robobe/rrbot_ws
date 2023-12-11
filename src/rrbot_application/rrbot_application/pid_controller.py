from cmath import inf
import time


class PID:
    """
    Implements a PID controller.
    """

    def __init__(
        self, Kp: float, Ki: float, Kd: float, target: float, tau: float = 0
    ) -> None:
        """

        Parameters
        ----------
        Kp : float
            Proportional gain.
        Ki : float
            Integration gain.
        Kd : float
            Derivative gain.
        tau : float
            Low pass filter time constant.
        target : float
            Target value.
        """

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tau = tau
        self.target = target
        self.Dterm = 0
        self.Iterm = 0
        self.last_error = 0
        self.last_time = time.time()
        self.last_feedback = 0
        self.last_output = 0
        self.set_limits(-inf, inf, -inf, inf)

    def set_limits(
        self, min: float = -inf, max: float = inf, min_int: float = -inf, max_int: float = inf
    ) -> None:
        """
        Output limits.

        Parameters
        ----------
        min : float
            Minimum output.
        max : float
            Maximum output.
        """
        self.max = max
        self.max_int = max_int
        self.min = min
        self.min_int = min_int

    def update(self, feedback: float) -> float:
        """
        Calculate the PID output value.

        Parameters
        ----------
        feedback : float
            Value to be compared to the target.

        Returns
        -------
        float
            Output of the PID controller.
        """
        error = self.target - feedback

        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time == 0:
            print("-------------------------------------------------")
            return self.last_output

        self.Pterm = self.Kp * error
        # self.Iterm += (error + self.last_error) * 0.5 * self.Ki * delta_time
        self.Iterm += error * self.Ki * delta_time
        # self.Dterm = -2 * self.Kd * (feedback - self.last_feedback) + (
        #     2 * self.tau - delta_time
        # ) * self.Dterm / (2 * self.tau + delta_time)
        self.Dterm = -self.Kd * (error - self.last_error) / delta_time

        if self.Iterm > self.max_int:
            self.Iterm = self.max_int
        elif self.Iterm < self.min_int:
            self.Iterm = self.min_int

        self.last_time = current_time
        self.last_error = error
        self.last_feedback = feedback

        print(f"P: {self.Pterm}, I: {self.Iterm}, f: {feedback}")

        output = self.Pterm + self.Iterm + self.Dterm
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output
