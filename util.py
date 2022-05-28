from typing import Tuple

# Cardinal directions
N = 0 #north
W = 1 #west
S = 2 #south
E = 3 #east

# Directions
F = 0   # forwards
L = 1   # left
B = 2   # backwards
R = 3   # right

# street existence
UNKNOWN = -1
ABSENT = 0
PRESENT = 1
BLOCKED = 2


diff_to_dir = {
    (0, 1): N,
    (-1, 0): W,
    (0, -1): S,
    (1, 0): E,
}

dir_to_diff = {dir: diff for diff, dir in diff_to_dir.items()}


# first element is the left motor; second is the right.
# first element of each tuple is the negative terminal; second is positive
MOTOR_PINS = [(7, 8), (5, 6)]


def duty_to_pwm(
    motor_number: int,
    dutycycle: float
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    '''
    converts dutycycle to pwm

    Parameters
    ----------
    motor_number : int
        0 -> for left motor, 1 -> for right motor; errors if anything else
    dutycycle : float
        dutycycle, between -1.0 and 1.0

    Returns
    -------
    Tuple[Tuple[int, int], Tuple[int, int]]
        two tuples: the first is the (motor_pin, pwm) for the pin
        of the motor being set and the second is (other_motor_pin, 0)
    '''
    magnitude = abs(round(dutycycle*255))
    if magnitude > 255:
        # max pwm is 255
        magnitude = 255

    # negative dutycycle -> negative pin, positive -> positive pin
    pin_to_set = 0 if dutycycle < 0 else 1
    other_pin = 0 if pin_to_set == 1 else 1

    return ((MOTOR_PINS[motor_number][pin_to_set], magnitude),
            (MOTOR_PINS[motor_number][other_pin], 0))
