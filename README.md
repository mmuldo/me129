# drEEm tEEm's eebot

## `eebot` command

Note that this script is in the PI's `$PATH` so it can be run from any directory.

### Usage

```bash
$ eebot SUBCOMMAND ARG0 ARG1 ... [--time TIME]
```

Note that TIME is in seconds.

### `led-status`

prints status of each LED detector (colors indicate the jumper cable color connected to LED)

#### Usage

```bash
$ eebot led-status

orange 0 # this one is off
brown 1 # this one is on
white 0
```

### `set-pwm`

sets the PWM (between -1.0 and 1.0) of each wheel

#### Usage

```bash
$ eebot set-pwm LEFT_PWM RIGHT_PWM --time TIME
```

#### Examples

```bash
$ eebot set-pwm 0.6 -0.7 --time 1.2
```

### `set`

sets nominal velocity (m/s) and steering angle (degrees) of robot

#### Usage

```bash
$ eebot set VEL_NOM STEERING_ANGLE --time TIME
```

#### Examples

```bash
$ eebot set 0.2 -30 --time 0.7
```
