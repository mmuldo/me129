# drEEm tEEm's eebot

Welcome to the drEEm tEEm's ME129 project, in which 3 electrical engineers
(Hope Arnett, Dylan Perlson, and Matt Muldowney) attempt to infultrate the
MechE department and take it over from the inside >:)

## Documentation

All docs can be found at the [wiki](https://github.com/mmuldo/me129/wiki).

## Quick Start

`move_bot.py` is our driver, and can be run simply with:

```bash
$ python3 move_bot.py
```

at the project root. Inside `move_bot.py` is a function called `driver` in
which main code can be placed:

```python
def driver():
    '''driver function (put main code here)'''
    control = robot.EEBot(pigpio.pi())
    try:
        # stuff goes here
        # e.g.
        control.find(2,1)
    except BaseException as ex:
        print('uh oh...theres an exception')
        control.ultra.shutdown_ultrasonic()
        control.shutdown()
        print('heres the traceback:')
        raise ex
```
