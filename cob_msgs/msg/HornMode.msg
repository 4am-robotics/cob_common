uint8 NONE =           0           # will turn everything off
uint8 STATIC =         1           # will turn on horn permantly
uint8 FLASH =          2           # will change the Horn with "frequency" from "on" to "off"

uint8                  mode        # mode to switch on
float32                frequency   # in Hz
float32                timeout     # in s, requested mode will be executed for max timeout s.
                                   # default is 0 and means no timeout.
uint16                  pulses     # specifies the amount of pulses which will be executed.
                                   # eg: mode = flash, pulses = 2. Means the horn will honk two times
