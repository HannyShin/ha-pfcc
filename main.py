import machine
from machine import Pin, PWM
import utime
import functions
from functions import (bed1_handler, off_handler)


while True:
    # one wire to GPIO & the other to 3.3OUT
    # bed1 uses blue & blue/white wire
    bed1_handler()
    # off uses brown & brown/white wire
    off_handler()
