import machine
from machine import Pin, PWM
import utime
import functions
from functions import (bed1_handler, off_btn_handler)


while True:
    
    bed1_handler()
    utime.sleep_ms(400)
    off_btn_handler()
    utime.sleep_ms(400)
