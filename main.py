import machine
from machine import Pin, PWM
import utime
import functions
from functions import (bed1_down_handler, off_btn_handler)


while True:
    
    bed1_down_handler()
    utime.sleep_ms(400)
    off_btn_handler()
    utime.sleep_ms(400)
