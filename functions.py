import machine
from machine import Pin, PWM
import utime
import functions

LED1 = machine.Pin(0, Pin.OUT)
LED2 = Pin(15, Pin.OUT)
buzzer = PWM(Pin(22))

# # bed1 = Pin(1, Pin.IN, Pin.PULL_DOWN)
# # bed2 = Pin(26, Pin.IN, Pin.PULL_DOWN)
# # bed3 = Pin(17, Pin.IN, Pin.PULL_DOWN)
# # bed4 = Pin(18, Pin.IN, Pin.PULL_DOWN)
# # bth1 = Pin(14, Pin.IN, Pin.PULL_DOWN)
# # off_all = Pin(4, Pin.IN, Pin.PULL_DOWN)
# 
bed1_btn = machine.Pin(1,machine.Pin.IN,machine.Pin.PULL_DOWN)
bed1_prev_state = bed1_btn.value()
off_btn = Pin(4,Pin.IN,Pin.PULL_DOWN)
off_prev_state = off_btn.value()
 
def bed1_handler():
    global bed1_prev_state
    if (bed1_btn.value() == True) and (bed1_prev_state == False):
        bed1_prev_state = True

    elif (bed1_btn.value() == False) and (bed1_prev_state == True):
        bed1_prev_state = False
        LED1.value(1)
        buzzer.freq(300)
        buzzer.duty_u16(60000)
        utime.sleep_ms(400)
        print("Bed 1 has been pressed")

def off_handler():
    global off_prev_state
    if (off_btn.value() == True) and (off_prev_state == False):
        off_prev_state = True
        
    elif (off_btn.value() == False) and (off_prev_state == True):
        off_prev_state = False
        LED1.value(0)
        buzzer.duty_u16(0)
        utime.sleep_ms(400)
        print("Bed 1 has been answered")
        
print("Running")
