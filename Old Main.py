import machine
from machine import Pin, PWM
from umqtt.simple import MQTTClient
import rp2
import network
import ubinascii
import urequests as requests
import time
import utime
import socket
import secrets
import gc
gc.collect()
rp2.country('US')

# wlan = network.WLAN(network.STA_IF)
# wlan.active(True)
# 
# mac = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
# print('mac = ' + mac)
# 
# ssid = secrets.ssid
# password = secrets.pw
# wlan.connect(ssid, password)
# 
# timeout = 10
# while timeout > 0:
#     if wlan.status() < 0 or wlan.status() >= 3:
#         break
#     timeout -= 1
#     print('Waiting for connection...')
#     time.sleep(1)
#     
# def blink_onboard_led(num_blinks):
#     led = machine.Pin('LED', machine.Pin.OUT)
#     for i in range(num_blinks):
#         led.on()
#         time.sleep(.2)
#         led.off()
#         time.sleep(.2)
#         
# wlan_status = wlan.status()
# blink_onboard_led(wlan_status)
# 
# if wlan_status != 3:
#     raise RuntimeError('Wi-Fi connection failed')
# else:
#     print('Connected')
#     status = wlan.ifconfig()
#     print('ip = ' + status[0])
# 
# mqtt_server = secrets.mqtt_id
# client_id = ubinascii.hexlify(machine.unique_id())
# topic_sub = b'21 Sub'
# topic_pub = b'21'
# topic_msg = b'Test Msg'
# 
# 
# def sub_cb(topic, msg):
#     print("New message on topic {}".format(topic.decode('utf-8')))
#     msg = msg.decode('utf-8')
#     print(msg)
#     if msg == '21-1 On':
#         LED1.value(1)
#     elif msg == '21-1 Off':
#         LED1.value(0)
#     elif msg == '21-2 On':
#         LED1.value(1)
#     elif msg == '21-2 Off':
#         LED1.value(0)
#     elif msg == '21-3 On':
#         LED1.value(1)
#     elif msg == '21-3 Off':
#         LED1.value(0)
#     elif msg == '21-4 On':
#         LED1.value(1)
#     elif msg == '21-4 Off':
#         LED1.value(0)
#     elif msg == 'Bthrm 21 On':
#         LED1.value(1)
#     elif msg == 'Bthrm 21 Off':
#         LED1.value(0)
#                 
# 
# def mqtt_connect():
#     client = MQTTClient(client_id, mqtt_server)
#     client.connect()
#     print ('Connected to %s MQTT Broker'%(mqtt_server))
#     return client
# 
# def reconnect():
#     print('Failed to connect to the MQTT Broker. Reconnecting...')
#     time.sleep(5)
#     machine.reset()
#     
# try:
#     client = mqtt_connect()
# except OSError as e:
#     reconnect()
#     
# client.publish(topic_pub, topic_msg)

LED1 = Pin(0, Pin.OUT)
LED2 = Pin(15, Pin.OUT)
ALL_LED = Pin(0, Pin.OUT)
buzzer = PWM(Pin(22))


bed1 = Pin(1, Pin.IN, Pin.PULL_DOWN)
bed2 = Pin(26, Pin.IN, Pin.PULL_DOWN)
bed3 = Pin(17, Pin.IN, Pin.PULL_DOWN)
bed4 = Pin(18, Pin.IN, Pin.PULL_DOWN)
bth1 = Pin(14, Pin.IN, Pin.PULL_DOWN)
off_all = Pin(4, Pin.IN, Pin.PULL_DOWN)

while True:
    
#     client.set_callback(sub_cb)
#     client.subscribe(topic_sub)
    
    if bed1.value() == 1:
        LED1.value(1)
        buzzer.freq(300)
        buzzer.duty_u16(60000)
        else: LED1.value(0)
#         if bed1.value() == 0:
#             client.publish('21-1', '21-1 On')
#             print("1") 
#             utime.sleep_ms(300)
            
    if bed2.value() == 1:
        LED1.value(1)
        buzzer.freq(300)
        buzzer.duty_u16(60000)
#         if bed2.value() == 0:
#             client.publish('21-2', '21-2 On')
#             print("2")
#             utime.sleep_ms(300)
#                
    if bed3.value() == 1:
        LED1.value(1)
        buzzer.freq(300)
        buzzer.duty_u16(60000)
#         if bed3.value() == 0:
#             client.publish('21-3', '21-3 On')
#             print("3")
#             utime.sleep_ms(300)

    if bed4.value() == 1:
        LED1.value(1)
        buzzer.freq(300)
        buzzer.duty_u16(60000)
#         if bed4.value() == 0:
#             client.publish('21-4', '21-4 On')
#             print("4")
#             utime.sleep_ms(300) 
    
    if bth1.value() == 1:
        LED2.value(1)
        buzzer.freq(300)
        buzzer.duty_u16(60000)
#         if bth1.value() == 0:
#             client.publish('Bthrm 21', 'Bthrm 21 On')
#             print("Bth")
#             utime.sleep_ms(300)
#               
    if off_all.value() == 1:
        LED1.value(0)
        LED2.value(0)
        buzzer.duty_u16(0)
#         client.publish('21-1', '21-1 Off')
#         client.publish('21-2', '21-2 Off')
#         client.publish('21-3', '21-3 Off')
#         client.publish('21-4', '21-4 Off')
#         client.publish('Bthrm 21', 'Bthrm 21 Off')    
        print ("All Off")
