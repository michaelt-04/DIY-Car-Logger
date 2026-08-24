from machine import Pin
import time

led = Pin("LED", Pin.OUT)

for i in range(10):
    led.toggle()
    time.sleep(0.5)