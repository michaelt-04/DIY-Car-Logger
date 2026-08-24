from machine import ADC
import time

adc = ADC(4)

while True:
    volts = adc.read_u16() * 3.3 / 65535
    print("%.1f C" % (27 - (volts - 0.706) / 0.001721))
    time.sleep(1)