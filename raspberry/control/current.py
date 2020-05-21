#!/usr/bin/env python
import time
import INA3221

ina3221 = INA3221.INA3221(addr=0x40)

# ch1: cutter
# ch2: wheels
# ch3: 5V BEC RPi and uC

def measure():
    # same bus voltage for all channels
    bus_voltage = ina3221.getBusVoltage_V(1)
    shunt_mV = ina3221.getShuntVoltage_mV(1)
    battery_voltage = bus_voltage + 0.001 * shunt_mV

    # current in amps
    i1 = 0.001 * ina3221.getCurrent_mA(1) 
    i2 = 0.001 * ina3221.getCurrent_mA(2)
    i3 = 0.001 * ina3221.getCurrent_mA(3)

    return i1, i2, i3, battery_voltage



def main2():
    while True:
        i1, i2, i3, v_bat = measure()
        print(f"{v_bat:3.2f} V  {i1:3.2f} mA  {i2:3.2f} mA  {i3:3.2f} mA")
        time.sleep(1.0)


def main():
    while True:
        print("------------------------------")
        for channel in range(1, 4):
            busvoltage = ina3221.getBusVoltage_V(channel)
            shuntvoltage = ina3221.getShuntVoltage_mV(channel)
            current_mA = ina3221.getCurrent_mA(channel)  
            loadvoltage = busvoltage + (shuntvoltage / 1000)    
            print(f"{channel}   {busvoltage:3.2f} V   {loadvoltage:3.2f} V  {shuntvoltage:3.2f} mV {current_mA:3.2f} mA")        
        print()
        # print(ina3221.read())
        # print()

        time.sleep(1.0)


if __name__ == "__main__":
    main2()