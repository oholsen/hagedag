#!/usr/bin/env python
import time
import INA3221

ina3221 = INA3221.INA3221(addr=0x40)


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
    main()