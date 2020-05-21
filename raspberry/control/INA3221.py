#!/usr/bin/env python
#encoding: utf-8

# Adapted from 
# SDL_Pi_INA3221.py Python Driver Code
# SwitchDoc Labs March 4, 2015	 
# V 1.2

 
import smbus2 as smbus
from struct import pack, unpack

#/*=========================================================================
#    I2C ADDRESS/BITS
#    -----------------------------------------------------------------------*/
INA3221_ADDRESS =                         (0x40)    # 1000000 (A0+A1=GND)
INA3221_READ    =                         (0x01)
#/*=========================================================================*/

#/*=========================================================================
#    CONFIG REGISTER (R/W)
#    -----------------------------------------------------------------------*/
INA3221_REG_CONFIG            =          (0x00)
#    /*---------------------------------------------------------------------*/
INA3221_CONFIG_RESET          =          (0x8000)  # Reset Bit
	
INA3221_CONFIG_ENABLE_CHAN1   =          (0x4000)  # Enable Channel 1
INA3221_CONFIG_ENABLE_CHAN2   =          (0x2000)  # Enable Channel 2
INA3221_CONFIG_ENABLE_CHAN3   =          (0x1000)  # Enable Channel 3

INA3221_CONFIG_AVG2     =                (0x0800)  # AVG Samples Bit 2 - See table 3 spec
INA3221_CONFIG_AVG1     =                (0x0400)  # AVG Samples Bit 1 - See table 3 spec
INA3221_CONFIG_AVG0     =                (0x0200)  # AVG Samples Bit 0 - See table 3 spec

INA3221_CONFIG_VBUS_CT2 =                (0x0100)  # VBUS bit 2 Conversion time - See table 4 spec
INA3221_CONFIG_VBUS_CT1 =                (0x0080)  # VBUS bit 1 Conversion time - See table 4 spec
INA3221_CONFIG_VBUS_CT0 =                (0x0040)  # VBUS bit 0 Conversion time - See table 4 spec

INA3221_CONFIG_VSH_CT2  =                (0x0020)  # Vshunt bit 2 Conversion time - See table 5 spec
INA3221_CONFIG_VSH_CT1  =                (0x0010)  # Vshunt bit 1 Conversion time - See table 5 spec
INA3221_CONFIG_VSH_CT0  =                (0x0008)  # Vshunt bit 0 Conversion time - See table 5 spec

INA3221_CONFIG_MODE_2   =                (0x0004)  # Operating Mode bit 2 - See table 6 spec
INA3221_CONFIG_MODE_1   = 	             (0x0002)  # Operating Mode bit 1 - See table 6 spec
INA3221_CONFIG_MODE_0 	=                (0x0001)  # Operating Mode bit 0 - See table 6 spec

#/*=========================================================================*/

#/*=========================================================================
#    SHUNT VOLTAGE REGISTER (R)
#    -----------------------------------------------------------------------*/
INA3221_REG_SHUNTVOLTAGE_1   =             (0x01)
#/*=========================================================================*/

#/*=========================================================================
#    BUS VOLTAGE REGISTER (R)
#    -----------------------------------------------------------------------*/
INA3221_REG_BUSVOLTAGE_1     =             (0x02)
#/*=========================================================================*/

SHUNT_RESISTOR_VALUE         = 0.1   # default shunt resistor value of 0.1 Ohm



class INA3221():


    def __init__(self, twi=1, addr=INA3221_ADDRESS, shunt_resistor = SHUNT_RESISTOR_VALUE  ):
        self._bus = smbus.SMBus(twi)
        self._addr = addr
        config = INA3221_CONFIG_ENABLE_CHAN1 |		\
                    INA3221_CONFIG_ENABLE_CHAN2 |	\
                    INA3221_CONFIG_ENABLE_CHAN3 |	\
                    INA3221_CONFIG_AVG1 |		\
                    INA3221_CONFIG_VBUS_CT2 |		\
                    INA3221_CONFIG_VSH_CT2 |		\
                    INA3221_CONFIG_MODE_2 |		\
                    INA3221_CONFIG_MODE_1 |		\
                    INA3221_CONFIG_MODE_0
        self._write_word(INA3221_REG_CONFIG, config)


    def _read_word(self, register): 	
        result = self._bus.read_word_data(self._addr,register) & 0xFFFF
        lowbyte = (result & 0xFF00)>>8 
        highbyte = (result & 0x00FF) << 8
        return lowbyte + highbyte 
   
   
    def _write_word(self, register, data):
        data = data & 0xFFFF
        # reverse configure byte for little endian
        lowbyte = (data >> 8) & 0xFF
        highbyte = (data & 0xFF) << 8
        switchdata = lowbyte + highbyte
        self._bus.write_word_data(self._addr, register, switchdata)       


    def _getBusVoltage_raw(self, channel):
    	# Gets the raw bus voltage (16-bit signed integer, so +-32767)
        value = self._read_word(INA3221_REG_BUSVOLTAGE_1 + (channel-1)*2) 
        if value > 32767:
            value -= 65536
        return value


    def _getShuntVoltage_raw(self, channel):
    	# Gets the raw shunt voltage (16-bit signed integer, so +-32767)	
        value = self._read_word(INA3221_REG_SHUNTVOLTAGE_1 + (channel-1)*2)
        if value > 32767:
            value -= 65536
        return value


    def getBusVoltage_V(self, channel):
	    # Gets the Bus voltage in volts
        # LSB at bit 3 is 8mV
        return 0.001 * self._getBusVoltage_raw(channel)


    def getShuntVoltage_mV(self, channel):
	    # Gets the shunt voltage in mV (so +-168.3mV)
        # LSB at bit 3 is 40uV
        return 0.005 * self._getShuntVoltage_raw(channel)


    def getCurrent_mA(self, channel):
        # Gets the current value in mA, taking into account the config settings and current LSB    	
        return self.getShuntVoltage_mV(channel) / SHUNT_RESISTOR_VALUE


    def read(self):
        # read all channels in one I2C transaction
        data = self._bus.read_i2c_block_data(self._addr, 1, 12)
        # array of ints...
        return data
        #print(repr(data))
        #return unpack("<hhhhhh", data)
