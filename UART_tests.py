# -*- coding: utf-8 -*-
"""MPSSE.py: A python wrapper for the FTDI-provided libMPSSE DLL (I2C only)

__author__ = "Jason M. Battle"
__copyright__ = "Copyright 2016, Jason M. Battle"
__license__ = "MIT"
__email__ = "jason.battle@gmail.com"
"""

from UART import UARTMaster

if __name__ == '__main__':
    
    test = UARTMaster() 
    numchan = test.CreateDeviceInfoList()
    chaninfo = test.GetDeviceInfoList() # Channel index starts at 0
    handle = test.Open(0)
    test.SetBaudRate(handle)
    test.SetDataCharacteristics(handle)
    test.SetFlowControl(handle)
    test.Purge(handle)
    test.Write(handle, '01,TEMP,S40.0\r\n')
#    test.Read(handle, 2)
    test.SetDtr(handle)
    test.ClrDtr(handle)
    test.SetRts(handle)
    test.ClrRts(handle)
    test.SetBreakOn(handle)
    test.SetBreakOff(handle)
    test.GetQueueStatus(handle)
    test.GetStatus(handle)
#   test.GetModemStatus(handle)
    test.ResetDevice(handle)
    test.ResetPort(handle)
    test.CyclePort(handle)
    test.Close(handle)