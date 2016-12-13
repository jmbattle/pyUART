# -*- coding: utf-8 -*-
"""UART_tests.py: Simple test routine for pyUART wrapper functions

__author__ = "Jason M. Battle"
"""

from UART import UARTMaster

if __name__ == '__main__':

    test = UARTMaster()
    test.CreateDeviceInfoList()
    test.GetDeviceInfoList()
    test.Open()
    test.SetBaudRate()
    test.SetDataCharacteristics()
    test.SetFlowControl()
    test.SetTimeouts()
    test.SetEventNotification()
    test.SetBreakOn()
    test.SetBreakOff()
    test.SetRts()
    test.ClrRts()
    test.SetDtr()
    test.ClrDtr()
    test.GetModemStatus()
    test.GetQueueStatus()
    test.GetStatus()
    test.Purge()
    test.Write('')
    test.Read()
    test.ResetDevice()
    test.ResetPort() 
    test.Close()
    
