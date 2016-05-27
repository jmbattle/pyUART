# -*- coding: utf-8 -*-
"""UART.py: A python wrapper for the FTDI-provided ftd2xx DLL (UART Only) 

__author__ = "Jason M. Battle"
__copyright__ = "Copyright 2016, Jason M. Battle"
__license__ = "MIT"
__email__ = "jason.battle@gmail.com"

"""

import ctypes

dll_loc = r'C:\Python27\Lib\site-packages\ftd2xx.dll'

try:
    dll = ctypes.WinDLL(dll_loc)
except:
    print '%s not found' % dll_loc.split('packages\\')[-1]
        
#####GLOBALS###################################################################

# UART OpenEx Flags
UART_OPEN_BY_SERIAL = 1
UART_OPEN_BY_DESCRIPTION = 2
UART_OPEN_BY_LOCATION = 4

# UART ListDevices Flags
UART_LIST_NUMBER_ONLY = 0x80000000
UART_LIST_BY_INDEX = 0x40000000
UART_LIST_ALL = 0x20000000
UART_LIST_MASK = UART_LIST_NUMBER_ONLY | UART_LIST_BY_INDEX | UART_LIST_ALL

# UART Baud Rate Options
UART_BAUD_600 = 600
UART_BAUD_1200 = 1200
UART_BAUD_2400 = 2400
UART_BAUD_4800 = 4800
UART_BAUD_9600 = 9600
UART_BAUD_14400 = 14400
UART_BAUD_19200 = 19200
UART_BAUD_38400 = 38400
UART_BAUD_57600 = 57600
UART_BAUD_115200 = 115200
UART_BAUD_230400 = 230400
UART_BAUD_460800 = 460800
UART_BAUD_921600 = 921600

# UART Word Length Options
UART_BITS_8 = 8
UART_BITS_7 = 7

# UART Stop Bit Options
UART_STOP_BITS_1 = 0
UART_STOP_BITS_2 = 2

# UART Parity Bit Options
UART_PARITY_NONE = 0
UART_PARITY_ODD = 1
UART_PARITY_EVEN = 2
UART_PARITY_MARK = 3
UART_PARITY_SPACE = 4

# UART Flow Control Options
UART_FLOW_NONE = 0x0000
UART_FLOW_RTS_CTS = 0x0100
UART_FLOW_DTR_DSR = 0x0200
UART_FLOW_XON_XOFF = 0x0400

# UART Buffer Purge Options
UART_PURGE_RX = 1
UART_PURGE_TX = 2

# UART Timeout Options
UART_DEFAULT_RX_TIMEOUT = 300
UART_DEFAULT_TX_TIMEOUT = 300

# UART Event Options
UART_EVENT_RXCHAR = 1
UART_EVENT_MODEM_STATUS = 2
UART_EVENT_LINE_STATUS = 4

# Status Codes
STATUS_CODES = {0: 'FT_OK',
          1: 'FT_INVALID_HANDLE',
          2: 'FT_DEVICE_NOT_FOUND',
          3: 'FT_DEVICE_NOT_OPENED',
          4: 'FT_INVALID_HANDLE',
          5: 'FT_IO_ERROR',
          6: 'FT_INVALID_PARAMETER',
          7: 'FT_INVALID_BAUD_RATE',
          8: 'FT_DEVICE_NOT_OPENED_FOR_ERASE',
          9: 'FT_DEVICE_NOT_OPENED_FOR_WRITE',
          10: 'FT_FAILED_TO_WRITE_DEVICE',
          11: 'FT_EEPROM_READ_FAILED',
          12: 'FT_EEPROM_WRITE_FAILED',
          13: 'FT_EEPROM_ERASE_FAILED',
          14: 'FT_EEPROM_NOT_PRESENT',
          15: 'FT_EEPROM_NOT_PROGRAMMED',
          16: 'FT_INVALID_ARGS',
          17: 'FT_NOT_SUPPORTED',
          18: 'FT_OTHER_ERROR',
          19: 'FT_DEVICE_LIST_NOT_READY'}

# Device Types
DEVICE_TYPES = {0: 'FT_DEVICE_BM',
          1: 'FT_DEVICE_BM',
          2: 'FT_DEVICE_100AX',
          3: 'FT_DEVICE_UNKNOWN',
          4: 'FT_DEVICE_2232C',
          5: 'FT_DEVICE_232R',
          6: 'FT_DEVICE_2232H',
          7: 'FT_DEVICE_4232H',
          8: 'FT_DEVICE_232H',
          9: 'FT_DEVICE_X_SERIES'}
          
# Line Status
LINE_STATUS = {2: 'OVERRUN_ERROR',
          4: 'PARITY_ERROR',
          8: 'FRAMING_ERROR',
          16: 'BREAK_INTERRUPT'}
          
# Modem Status
MODEM_STATUS = {16: 'CLEAR_TO_SEND',
          32: 'DATA_SET_READY',
          64: 'RING_INDICATOR',
          128: 'DATA_CARRIER_DETECT'}
          
#####STRUCTS###################################################################
          
class FT_DEVICE_LIST_INFO_NODE(ctypes.Structure):
    _fields_ = [
        ('Flags', ctypes.c_ulong),        
        ('Type', ctypes.c_ulong),
        ('ID', ctypes.c_ulong),
        ('LocID', ctypes.c_ulong),
        ('SerialNumber', ctypes.c_ubyte*16),
        ('Description', ctypes.c_ubyte*64),
        ('ftHandle', ctypes.c_ulong)]
          
#####CLASSES###################################################################

class FlagError(Exception):
    pass

class OptionsError(Exception):
    pass

class UARTMaster():
       
    def __init__(self):
        pass

# FT_CreateDeviceInfoList(LPDWORD lpdwNumDevs)

    def CreateDeviceInfoList(self):
        dll.FT_CreateDeviceInfoList.argtypes = [ctypes.POINTER(ctypes.c_ulong)]
        dll.FT_CreateDeviceInfoList.restypes = ctypes.c_ulong
        self._numdevices = ctypes.c_ulong()
        if dll.FT_CreateDeviceInfoList(ctypes.byref(self._numdevices)) != 0:
            print STATUS_CODES[dll.FT_CreateDeviceInfoList(ctypes.byref(self._numdevices))]
        else:
            print 'Number of Devices: %i' % self._numdevices.value
            return self._numdevices.value      

# FT_GetDeviceInfoList(FT_DEVICE_LIST_INFO_NODE *pDest, LPDWORD lpdwNumDevs)

    def GetDeviceInfoList(self):
        dll.FT_GetDeviceInfoList.argtypes = [ctypes.POINTER(FT_DEVICE_LIST_INFO_NODE), ctypes.POINTER(ctypes.c_ulong)]
        dll.FT_GetDeviceInfoList.restypes = ctypes.c_ulong
        self._chaninfo = FT_DEVICE_LIST_INFO_NODE()
        self._numdevices = ctypes.c_ulong()
        if dll.FT_GetDeviceInfoList(ctypes.byref(self._chaninfo), ctypes.byref(self._numdevices)) != 0:
            print STATUS_CODES[dll.FT_GetDeviceInfoList(ctypes.byref(self._chaninfo), ctypes.byref(self._numdevices))]
        else:
            self._Type = DEVICE_TYPES[self._chaninfo.Type]
            self._SerialNumber = ''.join(map(chr, self._chaninfo.SerialNumber)).split('\x00')[0]  # Remove non-ASCII characters
            self._Description = ''.join(map(chr, self._chaninfo.Description)).split('\x00')[0] # Remove non-ASCII characters
            print 'Flags: %i' % self._chaninfo.Flags 
            print 'Type: %s' % self._Type
            print 'ID: %i' % self._chaninfo.ID
            print 'LocID: %i' % self._chaninfo.LocID
            print 'SerialNumber: %s' % self._SerialNumber
            print 'Description: %s' % self._Description
            print 'Handle: %i' % self._chaninfo.ftHandle
            print 'Number of Devices: %i' % self._numdevices.value
            return (self._chaninfo.Flags, self._Type, self._chaninfo.ID, self._chaninfo.LocID, self._SerialNumber, self._Description, self._chaninfo.ftHandle, self._numdevices.value)
            
# FT_Open(int deviceNumber, FT_HANDLE *pHandle)

    def Open(self, index):
        dll.FT_Open.argtypes = [ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong)]
        dll.FT_Open.restypes = ctypes.c_ulong
        self._index = ctypes.c_ulong(index)
        self._handle = ctypes.c_ulong()
        if dll.FT_Open(self._index, ctypes.byref(self._handle)) != 0:
            print STATUS_CODES[dll.FT_Open(self._index, ctypes.byref(self._handle))]
        else:
            print 'Successfully opened device channel %i with handle %i' % (self._index.value, self._handle.value)
            return self._handle.value

# FT_OpenEx(PVOID pArg1, DWORD Flags, FT_HANDLE *pHandle)

    def OpenEx(self, arg, flags='bylocation'):
        dll.FT_OpenEx.restypes = ctypes.c_ulong 
        self._handle = ctypes.c_ulong()         
        if flags == 'byserial':
            dll.FT_OpenEx.argtypes = [ctypes.POINTER(ctypes.c_ubyte*16), ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong)]           
            self._arg = (ctypes.c_ubyte*16)(map(ord, arg))
            self._flags = ctypes.c_ulong(UART_OPEN_BY_SERIAL)
            if dll.FT_OpenEx(ctypes.byref(self._arg1), self._flags, ctypes.byref(self._handle)) != 0:
                print STATUS_CODES[dll.FT_OpenEx(ctypes.byref(self._arg1), self._flags, ctypes.byref(self._handle))]
            else:
                print 'Successfully opened handle %i with serial %s' % (self._handle.value, ''.join(map(chr, self._arg1[:])))
                return self._handle.value
        elif flags == 'bydescription':
            dll.FT_OpenEx.argtypes = [ctypes.POINTER(ctypes.c_ubyte*64), ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong)]
            self._arg = (ctypes.c_ubyte*64)(map(ord, arg))            
            self._flags = ctypes.c_ulong(UART_OPEN_BY_DESCRIPTION)
            if dll.FT_OpenEx(ctypes.byref(self._arg1), self._flags, ctypes.byref(self._handle)) != 0:
                print STATUS_CODES[dll.FT_OpenEx(ctypes.byref(self._arg1), self._flags, ctypes.byref(self._handle))]
            else:
                print 'Successfully opened handle %i with description %s' % (self._handle.value, ''.join(map(chr, self._arg1[:])))
                return self._handle.value            
        elif flags == 'bylocation':
            dll.FT_OpenEx.argtypes = [ctypes.POINTER(ctypes.c_ulong), ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong)]
            self._arg = ctypes.c_ulong(arg)            
            self._flags = ctypes.c_ulong(UART_OPEN_BY_LOCATION)
            if dll.FT_OpenEx(ctypes.byref(self._arg), self._flags, ctypes.byref(self._handle)) != 0:
                print STATUS_CODES[dll.FT_OpenEx(ctypes.byref(self._arg), self._flags, ctypes.byref(self._handle))]
            else:
                print 'Successfully opened handle %i with location %i' % (self._handle.value, self._arg)
                return self._handle.value            
        else:
            raise FlagError('Unsupported string passed for flag parameter')        

# FT_Close(	FT_HANDLE ftHandle)

    def Close(self, handle):
        dll.FT_Close.argtypes = [ctypes.c_ulong]
        dll.FT_Close.restypes = ctypes.c_ulong
        self._handle = ctypes.c_ulong(handle)
        if dll.FT_Close(self._handle) != 0:
            print STATUS_CODES[dll.FT_Close(self._handle)]
        else:
            print 'Successfully closed device channel %i with handle %i' % (self._index.value, self._handle.value)

# FT_Read(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD dwBytesToRead, LPDWORD lpBytesReturned)

    def Read(self, handle, numbytes):
        dll.FT_Read.argtypes = [ctypes.c_ulong, ctypes.POINTER(ctypes.c_ubyte*numbytes), ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong)]
        dll.FT_Read.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        self._buffer = (ctypes.c_ubyte*numbytes)()
        self._readbytes = ctypes.c_ulong(numbytes)
        self._numsent = ctypes.c_ulong()
        if dll.FT_Read(self._handle, ctypes.byref(self._buffer), self._readbytes, ctypes.byref(self._numsent)) != 0:
            print STATUS_CODES[dll.FT_Read(self._handle, ctypes.byref(self._buffer), self._readbytes, ctypes.byref(self._numsent))]
        else:
            print 'UART read transaction complete'
            for idx, byte in enumerate(self._buffer):
                print 'Data Byte %i: 0x%02X' % (idx+1, byte)
            print 'Data Length: %i' % self._numsent.value
            return self._buffer[:]

# FT_Write(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD dwBytesToWrite, LPDWORD lpBytesWritten)

    def Write(self, handle, data):
        data = map(ord, data)
        dll.FT_Write.argtypes = [ctypes.c_ulong, ctypes.POINTER(ctypes.c_ubyte*len(data)), ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong)]
        dll.FT_Write.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        self._buffer = (ctypes.c_ubyte*len(data))(*data)
        self._writebytes = ctypes.c_ulong(len(data))
        self._numsent = ctypes.c_ulong()
        if dll.FT_Write(self._handle, ctypes.byref(self._buffer), self._writebytes, ctypes.byref(self._numsent)) != 0:
            print STATUS_CODES[dll.FT_Write(self._handle, ctypes.byref(self._buffer), self._writebytes, ctypes.byref(self._numsent))]
        else:
            print 'UART write transaction complete'
            for idx, byte in enumerate(self._buffer):
                print 'Data Byte %i: 0x%02X' % (idx+1, byte)
            print 'Data Length: %i' % self._numsent.value
  
# FT_SetBaudRate(FT_HANDLE ftHandle, ULONG BaudRate)

    def SetBaudRate(self, handle, options='standard'):
        dll.FT_SetBaudRate.argtypes = [ctypes.c_ulong, ctypes.c_ulong]
        dll.FT_SetBaudRate.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        if options == 'standard':
            self._baudrate = ctypes.c_ulong(UART_BAUD_19200)
            if dll.FT_SetBaudRate(self._handle, self._baudrate) != 0:
                print STATUS_CODES[dll.FT_SetBaudRate(self._handle, self._baudrate)]
            else:
                print 'Successfully updated baudrate for device channel %i with device handle %i' % (self._index.value, self._handle.value)
                print 'Baud Rate: %i' % (self._baudrate.value)
        else:
            raise OptionsError('Unsupported or no option specified. Baud rate left unchanged')

# FT_SetDataCharacteristics(FT_HANDLE ftHandle, UCHAR WordLength, UCHAR StopBits, UCHAR Parity)

    def SetDataCharacteristics(self, handle, options='standard'):
        dll.FT_SetDataCharacteristics.argtypes = [ctypes.c_ulong, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_ubyte]
        dll.FT_SetDataCharacteristics.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        if options == 'standard':
            self._wordlength = ctypes.c_ubyte(UART_BITS_8)
            self._stopbits = ctypes.c_ubyte(UART_STOP_BITS_1)
            self._parity = ctypes.c_ubyte(UART_PARITY_NONE)
            if dll.FT_SetDataCharacteristics(self._handle, self._wordlength, self._stopbits, self._parity) != 0:
                print STATUS_CODES[dll.FT_SetDataCharacteristics(self._handle, self._wordlength, self._stopbits, self._parity)]
            else:
                print 'Successfully updated data characteristics for device channel %i with device handle %i' % (self._index.value, self._handle.value)
                print 'Word Length: %i' % (self._wordlength.value)
                print 'Stop Bits: %i' % (self._stopbits.value)
                print 'Parity: %i' % (self._parity.value)
        else:
            raise OptionsError('Unsupported or no option specified. Data characteristics left unchanged')

# FT_SetFlowControl(FT_HANDLE ftHandle, USHORT FlowControl, UCHAR XonChar, UCHAR XoffChar)

    def SetFlowControl(self, handle, options='standard'):
        dll.FT_SetFlowControl.argtypes = [ctypes.c_ulong, ctypes.c_ushort, ctypes.c_ubyte, ctypes.c_ubyte]
        dll.FT_SetFlowControl.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        if options == 'standard':
            self._flowcontrol = ctypes.c_ushort(UART_FLOW_NONE)
            self._xonchar = ctypes.c_ubyte(0x11)
            self._xoffchar = ctypes.c_ubyte(0x13)
            if dll.FT_SetFlowControl(self._handle, self._flowcontrol, self._xonchar, self._xoffchar) != 0:
                print STATUS_CODES[dll.FT_SetFlowControl(self._handle, self._flowcontrol, self._xonchar, self._xoffchar)]
            else:
                print 'Successfully updated flow control for device channel %i with device handle %i' % (self._index.value, self._handle.value)
                print 'Flow Control: %i' % (self._flowcontrol.value)
                print 'XON Character: 0x%02X' % (self._xonchar.value)
                print 'XOFF Character: 0x%02X' % (self._xoffchar.value)
        else:
            raise OptionsError('Unsupported or no option specified. Flow control left unchanged')

# FT_SetDtr(FT_HANDLE ftHandle)

    def SetDtr(self, handle):
        dll.FT_SetDtr.argtypes = [ctypes.c_ulong]
        dll.FT_SetDtr.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        if dll.FT_SetDtr(self._handle) != 0:
            print STATUS_CODES[dll.FT_SetDtr(self._handle)]
        else:
            print 'DTR line is asserted'

# FT_ClrDtr(FT_HANDLE ftHandle)

    def ClrDtr(self, handle):
        dll.FT_ClrDtr.argtypes = [ctypes.c_ulong]
        dll.FT_ClrDtr.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        if dll.FT_ClrDtr(self._handle) != 0:
            print STATUS_CODES[dll.FT_ClrDtr(self._handle)]
        else:
            print 'DTR line is de-asserted'

# FT_SetRts(FT_HANDLE ftHandle)

    def SetRts(self, handle):
        dll.FT_SetRts.argtypes = [ctypes.c_ulong]
        dll.FT_SetRts.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        if dll.FT_SetRts(self._handle) != 0:
            print STATUS_CODES[dll.FT_SetRts(self._handle)]
        else:
            print 'RTS line is asserted'

# FT_ClrRts(FT_HANDLE ftHandle)

    def ClrRts(self, handle):
        dll.FT_ClrRts.argtypes = [ctypes.c_ulong]
        dll.FT_ClrRts.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        if dll.FT_ClrRts(self._handle) != 0:
            print STATUS_CODES[dll.FT_ClrRts(self._handle)]
        else:
            print 'RTS line is de-asserted'

# FT_GetModemStatus(FT_HANDLE ftHandle, ULONG *pModemStatus)

    def GetModemStatus(self, handle):
        dll.FT_GetModemStatus.argtypes = [ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong)]
        dll.FT_GetModemStatus.restypes = ctypes.c_ulong
        self._handle = ctypes.c_ulong(handle)
        self._modemstatus = ctypes.c_ulong()
        if dll.FT_GetModemStatus(self._handle, ctypes.byref(self._modemstatus)) != 0:
            print STATUS_CODES[dll.FT_GetModemStatus(self._handle, ctypes.byref(self._modemstatus))]
        else:
            self._linestatus = LINE_STATUS[(self._modemstatus.value >> 8) & 255]
            self._modemstatus = MODEM_STATUS[self._modemstatus.value & 255]
            print 'Line Status: %s' % (self._linestatus)
            print 'Modem Status: %s' % (self._modemstatus)
            return (self._linestatus, self._modemstatus)

# FT_GetQueueStatus(FT_HANDLE ftHandle, DWORD *dwRxBytes)

    def GetQueueStatus(self, handle):
        dll.FT_GetModemStatus.argtypes = [ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong)]
        dll.FT_GetModemStatus.restypes = ctypes.c_ulong
        self._handle = ctypes.c_ulong(handle)
        self._queuestatus = ctypes.c_ulong()
        if dll.FT_GetQueueStatus(self._handle, ctypes.byref(self._queuestatus)) != 0:
            print STATUS_CODES[dll.FT_GetQueueStatus(self._handle, ctypes.byref(self._queuestatus))]
        else:
            print 'Bytes Left in Receive Queue: %i' % (self._queuestatus.value)
            return self._queuestatus.value

# FT_GetStatus(FT_HANDLE ftHandle, DWORD *dwRxBytes, DWORD *dwTxBytes, DWORD *dwEventDWord)

    def GetStatus(self, handle):
        dll.FT_GetStatus.argtypes = [ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong)]
        dll.FT_GetStatus.restypes = ctypes.c_ulong
        self._handle = ctypes.c_ulong(handle)
        self._rxqueue = ctypes.c_ulong()
        self._txqueue = ctypes.c_ulong()
        self._eventstatus = ctypes.c_ulong()
        if dll.FT_GetStatus(self._handle, ctypes.byref(self._rxqueue), ctypes.byref(self._txqueue), ctypes.byref(self._eventstatus)) != 0:
            print STATUS_CODES[dll.FT_GetStatus(self._handle, ctypes.byref(self._rxqueue), ctypes.byref(self._txqueue), ctypes.byref(self._eventstatus))]
        else:
            print 'Bytes Left in Receive Queue: %i' % (self._rxqueue.value)
            print 'Bytes Left in Transmit Queue: %i' % (self._txqueue.value)
            print 'Event Status: %i' % (self._eventstatus.value)
            return (self._rxqueue.value, self._txqueue.value, self._eventstatus.value)

# FT_SetEventNotification(FT_HANDLE ftHandle, DWORD Mask, PVOID Param)

    def SetEventNotification(self, handle, options='line_status'):
        dll.FT_SetEventNotification.argtypes = [ctypes.c_ulong, ctypes.c_ulong, ctypes.POINTER(None)]
        dll.FT_SetEventNotification.restypes = ctypes.c_ulong
        self._handle = ctypes.c_ulong(handle)
        self._eventhandle = ctypes.c_void_p
        if options == 'rxchar':
            self._eventmask = ctypes.c_ulong(UART_EVENT_RXCHAR)
            if dll.FT_SetEventNotification(self._handle, self._eventmask, self._eventhandle) != 0:
                print STATUS_CODES[dll.FT_SetEventNotification(self._handle, self._eventmask, self._eventhandle)]
            else:
                print 'Event notifications set for new receive characters'
        elif options == 'modem_status':
            self._eventmask = ctypes.c_ulong(UART_EVENT_MODEM_STATUS)
            if dll.FT_SetEventNotification(self._handle, self._eventmask, self._eventhandle) != 0:
                print STATUS_CODES[dll.FT_SetEventNotification(self._handle, self._eventmask, self._eventhandle)]
            else:
                print 'Event notifications set for modem status changes'            
        elif options == 'line_status':
            self._eventmask = ctypes.c_ulong(UART_EVENT_LINE_STATUS)
            if dll.FT_SetEventNotification(self._handle, self._eventmask, self._eventhandle) != 0:
                print STATUS_CODES[dll.FT_SetEventNotification(self._handle, self._eventmask, self._eventhandle)]
            else:
                print 'Event notifications set for line status changes'            
        else:
            raise OptionsError('Unsupported or no option specified. Event settings left unchanged')

# FT_SetChars(FT_HANDLE ftHandle, UCHAR EventChar, UCHAR EventCharEnabled, UCHAR ErrorChar, UCHAR ErrorCharEnabled)

    def SetChars(self, handle, eventchar=0x82, eventcharenable=0, errorchar=0x88, errorcharenable=0):
        dll.FT_SetChars.argtypes = [ctypes.c_ulong, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_ubyte]
        dll.FT_SetChars.restypes = ctypes.c_ulong
        self._handle = ctypes.c_ulong(handle)
        self._eventchar = ctypes.c_ubyte(eventchar)
        self._eventcharenable = ctypes.c_ubyte(eventcharenable)
        self._errorchar = ctypes.c_ubyte(errorchar)
        self._errorcharenable = ctypes.c_ubyte(errorcharenable)
        if dll.FT_SetChars(self._handle, self._eventchar, self._eventcharenable, self._errorchar, self._errorcharenable) != 0:
            print STATUS_CODES[dll.FT_SetChars(self._handle, self._eventchar, self._eventcharenable, self._errorchar, self._errorcharenable)]
        else:
            print 'Configured event and error characters were successfully updated '
            print 'Event Character: 0x%02X' % (self._eventchar)
            print 'Error Character: 0x%02X' % (self._errorchar)

# FT_SetBreakOn(FT_HANDLE ftHandle)

    def SetBreakOn(self, handle):
        dll.FT_SetBreakOn.argtypes = [ctypes.c_ulong]
        dll.FT_SetBreakOn.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        if dll.FT_SetBreakOn(self._handle) != 0:
            print STATUS_CODES[dll.FT_SetBreakOn(self._handle)]
        else:
            print 'BREAK condition set'

#FT_SetBreakOff(FT_HANDLE ftHandle)

    def SetBreakOff(self, handle):
        dll.FT_SetBreakOff.argtypes = [ctypes.c_ulong]
        dll.FT_SetBreakOff.restypes = ctypes.c_ulong        
        self._handle = ctypes.c_ulong(handle)
        if dll.FT_SetBreakOff(self._handle) != 0:
            print STATUS_CODES[dll.FT_SetBreakOff(self._handle)]
        else:
            print 'BREAK condition cleared'

# FT_Purge(FT_HANDLE ftHandle, ULONG Mask)

    def Purge(self, handle, option='both'):
        dll.FT_Purge.argtypes = [ctypes.c_ulong, ctypes.c_ulong]
        dll.FT_Purge.restypes = ctypes.c_ulong
        self._handle = ctypes.c_ulong(handle)
        if option == 'tx':
            self._purgemask = ctypes.c_ulong(UART_PURGE_TX)
            if dll.FT_Purge(self._handle, self._purgemask) != 0:
                print STATUS_CODES[dll.FT_Purge(self._handle, self._purgemask)]
            else:
                print 'TX buffer cleared'
        elif option == 'rx':
            self._purgemask = ctypes.c_ulong(UART_PURGE_RX)
            if dll.FT_Purge(self._handle, self._purgemask) != 0:
                print STATUS_CODES[dll.FT_Purge(self._handle, self._purgemask)]
            else:
                print 'RX buffer cleared'
        elif option == 'both':            
            self._purgemask = ctypes.c_ulong(UART_PURGE_TX | UART_PURGE_RX)
            if dll.FT_Purge(self._handle, self._purgemask) != 0:
                print STATUS_CODES[dll.FT_Purge(self._handle, self._purgemask)]
            else:
                print 'TX and RX buffers cleared'
        else:
            raise OptionsError('Unsupported or no option specified. Buffers left unchanged')

# FT_ResetDevice(	FT_HANDLE ftHandle)

    def ResetDevice(self, handle):
        dll.FT_ResetDevice.argtypes = [ctypes.c_ulong]
        dll.FT_ResetDevice.restypes = ctypes.c_ulong
        self._handle = ctypes.c_ulong(handle)
        if dll.FT_ResetDevice(self._handle) != 0:
            print STATUS_CODES[dll.FT_ResetDevice(self._handle)]
        else:
            print 'Successfully resetted device channel %i with handle %i' % (self._index.value, self._handle.value)

# FT_ResetPort(FT_HANDLE ftHandle)

    def ResetPort(self, handle):
        dll.FT_ResetPort.argtypes = [ctypes.c_ulong]
        dll.FT_ResetPort.restypes = ctypes.c_ulong
        self._handle = ctypes.c_ulong(handle)
        if dll.FT_ResetPort(self._handle) != 0:
            print STATUS_CODES[dll.FT_ResetPort(self._handle)]
        else:
            print 'Successfully resetted port of device channel %i with handle %i' % (self._index.value, self._handle.value)
  
# FT_CyclePort(FT_HANDLE ftHandle)

    def CyclePort(self, handle):
        dll.FT_CyclePort.argtypes = [ctypes.c_ulong]
        dll.FT_CyclePort.restypes = ctypes.c_ulong
        self._handle = ctypes.c_ulong(handle)
        if dll.FT_CyclePort(self._handle) != 0:
            print STATUS_CODES[dll.FT_CyclePort(self._handle)]
        else:
            print 'Successfully power-cycled device channel %i with handle %i' % (self._index.value, self._handle.value)
 