# pyUART
Yet another Python wrapper for the [ftd2xx] library, intended for use with UART-capable FTDI USB-Serial Bridge ICs.

**NOTE:** [ftd2xx] works in tandem with the D2xx driver. Please ensure you've installed the appropriate package and copied the dll to a known location (as specified in the script) prior to exploring pyUART.  

**NOTE:** Tested with Python 2.7 on Windows 7. 

## Setup
The script currently looks for ftd2xx.dll in `C:\Python27\Lib\site-packages\`. 
The location must be updated if residing elsewhere on your system.   

## References

[ftd2xx Programmer's Guide]

[ftd2xx Sample Code]

[FT232R Datasheet]

[ftd2xx]: http://www.ftdichip.com/Drivers/D2XX.htm
[ftd2xx Programmer's Guide]:  http://www.ftdichip.com/Support/Documents/ProgramGuides/D2XX_Programmer's_Guide(FT_000071).pdf
[ftd2xx Sample Code]: http://www.ftdichip.com/Support/SoftwareExamples/CodeExamples.htm
[FT232R Datasheet]: http://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT232R.pdf
