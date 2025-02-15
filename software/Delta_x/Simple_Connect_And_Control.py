import serial
import time

ser = serial.Serial('/dev/ttyACM0',115200, timeout = 1)  # open serial port
time.sleep(2)    
print(ser.readline())

gcodes = []
gcodes.append('G28')
gcodes.append('G01 Z-320')
gcodes.append('G01 X-100')
gcodes.append('G01 Z-350')

gcodes.append('G01 Z-320')
gcodes.append('G01 X100')
gcodes.append('G01 Z-350')

gcodes.append('G01 Z-320')
gcodes.append('G01 X100 Y100')
gcodes.append('G01 Z-350')

gcodes.append('G01 X-100 Y-100 Z-320')
gcodes.append('G01 Z-350')
gcodes.append('G28')

for gcode in gcodes:
    print(gcode)
    data = gcode + '\r\n'
    ser.write( data.encode() )
    while 1:
        response = ser.readline()
        print(response)

ser.close()             # close port