import numpy as np
import serial
import os 
from time import sleep

#Define absolute max value per axis
X = Y = 60
Z = [-200,30]

#Speed
F_max = 4000
F_min = 100

'''init connection'''
s = serial.Serial('/dev/ttyACM0', 115200)
s.write(b"\r\n\r\n") 
sleep(3)
s.flushInput()  # Flush startup text in serial input

'''send homing command'''
s.write(b"G28\n".format(seq))
grbl_out = s.read_until(b'ok\n')

'''gcode line processing'''
try : 
	Xl = np.random.randint(-X, X + 1)
	Yl = np.random.randint(-Y, Y)
	Zl = np.random.randint(Z[0],Z[1])
	Fl = np.random.randint(F_min,F_max)
	s.write(b"G1 X{Xl} Y{Yl} Z{Zl} F{Fl}\n")

	grbl_out = s.read_until(b'ok\n')
	#grbl_out = s.readline() # Wait for response with carriage return
	print(grbl_out.decode('utf_8'))

except Error as e :
	print(e)
