import numpy as np
import os 

#Define absolute max value per axis
X = Y = 60
Z = [-200,30]

#Speed
F_max = 4000
F_min = 100

#Length
lines = 300

#DELTA8SERAND
print("DELTA SERENDIPITY GEN !!!")
# Ask for the filename
filename = os.path.join('gcodes',input("Enter the name of the file: "))

try:
    # Open the file in write mode
	with open(filename, 'w') as file:
		file.write("G0 X0 Y0 Z100 \n")
		for _ in range(lines):
			Xl = np.random.randint(-X, X + 1)
			Yl = np.random.randint(-Y, Y)
			Zl = np.random.randint(Z[0],Z[1])
			Fl = np.random.randint(F_min,F_max)

			line = f"G1 X{Xl} Y{Yl} Z{Zl} F{Fl}\n"
			print(line)

			file.write(line)


except FileNotFoundError:
    print(f"The file {filename} could not be found.")



