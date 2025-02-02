import numpy as np

#Define absolute max value per axis
X = Y = 60
Z = 30

#Speed
F_max = 4000
F_min = 500

#Length
lines = 15

#DELTA8SERAND
print("DELTA SERENDIPITY GEN !!!")
# Ask for the filename
filename = input("Enter the name of the file: ")

try:
    # Open the file in write mode
	with open(filename, 'w') as file:
		file.write("G0 X0 Y0 Z100 \n")
		file.write("G91 \n")
		for _ in range(lines):
			Xl = np.random.randint(-X, X + 1)
			Yl = np.random.randint(-Y, Y)
			Zl = np.random.randint(-Z, Z)
			Fl = np.random.randint(F_min,F_max)

			line = f"G1 X{Xl} Y{Yl} Z{Zl} \n"
			print(line)

			file.write(line)


except FileNotFoundError:
    print(f"The file {filename} could not be found.")



