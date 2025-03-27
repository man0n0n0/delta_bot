from printrun.printcore import printcore as PrintCore
from printrun import gcoder
from geometry import Point
import time

class PrintDriver:
    def __init__(self, usbserial, baudrate=115200):
        self.printcore = PrintCore(usbserial, baudrate)
        self.highest_z = 10

        self.waitUntilOnline()
        self.goHomeSafely()
        
    def goHomeSafely(self):
        # go to home without interference to the labware and switch to absolute coordinate mode
        gcode_list = [f'G0 Z{self.highest_z + 10} F1500;','G28', 'G90']
        print(gcode_list)
        gcode = gcoder.LightGCode(gcode_list)
        self.printcore.startprint(gcode)
        self.current_position = Point()

    def waitUntilOnline(self):
        # wait until p._listen_until_online() set p online
        while not self.printcore.online:
            print(self.printcore.online)
            print(self.printcore.sent)
            time.sleep(1)
            
    def updateCurrentPosition(self, point):
        self.current_position = point

    def executeGcode(self, gcode_list):
        for command in gcode_list:
            self.printcore.send(command)
        print(self.printcore.mainqueue)

    def moveTipToPoint(self, kargs):
        command = "G0"

        for axis in ["X", "Y", "Z"]:
            if axis in kargs.keys():
                command += f' {axis}{kargs[axis]}'
            else:
                raise ValueError(f"No {axis} entry")

        command += " F1500"

        gcode_list = [command]
        self.executeGcode(gcode_list)
        self.updateCurrentPosition(Point(kargs["X"],kargs["Y"],kargs["Z"]))

