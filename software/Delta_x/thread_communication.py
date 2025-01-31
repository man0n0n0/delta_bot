import serial
import threading
import time

class GRBLController:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.serial_port = serial.Serial(port, baudrate)
        self.is_running = True
        self.lock = threading.Lock()

    def send_command(self, command):
        with self.lock:
            self.serial_port.write(f"{command}\n".encode())
            response = self.serial_port.readline().decode().strip()
            return response

    def listen(self):
        while self.is_running:
            try:
                response = self.serial_port.readline().decode().strip()
                print(f"GRBL: {response}")
            except Exception as e:
                print(f"Error in listen thread: {e}")

    def run(self):
        listener_thread = threading.Thread(target=self.listen)
        listener_thread.start()

        while self.is_running:
            command = input("Enter GRBL command (or 'quit' to stop): ")
            if command.lower() == 'quit':
                self.is_running = False
                print("Stopping GRBL controller...")
                break
            try:
                response = self.send_command(command)
                print(f"Response: {response}")
            except Exception as e:
                print(f"Error sending command: {e}")

        self.serial_port.close()

if __name__ == "__main__":
    controller = GRBLController()
    controller.run()
