import serial
import time

class SerialControl:
    def __init__(self, port):
        self.port = port
        self.serial = None

    def open_serial(self):
        try:
            self.serial = serial.Serial(self.port, 9600, timeout=1)
            print(f"Port is available")
        except Exception as e:
            print(f"Failed to open port: {str(e)}")

    def close_serial(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    def send_command(self, command):
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(command.encode() + b'\n')
                time.sleep(0.1)  # Espera breve para permitir que el Arduino procese el comando
                self.read_response()
            except Exception as e:
                print(f"Error sending command: {str(e)}")

    def read_response(self):
        if self.serial and self.serial.is_open:
            while self.serial.in_waiting > 0:
                response = self.serial.readline().decode().strip()
                print(f"Arduino Response: {response}")

    def home(self):
        self.send_command("HOME")
        while True:
            response = self.read_response()
            if response == "HOMING_COMPLETE":
                break
            time.sleep(0.1)  # Espera breve antes de revisar de nuevoef home(self):
            self.send_command("HOME")

    def send_angles(self, q1, q2, q3):
        command = f"MOVE {q1},{q2},{q3}\n"
        self.send_command(command)
        while(self.read_response() != "Arduino Response: Movimiento terminado"):
            time.sleep(0.1)
