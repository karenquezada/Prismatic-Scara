import serial
from serial import Serial
import time

class SerialControl:

    def __init__(self, port="/dev/ttyUSB0"):
        self.port = port
        self.serial = None
        self.serial_port = ""

    def open_serial(self):
        try:
            self.serial = Serial(self.port, 9600, timeout=1, write_timeout=0.2)
            print("The port is available")
            self.serial_port = "Open"
            time.sleep(2)
        except serial.serialutil.SerialException:
            print("The port is at use")
            self.serial.close()
            self.serial.open()


    def close_serial(self):
        time.sleep(0.2)
        self.serial.close()
        self.serial_port = "Close"

    def envio_pos(self, diametro=0, x_pos=0, n_dientes=0):
        mensaje = ",{},{},{}\n".format(diametro,x_pos,n_dientes)
        print(mensaje)
        self.serial.write(mensaje.encode())
        print(self.serial.readline())


   
    def stop(self):
        self.serial.write('STOP\n'.encode())
   