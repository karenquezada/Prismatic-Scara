import serial

class SCARARobot:
    def __init__(self, port):
        self.ser = serial.Serial(port, 9600, timeout=1)
        self.ser.flush()

    def send_command(self, command):
        self.ser.write((command + "\n").encode())
        response = self.ser.readline().decode().strip()
        return response
    
    def home_robot(self):
        print("Homing robot...")
        self.send_command("HOMING")

        while True: 
            response = self.send_command("GET_STATUS")
            if response == "READY":
                print("Robot homed!")
                break
    
    def fordward_kinematics(self, q1, q2, q3, q4):
        # response = self.send_command(f"MOVETO {x} {y} {z}")
        # return response
        pass

    def inverse_kinematics(self, x, y, z):
        # response = self.send_command(f"MOVETO {x} {y} {z}")
        # return response
        pass

    def move_to_position(self, x, y, z):
        angles,distances = self.inverse_kinematics(x, y, z)
        print(f"Moving to position ({x}, {y}, {z})")
        command = f"MOVETO {angles[0]} {angles[1]} {angles[2]} {angles[3]} {distances[0]} {distances[1]} {distances[2]} {distances[3]}"
        self.send_command(command)

        while True:
            response = self.send_command("GET_STATUS")
            if response == "READY":
                print("Robot moved!")
                break
