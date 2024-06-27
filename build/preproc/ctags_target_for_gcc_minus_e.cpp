# 1 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino"
# 2 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino" 2
# 3 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino" 2
# 4 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino" 2
# 5 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino" 2
# 6 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino" 2
# 7 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino" 2
# 23 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino"
AccelStepper stepperBase(AccelStepper::DRIVER, 3, 2);
AccelStepper stepperSecond(AccelStepper::DRIVER, 5, 4);
AccelStepper stepperPrismatic(AccelStepper::DRIVER, 7, 6);
Servo servo;

bool endstopBase = false;
bool endstopSecond = false;
bool endstopPrismatic = false;
bool homingComplete = false;
bool movementComplete = false;

//serial communication with python
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; // temporary array for use when parsing
int integerFromPC = 0;
int integerFromPC2 = 0;
int integerFromPC3 = 0;
char messageFromPC[numChars] = {0};
boolean newData = false;


void setup() {
    Serial.begin(115200);
    servo.attach(9 /* PWM*/);
    //for the steppers motors, set the maximum speed, acceleration and target position
    stepperBase.setMaxSpeed(200); //try another values
    stepperBase.setAcceleration(100); //try another values
    stepperSecond.setMaxSpeed(200); //try another values
    stepperSecond.setAcceleration(100); //try another values
    stepperPrismatic.setMaxSpeed(200); //try another values
    stepperPrismatic.setAcceleration(100); //try another values
}

void loop(){
 //verify recieved commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processCommand(command);
    }
}
//process command function
void processCommand(String command){
    if (command == "home_robot") {
        home();
    }
    else if (command.startsWith("MOVETO")) {
        float q1 = command.substring(5,command.indexOf(",")).toFloat();
        float q2 = command.substring(command.indexOf(",")+1,command.lastIndexOf(",")).toFloat();
        float q3 = command.substring(command.indexOf(",")+1,command.lastIndexOf(",")).toFloat();
        float q4 = command.substring(command.lastIndexOf(",")+1).toFloat();

        moveTo(q1, q2, q3, q4);
    }
}
//homing function
void home() {
    stepperBase.setSpeed(-200);
    stepperSecond.setSpeed(-200);
    stepperPrismatic.setSpeed(-200);
    while (!endstopBase || !endstopSecond || !endstopPrismatic) {
        stepperBase.runSpeed();
        stepperSecond.runSpeed();
        stepperPrismatic.runSpeed();
        if (digitalRead(8) == 0x1)
            stepperBase.setCurrentPosition(0);
            stepperBase.stop();
            endstopBase = true;
        }
        if (digitalRead(10) == 0x1) {
            stepperSecond.setCurrentPosition(0);
            stepperSecond.stop();
            endstopSecond = true;
        }
        if (digitalRead(11) == 0x1) {
            stepperPrismatic.setCurrentPosition(0);
            stepperPrismatic.stop();
            endstopPrismatic = true;
        }
    homingComplete = true;
    Serial.println("Homing complete");
    }

    void moveTo(float q1, float q2, float q3, float q4) {
        if (!homingComplete) {
            Serial.println("Robot not homed yet");
            return;
        }
        stepperBase.moveTo(q1);
        stepperSecond.moveTo(q2);
        stepperPrismatic.moveTo(q3);
        servo.write(q4);
        while (stepperBase.distanceToGo() != 0 || stepperSecond.distanceToGo() != 0 || stepperPrismatic.distanceToGo() != 0) {
            stepperBase.run();
            stepperSecond.run();
            stepperPrismatic.run();
        }
        movementComplete = true;
        Serial.println("Movement complete");
}
