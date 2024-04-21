#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <DRV8825.h>
#include <SyncDriver.h>
#include <MultiDriver.h> //see if this one is being used womp womp


#define MOTOR_STEPS 200

#define STEPPER_BASE_DIR 2
#define STEPPER_BASE_STEP 3
#define STEPPER_SECOND_DIR 4
#define STEPPER_SECOND_STEP 5
#define STEPPER_PRISMATIC_DIR 6
#define STEPPER_PRISMATIC_STEP 7
#define SERVO_PIN 9 // PWM

#define ENDSTOP_BASE_PIN 8
#define ENDSTOP_SECOND_PIN 10
#define ENDSTOP_PRISMATIC_PIN 11

AccelStepper stepperBase(AccelStepper::DRIVER, STEPPER_BASE_STEP, STEPPER_BASE_DIR);
AccelStepper stepperSecond(AccelStepper::DRIVER, STEPPER_SECOND_STEP, STEPPER_SECOND_DIR);
AccelStepper stepperPrismatic(AccelStepper::DRIVER, STEPPER_PRISMATIC_STEP, STEPPER_PRISMATIC_DIR);
Servo servo;

bool endstopBase = false;
bool endstopSecond = false;
bool endstopPrismatic = false;
bool homingComplete = false;
bool movementComplete = false;

//serial communication with python
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
int integerFromPC = 0;
int integerFromPC2 = 0;
int integerFromPC3 = 0;
char messageFromPC[numChars] = {0};
boolean newData = false;


void setup() {
    Serial.begin(115200);
    servo.attach(SERVO_PIN);
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
        if (digitalRead(ENDSTOP_BASE_PIN) == HIGH) 
            stepperBase.setCurrentPosition(0);
            stepperBase.stop();
            endstopBase = true;
        }
        if (digitalRead(ENDSTOP_SECOND_PIN) == HIGH) {
            stepperSecond.setCurrentPosition(0);
            stepperSecond.stop();
            endstopSecond = true;
        }
        if (digitalRead(ENDSTOP_PRISMATIC_PIN) == HIGH) {
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