#line 1 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino"
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


#line 45 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino"
void setup();
#line 57 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino"
void loop();
#line 65 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino"
void processCommand(String command);
#line 79 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino"
void home();
#line 106 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino"
void moveTo(float q1, float q2, float q3, float q4);
#line 45 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\SCARAprismático.ino"
void debugPrint(String message);
#line 51 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\SCARAprismático.ino"
void homingRoutine();
#line 105 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\SCARAprismático.ino"
void serialReading();
#line 133 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\SCARAprismático.ino"
void serialEvent();
#line 45 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\arduino_code.ino"
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
#line 1 "C:\\Users\\Karen\\Desktop\\SCARA prismatico\\Prismatic-Scara\\arduino_code\\SCARAprismático.ino"
#include <Servo.h>
#include <AccelStepper.h>

// Pines del servo
int servoPin = 11;
Servo Servo1;

// Pines de los motores paso a paso
#define DIR_PIN1 55
#define STEP_PIN1 54
#define EN_PIN1 38

#define DIR_PIN2 48
#define STEP_PIN2 46
#define EN_PIN2 62

#define DIR_PIN3 61
#define STEP_PIN3 60
#define EN_PIN3 56

#define ENDSTOP_PIN1 3
#define ENDSTOP_PIN2 2
#define ENDSTOP_PIN3 14

// Crea objetos AccelStepper para los motores paso a paso
AccelStepper stepper1(1, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(1, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(1, STEP_PIN3, DIR_PIN3);

// Constantes del sistema
const int stepsPerRevolution = 600; 
const int stepsPerMmLinear = 5;     // Pasos para 1 mm en motor lineal
// Variables de movimiento
float q1, q2, q3;

// Variables para la lectura de la cadena serial
String inputString = "";
bool stringComplete = false;

// Flags para coordinación de operaciones
bool homingComplete = false;
bool serialDataReceived = false;

// Función para imprimir mensajes de depuración
void debugPrint(String message) {
  Serial.print("Arduino Response: ");
  Serial.println(message);
}

// Rutina de homing para las tres rotaciones
void homingRoutine() {
    debugPrint("Iniciando rutina de homing");

    // Reducir la velocidad a la mitad
    stepper1.setMaxSpeed(100);  // Nueva velocidad máxima ajustada a 198 pasos/segundo
    stepper1.setAcceleration(250);  // Aceleración ajustada

    stepper2.setMaxSpeed(100);  // Nueva velocidad máxima ajustada a 198 pasos/segundo
    stepper2.setAcceleration(250);  // Aceleración ajustada

    stepper3.setMaxSpeed(100);  // Nueva velocidad máxima ajustada a 198 pasos/segundo
    stepper3.setAcceleration(250);  // Aceleración ajustada

    // Homing para stepper1
    debugPrint("Homing stepper1");
    stepper1.setSpeed(-50);  // Establecer velocidad negativa para moverse hacia el endstop
    while (digitalRead(ENDSTOP_PIN1) == HIGH) {
        stepper1.runSpeed();
    }
    stepper1.setCurrentPosition(0);  // Establecer posición actual como 0
    digitalWrite(DIR_PIN1, LOW);  // Cambiar dirección para moverse en sentido contrario
    stepper1.moveTo(round(102 * (stepsPerRevolution / 360.0))); // Mueve el motor 125 pasos
    while (stepper1.distanceToGo() != 0) {
        stepper1.run();
    }
    stepper1.setCurrentPosition(0); // Establecer posición actual como 0

    // Homing para stepper2
    debugPrint("Homing stepper2");
    stepper2.setSpeed(50);  // Establecer velocidad positiva para moverse hacia el endstop
    while (digitalRead(ENDSTOP_PIN2) == HIGH) {
        stepper2.runSpeed();
    }
    stepper2.setCurrentPosition(0);  // Establecer posición actual como 0
    digitalWrite(DIR_PIN2, LOW);  // Cambiar dirección para moverse en sentido contrario
    stepper2.moveTo(round(-170 * (stepsPerRevolution / 360.0))); // Mueve el motor 100 pasos
    while (stepper2.distanceToGo() != 0) {
        stepper2.run();
    }
    stepper2.setCurrentPosition(0); // Establecer posición actual como 0

    // Homing para stepper3
    debugPrint("Homing stepper3");
    stepper3.setSpeed(50);  // Establecer velocidad negativa para moverse hacia el endstop
    while (digitalRead(ENDSTOP_PIN3) == HIGH) {
        stepper3.runSpeed();
    }
    stepper3.setCurrentPosition(0); // Establecer posición actual como 0
    stepper3.moveTo(-2 * stepsPerMmLinear);  // Mover el motor una pequeña distancia
    debugPrint("Rutina de home terminada :D");
    Serial.println("HOMING_COMPLETE");
}

// Función para leer la entrada serial y actualizar los valores de q1, q2, q3
void serialReading() {
    if (stringComplete) {
        // Divide la cadena de entrada usando ',' como delimitador
        int delimiter1 = inputString.indexOf(",");
        int delimiter2 = inputString.indexOf(",", delimiter1 + 1);
        int delimiter3 = inputString.indexOf(",", delimiter2 + 1);

        // Obtén los valores como subcadenas y conviértelos a float
        q1 = inputString.substring(0, delimiter1).toFloat();
        q2 = inputString.substring(delimiter1 + 1, delimiter2).toFloat();
        q3 = inputString.substring(delimiter2 + 1, delimiter3).toFloat();
        
        // Configurar la dirección de los motores en función del signo de los valores
        digitalWrite(DIR_PIN1, q1 >= 0 ? LOW : HIGH);
        digitalWrite(DIR_PIN2, q2 >= 0 ? LOW : HIGH);
        digitalWrite(DIR_PIN3, q3 >= 0 ? HIGH : LOW);

        // Mueve los motores de acuerdo a los valores de q1, q2, q3
        debugPrint("Comienzo del movimiento con los parametros recibidos");
        stepper1.moveTo(abs(round(q1 * (stepsPerRevolution / 360.0)))); // Convertir grados a pasos, redondear y tomar valor absoluto
        stepper2.moveTo(abs(round(q2 * (stepsPerRevolution / 360.0)))); // Convertir grados a pasos, redondear y tomar valor absoluto
        stepper3.moveTo(abs(round(q3 * stepsPerMmLinear)));            // Convertir mm a pasos y redondear
        
        // Limpiar la cadena
        inputString = "";
        stringComplete = false;
    }
}
void serialEvent() {
    while (Serial.available()) {
        // Obtener el nuevo byte
        char inChar = (char)Serial.read();
        // Añadir esto al inputString:
        inputString += inChar;
        // Si el carácter entrante es una nueva linea, establecer una bandera para que el ciclo principal pueda hacer algo al respecto
        if (inChar == '\n') {
            if (inputString.startsWith("HOME")) {
                debugPrint("Received HOME command");
                homingRoutine();
                stringComplete = false;
            }
           
            // Revisar si el comando es MOVE
            else if (inputString.startsWith("MOVE")) {
                debugPrint("Received MOVE command");
                stringComplete = true;
            else {
              debugPrint("Instrucción no reconocida");
            }
            }
        }
    }
}

void setup() {
    // Configuración del servo
    Servo1.attach(servoPin);

    // Configuración de los pines de los motores paso a paso como salidas
    pinMode(EN_PIN1, OUTPUT);
    pinMode(EN_PIN2, OUTPUT);
    pinMode(EN_PIN3, OUTPUT);

    // Habilita los motores
    digitalWrite(EN_PIN1, LOW);
    digitalWrite(EN_PIN2, LOW);
    digitalWrite(EN_PIN3, LOW);

    // Configuración de la velocidad máxima y aceleración de los motores paso a paso
    //VELOCIDAD MAXIMA PARA LOS MOTORES: 396
    stepper1.setMaxSpeed(270);  // Velocidad máxima ajustada a 396 pasos/segundo
    stepper1.setAcceleration(500);  // Aceleración ajustada

    stepper2.setMaxSpeed(270);  // Velocidad máxima ajustada a 396 pasos/segundo
    stepper2.setAcceleration(500);  // Aceleración ajustada

    stepper3.setMaxSpeed(270);  // Velocidad máxima ajustada a 396 pasos/segundo
    stepper3.setAcceleration(500);  // Aceleración ajustada

    // Inicializa la comunicación serial
    Serial.begin(9600);
    debugPrint("Arduino iniciado");
}

void loop() {
    serialEvent();
    serialReading();

while(stepper1.distanceToGo() != 0||stepper2.distanceToGo() != 0||stepper3.distanceToGo() != 0) {
    // Ejecutar movimientos de los motores paso a paso

    if (stepper1.distanceToGo() != 0) {
        stepper1.run();
    }

    if (stepper2.distanceToGo() != 0) {
        stepper2.run();
    }

    if (stepper3.distanceToGo() != 0) {
        stepper3.run();
    }
}
debugPrint("Movimiento terminado");
}

