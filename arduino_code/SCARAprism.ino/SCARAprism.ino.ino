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
    stepper3.moveTo(-4 * stepsPerMmLinear);  // Mover el motor una pequeña distancia
    debugPrint("Rutina de home terminada :D");
    inputString = "";
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
        stringComplete = false;
        inputString = "";
    }
}
void serialEvent() {
    if (Serial.available()) {
        inputString = Serial.readStringUntil('\n'); // Read until newline
        inputString.trim(); // Remove any trailing whitespace
        processInput(inputString);
    }
}

void processInput(String command) {
    if (command.startsWith("HOME")) {
        debugPrint("Received HOME command");
        homingRoutine();
        inputString = "";
        stringComplete = false;
    } else if (command.startsWith("MOVE")) {
        debugPrint("Received MOVE command");
        stringComplete = true;
    } else {
        Serial.print(command);
        debugPrint("Command not recognized");
        stringComplete = false;
        inputString = "";
    }
     // Clear the input string for the next command
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
    inputString.reserve(200);

    // Configuración de la velocidad máxima y aceleración de los motores paso a paso
    //VELOCIDAD MAXIMA PARA LOS MOTORES: 396
    stepper1.setMaxSpeed(270);  // Velocidad máxima ajustada a 396 pasos/segundo
    stepper1.setAcceleration(500);  // Aceleración ajustada

    stepper2.setMaxSpeed(270);  // Velocidad máxima ajustada a 396 pasos/segundo
    stepper2.setAcceleration(500);  // Aceleración ajustada

    stepper3.setMaxSpeed(270);  // Velocidad máxima ajustada a 396 pasos/segundo
    stepper3.setAcceleration(500);  // Aceleración ajustada

    // Inicializa la comunicación serial
    Serial.begin(115200);
    debugPrint("Arduino iniciado");
}

void loop() {
  
  while(!Serial.available()){}
    serialEvent();
    processInput(inputString);
    serialReading();

  while(stepper1.distanceToGo() != 0||stepper2.distanceToGo() != 0||stepper3.distanceToGo() != 0) {
      // Ejecutar movimientos de los motores paso a paso

      if (stepper1.distanceToGo() != 0) {stepper1.run();}

      if (stepper2.distanceToGo() != 0) {stepper2.run();}

      if (stepper3.distanceToGo() != 0) {stepper3.run();}
  }
  debugPrint("MOVEMENT_COMPLETE");
}
