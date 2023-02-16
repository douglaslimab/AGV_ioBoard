volatile unsigned long pulseTimeLeft = 0;
volatile unsigned long previousPulseTimeLeft = 0;
volatile unsigned long deltaTimeLeft = 0;
volatile unsigned long countLeft;

volatile unsigned long pulseTimeRight = 0;
volatile unsigned long previousPulseTimeRight = 0;
volatile unsigned long deltaTimeRight = 0;
volatile unsigned long countRight;

volatile int setSpeedLeft = 0;
volatile int setSpeedRight = 0;

void encoder1A_ISR(){
  countLeft++;
  pulseTimeLeft = micros();
  deltaTimeLeft = pulseTimeLeft - previousPulseTimeLeft;
  previousPulseTimeLeft = pulseTimeLeft;
}

void encoder1B_ISR(){
}

void encoder2A_ISR(){
  countRight++;
  pulseTimeRight = micros();
  deltaTimeRight = pulseTimeRight - previousPulseTimeRight;
  previousPulseTimeRight = pulseTimeRight;
}

void encoder2B_ISR(){
}

class Robot {
  public:
    Robot();
    // motion methods
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
    void stop();

    // sensing methods
    void initializeEncoderPins(){
      attachInterrupt(digitalPinToInterrupt(leftEncoderSensor0Pin), encoder1A_ISR, RISING);
//      attachInterrupt(digitalPinToInterrupt(leftEncoderSensor1Pin), encoder1B_ISR, RISING);
      attachInterrupt(digitalPinToInterrupt(rightEncoderSensor0Pin), encoder2A_ISR, RISING);
//      attachInterrupt(digitalPinToInterrupt(rightEncoderSensor1Pin), encoder2B_ISR, RISING);
      Serial.println("init..");
    }
    int readLeftEncoder();
    int readRightEncoder();

    int setLeftMotorSpeed(int speed);
    int getLeftMotorError(int speed);
        
    int getDistanceTravelledLeft();
    int getDistanceTravelledRight();
    void resetEncoders();

    
    void spinAngle(int angle);

    int readMotorsVoltage();    
    int readMotorsCurrent();    
    int readBattery();

    int readUltrasonicSensor0();
    int readUltrasonicSensor1();
    int readUltrasonicSensor2();
    int readUltrasonicSensor3();
    int readUltrasonicSensor4();
    int readUltrasonicSensor5();

    int* getReadings();

    void checkDistanceClearance();
    void spinToClearestPoint();

    void moveToPosition(int x, int y);
    String printStatusArray();
    char getCommandsArray();

  private:
    int leftMotorPwmPin;
    int leftMotorInAPin;
    int leftMotorInBPin;
    int rightMotorPwmPin;
    int rightMotorInAPin;
    int rightMotorInBPin;

    int leftEncoderSensor0Pin;
    int leftEncoderSensor1Pin;
    int rightEncoderSensor0Pin;
    int rightEncoderSensor1Pin;
    
    int leftMotorVoltageSensorPin;
    int rightMotorVoltageSensorPin;

    int leftMotorCurrentSensorPin;
    int rightMotorCurrentSensorPin;

    int batteryVoltageSensorPin;
    int batteryCurrentSensorPin;

    int ultrasonicSensor0TrigPin;
    int ultrasonicSensor0EchoPin;
    int ultrasonicSensor1TrigPin;
    int ultrasonicSensor1EchoPin;
    int ultrasonicSensor2TrigPin;
    int ultrasonicSensor2EchoPin;
    int ultrasonicSensor3TrigPin;
    int ultrasonicSensor3EchoPin;
    int ultrasonicSensor4TrigPin;
    int ultrasonicSensor4EchoPin;
    int ultrasonicSensor5TrigPin;
    int ultrasonicSensor5EchoPin;
    
    byte robotData[10];

};

Robot::Robot() {
  // initialize pins and other settings
  
  // motor control pins
  leftMotorPwmPin = 11;
  leftMotorInAPin = 13;
  leftMotorInBPin = 8;
  rightMotorPwmPin = 10;
  rightMotorInAPin = 12;
  rightMotorInBPin = 9;

  pinMode(leftMotorPwmPin, OUTPUT);
  pinMode(leftMotorInAPin, OUTPUT);
  pinMode(leftMotorInBPin, OUTPUT);
  pinMode(rightMotorPwmPin, OUTPUT);
  pinMode(rightMotorInAPin, OUTPUT);
  pinMode(rightMotorInBPin, OUTPUT);


  // Variables to store the time between pulses
  unsigned long pulseTime = 0;
  unsigned long previousPulseTime = 0;

  // encoders sensing pins
  leftEncoderSensor0Pin = 20;
  leftEncoderSensor1Pin = 21;
  rightEncoderSensor0Pin = 18;
  rightEncoderSensor1Pin = 19;

  pinMode(leftEncoderSensor0Pin, INPUT_PULLUP);
  pinMode(leftEncoderSensor1Pin, INPUT_PULLUP);
  pinMode(rightEncoderSensor0Pin, INPUT_PULLUP);
  pinMode(rightEncoderSensor1Pin, INPUT_PULLUP);
  
  // Set the pulsePin as an input
  pinMode(leftEncoderSensor0Pin, INPUT);
    
  // motors voltage sensing pins
  leftMotorVoltageSensorPin = A0;
  rightMotorVoltageSensorPin = 14;
  
  // motors current sensing pins
  leftMotorCurrentSensorPin = 15;
  rightMotorCurrentSensorPin = 16;

  // battery voltage and current sensing pins
  batteryVoltageSensorPin = 17;
  batteryCurrentSensorPin = 18;
  
  // pins used to read ultrasonic sensors
  ultrasonicSensor0TrigPin = 34;
  ultrasonicSensor0EchoPin = 35;
  ultrasonicSensor1TrigPin = 36;
  ultrasonicSensor1EchoPin = 37;
  ultrasonicSensor2TrigPin = 38;
  ultrasonicSensor2EchoPin = 39;
  ultrasonicSensor3TrigPin = 40;
  ultrasonicSensor3EchoPin = 41;
  ultrasonicSensor4TrigPin = 42;
  ultrasonicSensor4EchoPin = 43;
  ultrasonicSensor5TrigPin = 44;
  ultrasonicSensor5EchoPin = 45;

  // other initializations
  // ultrasonic sensors pins
  pinMode(ultrasonicSensor0TrigPin, OUTPUT);
  pinMode(ultrasonicSensor0EchoPin, INPUT); 
  pinMode(ultrasonicSensor1TrigPin, OUTPUT);
  pinMode(ultrasonicSensor1EchoPin, INPUT); 
  pinMode(ultrasonicSensor2TrigPin, OUTPUT);
  pinMode(ultrasonicSensor2EchoPin, INPUT); 
  pinMode(ultrasonicSensor3TrigPin, OUTPUT);
  pinMode(ultrasonicSensor3EchoPin, INPUT); 
  pinMode(ultrasonicSensor4TrigPin, OUTPUT);
  pinMode(ultrasonicSensor4EchoPin, INPUT); 
  pinMode(ultrasonicSensor5TrigPin, OUTPUT);
  pinMode(ultrasonicSensor5EchoPin, INPUT);  
}

int Robot::setLeftMotorSpeed(int speed){
  setSpeedLeft = speed;

  int error = getLeftMotorError(setSpeedLeft);

  float correction = error * 0.5;

  setSpeedLeft += correction;

  return error;
}

int Robot::getLeftMotorError(int speed){
  return speed - readLeftEncoder();
}

void Robot::moveForward(){ // distance, angle, speed..
  // reset encoders
  // spin robot to desired angle
  // keep in a loop while encoder distance < final distance and no obstacle
  // stop
  analogWrite(rightMotorPwmPin, 250);
  analogWrite(leftMotorPwmPin, 250);
  digitalWrite(rightMotorInAPin, HIGH);
  digitalWrite(rightMotorInBPin, LOW);
  digitalWrite(leftMotorInAPin, LOW);
  digitalWrite(leftMotorInBPin, HIGH);
}

void Robot::moveBackward() {
  // code to control the motors to move backward
  analogWrite(rightMotorPwmPin, 250);
  analogWrite(leftMotorPwmPin, 250);
  digitalWrite(rightMotorInAPin, LOW);
  digitalWrite(rightMotorInBPin, HIGH);
  digitalWrite(leftMotorInAPin, HIGH);
  digitalWrite(leftMotorInBPin, LOW);
}

void Robot::turnLeft() {
  // code to control the motors to turn left
  analogWrite(rightMotorPwmPin, 250);
  analogWrite(leftMotorPwmPin, 250);
  digitalWrite(rightMotorInAPin, LOW);
  digitalWrite(rightMotorInBPin, HIGH);
  digitalWrite(leftMotorInAPin, LOW);
  digitalWrite(leftMotorInBPin, HIGH);
}

void Robot::turnRight() {
  // code to control the motors to turn right
  analogWrite(rightMotorPwmPin, 250);
  analogWrite(leftMotorPwmPin, 250);
  digitalWrite(rightMotorInAPin, HIGH);
  digitalWrite(rightMotorInBPin, LOW);
  digitalWrite(leftMotorInAPin, HIGH);
  digitalWrite(leftMotorInBPin, LOW);
}

void Robot::stop() {
  // code to stop the motors
  analogWrite(rightMotorPwmPin, 0);
  analogWrite(leftMotorPwmPin, 0);
  digitalWrite(rightMotorInAPin, LOW);
  digitalWrite(leftMotorInAPin, LOW);
  digitalWrite(rightMotorInBPin, LOW);
  digitalWrite(leftMotorInBPin, LOW);
  digitalWrite(leftMotorInBPin, LOW);
}

/*

    Encoder Methods

*/

// read speed and direction
int Robot::readLeftEncoder(){
  double speed = 60*1000000/(120*deltaTimeLeft*34*3);
  return speed; //char(speed) + char(dir);  // return char data type
}

// read speed and direction
int Robot::readRightEncoder(){
  double speed = 60*1000000/(120*deltaTimeRight*34*3);
  return speed; //char(speed) + char(dir);  // return char data type
}

int Robot::getDistanceTravelledLeft(){
  return (countLeft/120)*(PI*72);
}

int Robot::getDistanceTravelledRight(){
  return (countRight/120)*(PI*72);
}

void Robot::spinAngle(int angle){
  const float wheelDiameter = 65;
  const float distanceBetweenWheels = 200;
  const float wheelCircumference = wheelDiameter * PI;

  float arch = (angle / 360.0) * distanceBetweenWheels * PI;
  int encoderCount = (int) ((arch / wheelCircumference) * 120.0);

  turnLeft();

  while (countLeft + countRight < encoderCount * 2){
  }

  stop();
  resetEncoders();
}

void Robot::resetEncoders(){
  countLeft = 0;
  countRight = 0;
}

// read voltage in both motors left and right
int Robot::readMotorsVoltage(){
  int leftMotorVoltageValue = 0;
  int rightMotorVoltageValue = 0;

  // [leftMotorVoltageValue, rightMotorVoltageValue]
  return char(leftMotorVoltageValue) + char(rightMotorVoltageValue);      // return char data type
}

// read current in both motors left and right
int Robot::readMotorsCurrent(){
  int leftMotorCurrentValue = 0;
  int rightMotorCurrentValue = 0;

  // [leftMotorCurrentValue, rightMotorCurrentValue]
  return char(leftMotorCurrentValue) + char(rightMotorCurrentValue);      // return char data type
}

// read voltage and current in the battery
int Robot::readBattery(){
  int voltageValue = 0;
  int currentValue = 0;

  // [voltageValue, currentValue]
  return char(voltageValue) + char(currentValue);   // return char data type
}

/*

    Sonar Methods 

*/
int readUltrasonicSensor(int trigPin, int echoPin) {
  cli();
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  int time = pulseIn(echoPin, HIGH, 5000);
  int currentDistance = time*0.1715;
  
  if(currentDistance == 0){
    currentDistance = 255;
  }
  sei();
  return currentDistance;   // divide by 4 to keep range of 255 bits
}

// read distance in mm read in sensor0
int Robot::readUltrasonicSensor0(){
  return readUltrasonicSensor(ultrasonicSensor0TrigPin, ultrasonicSensor0EchoPin);
}

int Robot::readUltrasonicSensor1(){
  return readUltrasonicSensor(ultrasonicSensor1TrigPin, ultrasonicSensor1EchoPin);
}

int Robot::readUltrasonicSensor2(){
  return readUltrasonicSensor(ultrasonicSensor2TrigPin, ultrasonicSensor2EchoPin);
}

int Robot::readUltrasonicSensor3(){
  return readUltrasonicSensor(ultrasonicSensor3TrigPin, ultrasonicSensor3EchoPin);
}

int Robot::readUltrasonicSensor4(){
  return readUltrasonicSensor(ultrasonicSensor4TrigPin, ultrasonicSensor4EchoPin);
}

int Robot::readUltrasonicSensor5(){
  return readUltrasonicSensor(ultrasonicSensor5TrigPin, ultrasonicSensor5EchoPin);
}

/*

    Communication

*/
int* Robot::getReadings(){
  static int readings[15];

  readings[0] = readLeftEncoder();
  readings[2] = readRightEncoder();
  readings[4] = readMotorsVoltage();
  readings[6] = readMotorsCurrent();
  readings[8] = readBattery();
  readings[10] = readUltrasonicSensor0();
  readings[11] = readUltrasonicSensor1();
  readings[12] = readUltrasonicSensor2();
  readings[13] = readUltrasonicSensor3();
  readings[14] = readUltrasonicSensor4();
  readings[15] = readUltrasonicSensor5();

  return readings;
}
/*

    Navigation Methods

*/
void Robot::checkDistanceClearance(){
  // check ultrasonic sensor 0
  // return distance
}

void Robot::spinToClearestPoint(){
  // read all sensors
  // find the clearest way
  // spin the robot to the clearest way
}

void Robot::moveToPosition(int x, int y){
  int distance = sqrt(x*x + y*y);
  int angle = atan2(y, x) * 180 / PI;
  moveForward();
}

/*

    Send Sensor Readings

*/
String Robot::printStatusArray(){
  String dataJSON, encoder_A0, encoder_A1, encoder_B0, encoder_B1, V_motor_A, V_motor_B, V_battery, I_motor_A, I_motor_B, I_battery, ultrassonic_0, ultrassonic_1, ultrassonic_2, ultrassonic_3, ultrassonic_4, ultrassonic_5, x_axis, y_axis, z_axis;

  encoder_A0 = readLeftEncoder();
  encoder_A1 = char(52);
  encoder_B0 = readRightEncoder();
  encoder_B1 = char(54);
  V_motor_A = char(55);
  V_motor_B = char(56);
  V_battery = char(57);
  I_motor_A = char(58);
  I_motor_B = char(59);
  I_battery = char(60);
  ultrassonic_0 = readUltrasonicSensor0();
  ultrassonic_1 = readUltrasonicSensor1();
  ultrassonic_2 = readUltrasonicSensor2();
  ultrassonic_3 = readUltrasonicSensor3();
  ultrassonic_4 = readUltrasonicSensor4();
  ultrassonic_5 = readUltrasonicSensor5();
  x_axis = char(67);
  y_axis = char(68);
  z_axis = char(69);
  
  dataJSON = encoder_A0 + ",";
  dataJSON += encoder_A1 + ",";
  dataJSON += encoder_B0 + ",";
  dataJSON += encoder_B1 + ",";
  dataJSON += V_motor_A + ",";
  dataJSON += V_motor_B + ",";
  dataJSON += V_battery + ",";
  dataJSON += I_motor_A + ",";
  dataJSON += I_motor_A + ",";
  dataJSON += I_battery + ",";
  dataJSON += ultrassonic_0 + ",";
  dataJSON += ultrassonic_1 + ",";
  dataJSON += ultrassonic_2 + ",";
  dataJSON += ultrassonic_3 + ",";
  dataJSON += ultrassonic_4 + ",";
  dataJSON += ultrassonic_5 + ",";
  dataJSON += x_axis + ",";
  dataJSON += y_axis + ",";
  dataJSON += z_axis;

  return dataJSON;
}

char Robot::getCommandsArray(){
  if (Serial.available()) {
    for (int i = 0; i <= 4; i++) {
      robotData[i] = Serial.read();
      delay(5);
    }
  }
  return robotData;
}