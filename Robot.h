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
    int readLeftEncoder();
    int readRightEncoder();
    
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

  private:
    int leftMotorPwmPin;
    int leftMotorDirPin;
    int leftMotorBreakPin;
    int rightMotorPwmPin;
    int rightMotorDirPin;
    int rightMotorBreakPin;

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

};

Robot::Robot() {
  // initialize pins and other settings
  
  // motor control pins
  leftMotorPwmPin = 3;
  leftMotorDirPin = 4;
  leftMotorBreakPin = 5;
  rightMotorPwmPin = 6;
  rightMotorDirPin = 7;
  rightMotorBreakPin = 8;

  // encoders sensing pins
  leftEncoderSensor0Pin = 9;
  leftEncoderSensor1Pin = 10;
  rightEncoderSensor0Pin = 11;
  rightEncoderSensor1Pin = 12;
  
  // motors voltage sensing pins
  leftMotorVoltageSensorPin = 13;
  rightMotorVoltageSensorPin = 14;
  
  // motors current sensing pins
  leftMotorCurrentSensorPin = 15;
  rightMotorCurrentSensorPin = 16;

  // battery voltage and current sensing pins
  batteryVoltageSensorPin = 17;
  batteryCurrentSensorPin = 18;
  
  // pins used to read ultrasonic sensors
  ultrasonicSensor0TrigPin = 19;
  ultrasonicSensor0EchoPin = 20;
  ultrasonicSensor1TrigPin = 21;
  ultrasonicSensor1EchoPin = 22;
  ultrasonicSensor2TrigPin = 23;
  ultrasonicSensor2EchoPin = 24;
  ultrasonicSensor3TrigPin = 25;
  ultrasonicSensor3EchoPin = 26;
  ultrasonicSensor4TrigPin = 27;
  ultrasonicSensor4EchoPin = 28;
  ultrasonicSensor5TrigPin = 29;
  ultrasonicSensor5EchoPin = 30;
  // other initializations
}

void Robot::moveForward() {
  // code to control the motors to move forward
}

void Robot::moveBackward() {
  // code to control the motors to move backward
}

void Robot::turnLeft() {
  // code to control the motors to turn left
}

void Robot::turnRight() {
  // code to control the motors to turn right
}

void Robot::stop() {
  // code to stop the motors
}

// read speed and direction
int Robot::readLeftEncoder(){
  int speed = 0;
  int dir = 0;

  // [speed, dir]
  return char(speed) + char(dir);  // return char data type
}

// read speed and direction
int Robot::readRightEncoder(){
  int speed = 0;
  int dir = 0;

  // [speed, dir]
  return char(speed) + char(dir);  // return char data type
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

int readUltrasonicSensor(int trigPin, int echoPin) {
  int duration, distance;
  
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read echo pulse
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance
  distance = (duration/2) / 29.1;
  
  return distance;
}

// read distance in mm read in sensor0
int Robot::readUltrasonicSensor0(){
  int trigPin = 34;
  int echoPin = 35;
  return readUltrasonicSensor(trigPin, echoPin);
}

int Robot::readUltrasonicSensor1(){
  int trigPin = 36;
  int echoPin = 37;
  return readUltrasonicSensor(trigPin, echoPin);
}

int Robot::readUltrasonicSensor2(){
  int trigPin = 38;
  int echoPin = 39;
  return readUltrasonicSensor(trigPin, echoPin);
}

int Robot::readUltrasonicSensor3(){
  int trigPin = 40;
  int echoPin = 41;
  return readUltrasonicSensor(trigPin, echoPin);
}

int Robot::readUltrasonicSensor4(){
  int trigPin = 42;
  int echoPin = 43;
  return readUltrasonicSensor(trigPin, echoPin);
}

int Robot::readUltrasonicSensor5(){
  int trigPin = 44;
  int echoPin = 45;
  return readUltrasonicSensor(trigPin, echoPin);
}

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