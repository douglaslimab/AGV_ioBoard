/*

  implement encoders readings;
  move robot according to cordinates;
  avoidance collision control;
  recalculate the way if there is obstacle;
  detect collision (angle, direction);
  control the motor speed controlling the pwm based on encoder readings;
  calculate distance moved based on encoders;
  

*/

class Robot {
  public:
    Robot();
    // motion methods
    void moveForward(int distance, int angle);
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

    void pulseInterrupt();

    void checkDistanceClearance();
    void spinToClearestPoint();

    void moveToPosition(int x, int y);

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
    
    const int pulsePin = 2;
    unsigned long pulseTime = 0;
    unsigned long previousPulseTime = 0;

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

void Robot::moveForward(int distance, int angle) {
  // reset encoders
  // spin robot to desired angle
  // keep in a loop while encoder distance < final distance and no obstacle
  // stop
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
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    int time = pulseIn(echoPin, HIGH, 5000);
    int currentDistance = time*0.1715;
    
    if(currentDistance == 0){
      currentDistance = 255;
    }

    return currentDistance/4;   // divide by 4 to keep range of 255 bits
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

void Robot::pulseInterrupt(){
  // Get the current time
  pulseTime = micros();
  
  // Calculate the time between pulses
  unsigned long deltaTime = pulseTime - previousPulseTime;
  
  // Store the current time as the previous pulse time for the next pulse
  previousPulseTime = pulseTime;
  
  // Debugging output
  Serial.println(deltaTime);
}

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
  moveForward(distance, angle);
}