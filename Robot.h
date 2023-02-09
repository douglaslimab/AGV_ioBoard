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
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
    void stop();

    // sensing methods
    void attachInterrupt(uint8_t pin, void (*isr)(), uint8_t mode);
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
    String printStatusArray();
    char getCommandsArray();

  private:
    int leftMotorPwmPin;
    int leftMotorDirPin;
    int leftMotorBreakPin;
    int rightMotorPwmPin;
    int rightMotorDirPin;
    int rightMotorBreakPin;

//    int pulseInterrupt;

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

    
    byte robotData[10];

};

Robot::Robot() {
  // initialize pins and other settings
  
  // motor control pins
  leftMotorPwmPin = 11;
  leftMotorDirPin = 13;
  leftMotorBreakPin = 8;
  rightMotorPwmPin = 10;
  rightMotorDirPin = 12;
  rightMotorBreakPin = 9;


  // Variables to store the time between pulses
  unsigned long pulseTime = 0;
  unsigned long previousPulseTime = 0;

  // encoders sensing pins
  leftEncoderSensor0Pin = 26;
  leftEncoderSensor1Pin = 27;
  rightEncoderSensor0Pin = 28;
  rightEncoderSensor1Pin = 29;
  
  // Set the pulsePin as an input
  pinMode(leftEncoderSensor0Pin, INPUT);
    
  // motors voltage sensing pins
  leftMotorVoltageSensorPin = 19;
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

void Robot::attachInterrupt(uint8_t pin, void (*isr)(), uint8_t mode) {
  ::attachInterrupt(digitalPinToInterrupt(pin), isr, mode);
}

void Robot::moveForward() { // distance, angle, speed..
  // reset encoders
  // spin robot to desired angle
  // keep in a loop while encoder distance < final distance and no obstacle
  // stop
  analogWrite(10, 250);
  analogWrite(11, 250);
  digitalWrite(12, HIGH);
  digitalWrite(13, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
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
  analogWrite(rightMotorPwmPin, 0);
  analogWrite(leftMotorPwmPin, 0);
  digitalWrite(rightMotorDirPin, LOW);
  digitalWrite(leftMotorDirPin, LOW);
  digitalWrite(rightMotorBreakPin, LOW);
  digitalWrite(leftMotorBreakPin, LOW);
}

int pulseInterrupt(){

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
  moveForward();
}

String Robot::printStatusArray(){
  String dataJSON, encoder_A0, encoder_A1, encoder_B0, encoder_B1, V_motor_A, V_motor_B, V_battery, I_motor_A, I_motor_B, I_battery, ultrassonic_0, ultrassonic_1, ultrassonic_2, ultrassonic_3, ultrassonic_4, ultrassonic_5, x_axis, y_axis, z_axis;

  // after get sensor reading convert to String
  encoder_A0 = char(51);
  encoder_A1 = char(52);
  encoder_B0 = char(53);
  encoder_B1 = char(54);
  V_motor_A = char(55);
  V_motor_B = char(56);
  V_battery = char(57);
  I_motor_A = char(58);
  I_motor_B = char(59);
  I_battery = char(60);
  ultrassonic_0 = readUltrasonicSensor0();  // all data need to be sent as char and converted after receiving
  ultrassonic_1 = readUltrasonicSensor1();
  ultrassonic_2 = readUltrasonicSensor2();
  ultrassonic_3 = readUltrasonicSensor3();
  ultrassonic_4 = readUltrasonicSensor4();
  ultrassonic_5 = readUltrasonicSensor5();
  x_axis = char(67);
  y_axis = char(68);
  z_axis = char(69);
  
  dataJSON = "{";
  dataJSON += encoder_A0 + ",";
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
  dataJSON += z_axis + "}";

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