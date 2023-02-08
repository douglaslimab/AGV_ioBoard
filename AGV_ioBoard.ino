/*
  Robot()
  Motor()
  Sensor()

  array structure:

    {
      ,
    }

    send this:
      -timestamp
      -encoders: [encoder_A0, encoder_A1, encoder_B0, encoder_B1]
      -voltage: [motor_A, motor_B, battery]
      -current: [motor_A, motor_B, battery]
      -distance: [ultrassonic_0, ultrassonic_1, ultrassonic_2, ultrassonic_3, ultrassonic_4, ultrassonic_5]
      -accel: [x_axis, y_axis, z_axis]

    receive this:
      en_motor_A, dir_motor_A, speed_motor_A,
      en_motor_B, dir_motor_B, speed_motor_B,
      relay_0, relay_1, relay_2, relay_3
      
*/
#include "Robot.h"

Robot robot;

void interruptServiceRoutine() {
  // Interrupt code here
}

void setup() {
  Serial.begin(115200);
  robot.attachInterrupt(9, interruptServiceRoutine, RISING);
}

void loop() {
  Serial.println(robot.printStatusArray());
  delay(100);
}

void get_data(String receiveData){
  Serial.println(int(receiveData[1]));
  Serial.println(int(receiveData[3]));
  Serial.println(int(receiveData[5]));
  Serial.println(int(receiveData[7]));
  Serial.println(int(receiveData[9]));
  Serial.println(int(receiveData[11]));
  Serial.println(int(receiveData[13]));
  Serial.println(int(receiveData[15]));
  Serial.println(int(receiveData[17]));
  Serial.println(int(receiveData[19]));
}