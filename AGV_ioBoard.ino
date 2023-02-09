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
byte data[10];

void interruptServiceRoutine() {
  // Interrupt code here
}

void setup() {
  Serial.begin(115200);
  robot.attachInterrupt(9, interruptServiceRoutine, RISING);
}

void loop() {
  get_data();

  if(data[0] == 49){
    if(int(robot.readUltrasonicSensor0()) > 100){
      robot.moveForward();
    }else{
      robot.stop();
    }
  } else if(data[0] == 50){
    robot.stop();
  }

  Serial.println(robot.printStatusArray());
  delay(100);
}

byte get_data(){
  if (Serial.available()) {
    for (int i = 0; i <= 4; i++) {
      data[i] = Serial.read();
      delay(5);
    }
  }    
}

void data_handler(){

}