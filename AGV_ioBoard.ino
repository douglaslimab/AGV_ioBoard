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
#include "Sonar.h"

String dataJSON, encoder_A0, encoder_A1, encoder_B0, encoder_B1, V_motor_A, V_motor_B, V_battery, I_motor_A, I_motor_B, I_battery, ultrassonic_0, ultrassonic_1, ultrassonic_2, ultrassonic_3, ultrassonic_4, ultrassonic_5, x_axis, y_axis, z_axis;

Sonar sonar0(34, 35, 255);
Sonar sonar1(36, 37, 255);
Sonar sonar2(38, 39, 255);
Sonar sonar3(40, 41, 255);
Sonar sonar4(42, 43, 255);
Sonar sonar5(44, 45, 255);

void setup() {
  Serial.begin(115200);
}

void loop() {
  send_data();

  Serial.println(dataJSON);/*
  Serial.println(int(dataJSON[1]));
  Serial.println(int(dataJSON[3]));
  Serial.println(int(dataJSON[5]));
  Serial.println(int(dataJSON[7]));
  Serial.println(int(dataJSON[9]));
  Serial.println(int(dataJSON[11]));
  Serial.println(int(dataJSON[13]));
  Serial.println(int(dataJSON[15]));
  Serial.println(int(dataJSON[17]));
  Serial.println(int(dataJSON[19]));*/
  Serial.println(int(dataJSON[21])*4);
  Serial.println(int(dataJSON[23])*4);
  Serial.println(int(dataJSON[25])*4);
  Serial.println(int(dataJSON[27])*4);
  Serial.println(int(dataJSON[29])*4);
  Serial.println(int(dataJSON[31])*4);/*
  Serial.println(int(dataJSON[33]));
  Serial.println(int(dataJSON[35]));
  Serial.println(int(dataJSON[37]));*/
 
  delay(100);
}

void send_data(){
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
  ultrassonic_0 = char(sonar0.measure());
  ultrassonic_1 = char(sonar1.measure());
  ultrassonic_2 = char(sonar2.measure());
  ultrassonic_3 = char(sonar3.measure());
  ultrassonic_4 = char(sonar4.measure());
  ultrassonic_5 = char(sonar5.measure());
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

void set_motorA(bool en, bool dir, int speed){
  Serial.print(en);
  Serial.print(dir);
  Serial.print(speed);
}

void set_motorB(bool en, bool dir, int speed){
  Serial.print(en);
  Serial.print(dir);
  Serial.print(speed);
}

void set_relays(bool coil_0, bool coil_1, bool coil_2, bool coil_3){

}