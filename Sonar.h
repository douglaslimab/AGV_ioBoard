
class Sonar{
  int trigPin;
  int echoPin;
  int maxDistance;
  int currentDistance;
  String sensorsArray[6];

  public:
  Sonar(int trigpin, int echopin, int maxdistance){
    trigPin = trigpin;
    echoPin = echopin;
    maxDistance = maxdistance;
    
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
  }

  int measure(){
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    int time = pulseIn(echoPin, HIGH, 5000);
    currentDistance = time*0.1715;
    
    if(currentDistance == 0){
      currentDistance = maxDistance;
    }

    return currentDistance/4;   // divide by 4 to keep range of 255 bits
  }
};