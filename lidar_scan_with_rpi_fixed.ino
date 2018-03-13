// Load sketch into Arduino software, available from:
// https://www.arduino.cc/
// This sketch requires library "LIDAR-Lite v3" by Garmin.
// Select menu "Sketch", "Include Library", "Manage Libraries...",
// and in textbox "Filter your search...", enter "lidar".

#include <Servo.h>
#include <LIDARLite.h>

Servo panservo;
Servo tiltservo;
LIDARLite lidar;

// Minimum and maximum servo angle in degrees
// Modify to avoid hitting limits imposed by pan/tilt bracket geometry
int minPanAngle = 0;
int maxPanAngle = 180;
int minTiltAngle = 60;
int maxTiltAngle = 180;

boolean scanning = false;
boolean firstrun = true;
int scanIncrement = 1;
int posPan = maxPanAngle;
int posTilt = minTiltAngle;
float pi = 3.14159265;
float deg2rad = pi / 180.0;
int currReading = 0;

void setup() {
  // set up lidar sensor
  lidar.begin(0, true);
  lidar.configure(0);

  // attach servos and set initial pos
  panservo.attach(8);
  tiltservo.attach(9);
  panservo.write(posPan);
  tiltservo.write(posTilt);
  
  Serial.begin(9600);
}

void loop() {
  if (Serial.available())
  {
    long xin = 0;
    long yin = 0;
    long zin = 0;
    int theta = -30;// rotation about x
    int phi = 0; // rotation about y
    int beta = 0; // rotation about z
    Serial.read(); // reads the rest of the Serial line, cleaning it out
    //Serial.println("Input: " + String(xin) + " " + String(yin) + " " + String(zin) + " " + String(theta) + " " + String(phi) + " " + String(beta));
    runScan(xin, yin, zin, theta, phi, beta);
    Serial.println("terminate scan");
    delay(100);
    serialFlush();
  }
}



void runScan(long dcpx, long dcpy, long dcpz, int dcptheta, int dcpphi, int dcpbeta){
  int xreading = 0;
  int yreading = 0;
  int zreading = 0;
  int alpha = dcptheta; //-30;
  int beta = -posPan+90;
  int gamma = posTilt-90;
  int l0 = 75;
  int l1 = 79;
  int dist = 0;
  for (posTilt = minTiltAngle; posTilt < maxTiltAngle; posTilt++){
    tiltservo.write(posTilt);

    // scan sweeping left
    for (posPan = maxPanAngle; posPan > minPanAngle; posPan--){
      panservo.write(posPan);
      if (posPan==maxPanAngle){
        delay(1000);
      }
      currReading = 10*lidar.distance(); // in mm
      dist = l1 + currReading;
      beta = -posPan+90;
      gamma = posTilt-90;
      xreading = - dist * sin(beta*deg2rad) * cos(gamma*deg2rad);
      yreading = l0 * cos(alpha*deg2rad) - dist * (-cos(alpha*deg2rad)*sin(gamma*deg2rad) - sin(alpha*deg2rad)*cos(beta*deg2rad)*cos(gamma*deg2rad)) ;
      zreading = l0*sin(alpha*deg2rad) - dist*(-sin(alpha*deg2rad)*sin(gamma*deg2rad) + cos(alpha*deg2rad) * cos(beta*deg2rad) * cos(gamma*deg2rad));
      Serial.println(String(xreading) + " " + String(yreading) + " " + String(zreading));
      delay(10);
    }
    
  }
  
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   

