#include <Servo.h>
Servo myservo1;
Servo myservo2;

int pinFeedback1 = 9 ;
int pinFeedback2 = 11 ;

int pinServo1 = 5 ;
int pinServo2 = 3 ;

float angle1, angle2, turnAngle, startAngle;
int theta = 0, thetaP1 = 0, theta2 = 0, thetaP2 = 0;
unsigned long tHigh1 = 0, tLow1 = 0, tHigh2 = 0, tLow2 = 0;
float dc;
float dc2;
int tCycle1;
int tCycle2;
int rotAngle = 88;
int stopAngle = 90;
int turns1 = 0;
int turns2 = 0;
bool stopMovement = false;
const float Pi = 3.14159;
const float wheelRadius = 3.2;
float wheelPerimeter = 2 * Pi * wheelRadius;
int Kp = 10;
int unitsFC = 360;
float dcMin = 0.029; //29 duty cycle
float dcMax = 0.971; //971 duty cycle
float dutyScale = 1.0;
float q2min = unitsFC / 4.0;
float q3max = q2min * 3.0;
bool turn = false;
int turnDelay; 
int maxTurnAngle = 310;
int minTurnAngle = 300;
int radDegFactor = 57.2958;

float distanceToMove;
float angleToTurn;
int currentPosition = 0;


const int buttonPin = 2;     // the number of the pushbutton pin



void updateStatus(unsigned long& tHigh, int& tCycle, float& angle, int& thetaP, int& turns) {

  dc = (dutyScale * tHigh) / tCycle;//

  theta = ((dc - dcMin) * unitsFC) / (dcMax - dcMin); // full circle ,(dcMax - dcMin+1);

  if (theta < 0) theta = 0;
  else if (theta > (unitsFC - 1)) theta = unitsFC - 1;

  if ((theta < q2min) && (thetaP > q3max)) // If 4th to 1st quadrant
    turns++; // Increment turns count
  else if ((thetaP < q2min) && (theta > q3max)) // If in 1st to 4th quadrant
    turns --;// Decrement

  if (turns >= 0)
    angle = (turns * unitsFC) + theta;
  else if (turns1 < 0)
    angle = ((turns + 1) * unitsFC) - (unitsFC - theta);


  thetaP = theta;

}


void updateServo1Angle(float& angle){
  while (1){
     tHigh1 = pulseIn(pinFeedback1, 1);// Measure high pulse
     tLow1 = pulseIn(pinFeedback1, 0);// Measure low pulse

     tCycle1 = tHigh1 + tLow1;// Calculate cycle time
     if((tCycle1 > 1000) && (tCycle1 < 1200)) break; // Cycle time valid? Break!
       }
    
    updateStatus(tHigh1, tCycle1, angle, thetaP1, turns1);
    }


void updateServo2Angle(float& angle){
  while (1){
     tHigh2 = pulseIn(pinFeedback2, 1);// Measure high pulse
     tLow2 = pulseIn(pinFeedback2, 0);// Measure low pulse

     tCycle2 = tHigh2 + tLow2;// Calculate cycle time
     if((tCycle2 > 1000) && (tCycle2 < 1200)) break; // Cycle time valid? Break!
       }
    
    updateStatus(tHigh2, tCycle2, angle, thetaP2, turns2);
    }


void stopServo() {
  myservo1.writeMicroseconds(1500);
  myservo2.writeMicroseconds(1500);
}

void turnLeft() {
  myservo1.write(88);
  myservo2.write(88);
    
}

void turnRight() {
  myservo1.write(96);
  myservo2.write(96);
 
}


void turnLeft1() {
  myservo1.write(88);
  myservo2.write(90);
    
}

void turnRight1() {
  myservo1.write(90);
  myservo2.write(96);
 
}


void moveServo() {
  myservo1.write(96);
  myservo2.write(88);
}

void stopServo1() {
  myservo1.write(1500);
}

void advanceServo1() {
  myservo1.write(86);
}

void reduceServo1() {
  myservo1.write(89);
}




void setup() {
  Serial.begin(115200);
  pinMode(pinFeedback1, INPUT);
  pinMode(pinFeedback2, INPUT);
  myservo1.attach(pinServo1);
  myservo2.attach(pinServo2);
  //updateServo1Angle(startAngle);
  //updateServo2Angle(startAngle);

// moveServo();
// delay(5000);
 //stopServo();
  
}


void loop() {
  int error;
  int tCycle1 = 0;
  int tCycle2 = 0;

if(Serial.available() > 0) {
    //String data="0";
     String data = Serial.readStringUntil('\n');
    //Serial.print(" ");
     //Serial.println(data);

error=data.toInt();
Serial.print(error); 


if(error>40){ 
    stopServo();
    delay(20);
    turnLeft1();

}
else if(error<-40){
    stopServo();
    delay(20);
    turnRight1();

    
}

else {   
    moveServo();
}


    updateServo1Angle(angle1);
    updateServo2Angle(angle2);
}
}
