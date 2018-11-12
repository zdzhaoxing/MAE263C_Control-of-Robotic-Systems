#include <Servo.h>
#include <math.h>

const int servoPin[5]   = {13,11,9,6,4};   // define pins attached to servos
const int encoderPin[4] = {1,4,6,9};
Servo myservo1;                               // build servo object
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

const int maxrow=5;
const int maxcol=80;                           // the first trajectory
int jointAngle[maxrow+1][maxcol+1]=
                        {{0,100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,3300,3400,3500,3600,3700,3800,3900,4000,4100,4200,4300,4400,4500,4600,4700,4800,4900,5000,5100,5200,5300,5400,5500,5600,5700,5800,5900,6000,6100,6200,6300,6400,6500,6600,6700,6800,6900,7000,7100,7200,7300,7400,7500,7600,7700,7800,7900,8000},
{45,45,45,45,45,45,45,45,45,45,45,44,42,38,33,28,30,35,43,52,62,60,55,47,38,28,29,32,36,40,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45},
{14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,16,17,19,22,24,26,28,29,29,32,38,47,58,70,69,66,61,56,50,51,54,58,64,70,69,66,61,56,50,51,54,58,64,70,69,66,61,56,50,49,46,42,37,32,27,22,18,15,14},
{117,117,117,117,117,117,117,117,117,117,117,116,116,114,113,111,111,111,111,111,111,111,111,111,111,111,111,112,113,115,117,116,113,108,103,97,91,85,81,78,77,79,86,97,112,132,130,125,117,109,101,102,107,113,122,132,130,125,117,109,101,102,107,113,122,132,130,125,117,109,101,101,102,104,106,109,111,113,115,116,117},
{23,23,23,23,23,23,23,23,23,23,23,24,24,26,28,30,30,30,30,30,30,30,30,30,30,30,29,28,27,25,23,25,29,35,43,51,59,67,73,77,79,79,78,76,72,64,65,67,70,73,75,75,74,71,68,64,65,67,70,73,75,75,74,71,68,64,65,67,70,73,75,74,70,64,57,49,41,34,29,25,23},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,4,9,15,21,27,33,38,41,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42}};

double slope[6]     = { 0.4768,  0.4868,  0.4926,   0.515,  0.5104, 0};     //parameter to convert encoder signal to degree
double intercept[6] = {-59.954, -58.956, -58.561, -58.764, -57.508, 0};     //parameter to convert encoder signal to degree
                                                                            //servo # 2,3,4,5,1 from base frame origin to end-effector
double feedback_before[maxrow][maxcol+1];
double feedback_after[maxrow][maxcol+1];


int pushButton = 22;  //digital pin num of button
boolean clicks = false;   //count on and off status of button

void setup() {
  // put your setup code here, to run once:
  
  myservo1.attach(servoPin[0]);
  myservo2.attach(servoPin[1]);
  myservo3.attach(servoPin[2]);
  myservo4.attach(servoPin[3]);
  myservo5.attach(servoPin[4]);
  
  myservo1.write(jointAngle[1][0]);           // initialize servos
  myservo2.write(jointAngle[2][0]);
  myservo3.write(jointAngle[3][0]);
  myservo4.write(jointAngle[4][0]);
  myservo5.write(jointAngle[5][0]);

  Serial.begin(9600);
  pinMode(pushButton, INPUT_PULLUP);
  
  delay(2000);

}

void loop() {

  //button
  int buttonState = digitalRead(pushButton);
  delay(200);

  if (buttonState == 0) {
    clicks = true;
  }
  if (clicks == true){
    Serial.println("Motors are running");
    
  
    for(int i = 1; i<= maxcol; i++)
  {
//    int dt = jointAngle[0][i]-jointAngle[0][i-1];
    myservo1.write(jointAngle[1][i]);           // move servos seperately
//    delay(10);
//    feedback_before[0][i] = feedback1();
//    delay(10);
//    feedback_after[0][i] = analogRead(encoderPin[0]) * slope[0] - intercept[0];
    
    // delay a const time to let servo move
    myservo2.write(jointAngle[2][i]);
//    delay(90);
//    feedback_before[1][i] = feedback2();
//    delay(10);
//    feedback_after[1][i] = analogRead(encoderPin[1]) * slope[1] - intercept[1];
    
    myservo3.write(jointAngle[3][i]);
//    delay(90);
//    feedback_before[2][i] = feedback3();
//    delay(10);
//    feedback_after[2][i] = analogRead(encoderPin[2]) * slope[2] - intercept[2];
    
    myservo4.write(jointAngle[4][i]);
//    delay(90);
//    feedback_before[3][i] = feedback4();
//    delay(10);
//    feedback_after[3][i] = analogRead(encoderPin[3]) * slope[3] - intercept[3];
    
    myservo5.write(jointAngle[5][i]);
    delay(90);

    feedback_before[0][i-1] = feedback1();
    feedback_before[1][i-1] = feedback2();
    feedback_before[2][i-1] = feedback3();
    feedback_before[3][i-1] = feedback4();
    delay(10);
    feedback_after[0][i-1] = analogRead(encoderPin[0]) * slope[0] + intercept[0];
    feedback_after[1][i-1] = analogRead(encoderPin[1]) * slope[1] + intercept[1];
    feedback_after[2][i-1] = analogRead(encoderPin[2]) * slope[2] + intercept[2];
    feedback_after[3][i-1] = analogRead(encoderPin[3]) * slope[3] + intercept[3];
    
  }

   delay(2000);
   
   Serial.println("feedback before:");
   for(int i = 0; i < maxrow-1; i++){
      for(int j = 1; j < maxcol; j++){
          Serial.print(feedback_before[i][j]);
          if(j < maxcol-1){
              Serial.write(' ');
          }
          else{
              Serial.write('\n');
          }
      }
   }

    
  Serial.println("feedback after:");
   for(int i = 0; i < maxrow-1; i++){
      for(int j = 1; j < maxcol; j++){
        Serial.print(feedback_after[i][j]);
          if(j < maxcol-1){
              Serial.write(' ');
          }
          else{
              Serial.write('\n');
          }
      }
   }
  
  }
  clicks = false;


}

double feedback1(){
  int reference;
  double aSignal;
  double difference;
  int compensation;
  aSignal = analogRead(encoderPin[0]) * slope[0] + intercept[0];
  reference = myservo1.read();
  difference = aSignal - reference;
  if(abs(difference)<5){
    compensation = int(abs(difference));
     if(difference > 0)
        myservo1.write(reference + compensation);
     else
        myservo1.write(reference - compensation);
  }

  return aSignal;
}

double feedback2(){
  int reference;
  double aSignal;
  double difference;
  int compensation;
  aSignal = analogRead(encoderPin[1]) * slope[1] + intercept[1];
  reference = myservo2.read();
  difference = aSignal - reference;
  if(abs(difference)<5){
     compensation = int(abs(difference));
     if(difference > 0)
        myservo2.write(reference + compensation);
     else
        myservo2.write(reference - compensation);
  }
  return aSignal;
}

double feedback3(){
  int reference;
  double aSignal;
  double difference;
  int compensation;
  aSignal = analogRead(encoderPin[2]) * slope[2] + intercept[2];
  reference = myservo3.read();
  difference = aSignal - reference;
  if(abs(difference)<5){
    compensation = int(abs(difference));
     if(difference > 0)
        myservo3.write(reference + compensation);
     else
        myservo3.write(reference - compensation);
  }
  return aSignal;
}

double feedback4(){
  int reference;
  double aSignal;
  double difference;
  int compensation;
  aSignal = analogRead(encoderPin[3]) * slope[3] + intercept[3];
  reference = myservo4.read();
  difference = aSignal - reference;
  if(abs(difference)<5){
    compensation = int(abs(difference));
     if(difference > 0)
        myservo4.write(reference + compensation);
     else
        myservo4.write(reference - compensation);
  }
  return aSignal;
}



