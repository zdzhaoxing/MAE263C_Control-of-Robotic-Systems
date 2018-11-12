#include <Servo.h>

const int servoPin[5]   = {13,11,9,6,4};   // define pins attached to servos
const int encoderPin[4] = {1,4,6,9};
Servo myservo1;                               // build servo object
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

const int maxrow=5;
const int maxcol=125;                           // the first trajectory
int jointAngle[maxrow+1][maxcol+1]=
                        {{0,100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,3300,3400,3500,3600,3700,3800,3900,4000,4100,4200,4300,4400,4500,4600,4700,4800,4900,5000,5100,5200,5300,5400,5500,5600,5700,5800,5900,6000,6100,6200,6300,6400,6500,6600,6700,6800,6900,7000,7100,7200,7300,7400,7500,7600,7700,7800,7900,8000,8100,8200,8300,8400,8500,8600,8700,8800,8900,9000,9100,9200,9300,9400,9500,9600,9700,9800,9900,10000,10100,10200,10300,10400,10500,10600,10700,10800,10900,11000,11100,11200,11300,11400,11500,11600,11700,11800,11900,12000,12100,12200,12300,12400,12500},
{45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,46,47,49,51,54,57,59,62,63,63,63,62,61,60,59,59,58,56,55,53,53,54,55,57,59,59,58,56,55,53,53,54,55,57,59,59,59,60,61,61,62,62,63,63,63,62,56,49,40,31,31,32,34,35,37,37,36,35,33,31,31,32,34,35,37,37,36,35,33,31,31,32,34,36,38,40,42,44,45,45},
{14,14,14,14,14,14,14,14,14,14,14,15,16,18,21,24,27,30,32,34,34,35,37,41,45,50,51,54,58,64,70,69,66,61,56,50,51,54,58,64,70,69,66,61,56,50,50,51,52,54,56,57,59,60,61,61,62,64,66,70,74,74,76,78,81,84,84,82,79,76,74,74,76,78,81,84,84,82,79,76,74,73,72,71,69,67,66,64,62,61,61,61,61,62,66,74,74,76,78,81,84,84,82,79,76,74,74,76,78,81,84,84,82,79,76,74,72,68,61,53,44,35,27,20,16,14},
{117,117,117,117,117,117,117,117,117,117,117,116,115,113,110,107,104,102,100,98,98,98,98,98,99,101,102,107,113,122,132,130,125,117,109,101,102,107,113,122,132,130,125,117,109,101,102,104,108,112,117,121,126,129,132,133,133,133,134,136,138,139,142,145,151,157,156,153,148,143,138,139,142,145,151,157,156,153,148,143,138,138,138,137,136,135,135,134,133,133,133,131,129,127,130,138,139,142,145,151,157,156,153,148,143,138,139,142,145,151,157,156,153,148,143,138,138,136,134,131,128,124,121,119,117,117},
{23,23,23,23,23,23,23,23,23,23,23,24,27,32,37,43,49,54,58,61,62,63,65,68,72,75,75,74,71,68,64,65,67,70,73,75,75,74,71,68,64,65,67,70,73,75,75,73,71,68,65,62,59,57,55,54,55,56,58,60,61,61,60,58,56,53,54,55,57,59,61,61,60,58,56,53,54,55,57,59,61,61,61,60,59,58,57,56,55,55,54,55,58,61,62,61,61,60,58,56,53,54,55,57,59,61,61,60,58,56,53,54,55,57,59,61,60,57,53,48,42,37,31,27,24,23},
{0,0,0,0,0,0,0,0,0,0,0,1,4,9,15,21,27,33,38,41,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,43,46,51,57,63,69,75,80,83,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,82,75,66,54,42,30,18,9,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,4,9,15,21,27,33,38,41,42}};

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

  /*
  for(int i = 1; i<= maxcol; i++)
  {
    int dt = jointAngle[0][i]-jointAngle[0][i-1];
    myservo1.write(jointAngle[1][i]);           // move servos seperately
    delay(dt/5);
    feedback1();
    
    // delay a const time to let servo move
    myservo2.write(jointAngle[2][i]);
    delay(dt/5);
    feedback2();
    
    myservo3.write(jointAngle[3][i]);
    delay(dt/5);
    feedback3();
    
    myservo4.write(jointAngle[4][i]);
    delay(dt/5);
    feedback4();
    
    myservo5.write(jointAngle[5][i]);
    delay(dt/5);
  }
  */
}

void loop() {

  //button
  int buttonState = digitalRead(pushButton);
  delay(200);

  if (buttonState == 0) {
    clicks = true;
    /*
    if (clicks == 0) {
      clicks++;
//      Serial.println(clicks);
      //Serial.println("Robot arm is working");
    }
    else {
      clicks--;
//      Serial.println(clicks);
      Serial.println("Stand by");
    } 
    */
  }
  if (clicks == true){
    Serial.println("Motors are running");
    
  
    for(int i = 1; i<= maxcol; i++)
  {
//    int dt = jointAngle[0][i]-jointAngle[0][i-1];
    myservo1.write(jointAngle[1][i]);           // move servos seperately
//    delay(dt/5);
//    feedback_before[0][i] = feedback1();
//    feedback_after[0][i] = analogRead(encoderPin[0]) * slope[0] - intercept[0];
    
    // delay a const time to let servo move
    myservo2.write(jointAngle[2][i]);
//    delay(dt/5);
//    feedback_before[1][i] = feedback2();
//    feedback_after[1][i] = analogRead(encoderPin[1]) * slope[1] - intercept[1];
    
    myservo3.write(jointAngle[3][i]);
//    delay(dt/5);
//    feedback_before[2][i] = feedback3();
//    feedback_after[2][i] = analogRead(encoderPin[2]) * slope[2] - intercept[2];
    
    myservo4.write(jointAngle[4][i]);
//    delay(dt/5);
//    feedback_before[3][i] = feedback4();
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
      for(int j = 0; j < maxcol; j++){
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
      for(int j = 0; j < maxcol; j++){
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
