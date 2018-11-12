#include <Servo.h>

const int servoPin[5] = {13,11,9,6,4};                              // define pins attached to servos
  
Servo myservo1;                               // build servo object
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

const int maxrow=5;
const int maxcol=31;                           // the first trajectory
int jointAngle[maxrow+1][maxcol+1]=
                        {
                          { 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100},
{47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47},
{22,22,22,23,24,26,27,28,29,29,29,29,34,44,50,54,64,70,66,56,50,54,64,70,66,56,50,54,64,70,66,56},
{110,109,107,103,98,93,88,84,80,78,77,77,82,94,101,107,122,132,125,109,101,107,122,132,125,109,101,107,122,132,125,109},
{37,38,41,45,51,57,63,69,73,77,78,78,77,75,74,73,67,63,66,72,74,73,67,63,66,72,74,73,67,63,66,72},
{0,5,19,39,63,90,117,141,161,175,180,0,37,127,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

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
  delay(2000);

  /*
  for(int i = 1; i<= maxcol; i++)
  {
    int dt = jointAngle[0][i]-jointAngle[0][i-1];
    myservo1.write(jointAngle[1][i]);           // move servos seperately
                                    // delay a const time to let servo move
    myservo2.write(jointAngle[2][i]);
   
    myservo3.write(jointAngle[3][i]);
    
    myservo4.write(jointAngle[4][i]);
   
    myservo5.write(jointAngle[5][i]);
    
    delay(dt);
  }
  */
 

  Serial.begin(9600);
  pinMode(pushButton, INPUT_PULLUP);
}

void loop() {

  //button
  int buttonState = digitalRead(pushButton);
  delay(200);
  if (buttonState == 0) {
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
    clicks = true;
  }
  if (clicks == true){
    Serial.println("Motors are running");

  
  for(int i = 1; i<= maxcol; i++)
  {
    int dt = jointAngle[0][i]-jointAngle[0][i-1];
    myservo1.write(jointAngle[1][i]);           // move servos seperately
                                    // delay a const time to let servo move
    myservo2.write(jointAngle[2][i]);
   
    myservo3.write(jointAngle[3][i]);
    
    myservo4.write(jointAngle[4][i]);
   
    myservo5.write(jointAngle[5][i]);
    
    delay(dt);
  }
   clicks = false;
  }

}
