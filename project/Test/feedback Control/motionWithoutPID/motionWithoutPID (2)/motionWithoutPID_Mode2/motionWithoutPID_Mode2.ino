#include <Servo.h>

const int servoPin[5] = {13,11,9,6,4};                              // define pins attached to servos
  
Servo myservo1;                               // build servo object
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

const int maxrow=5;
const int maxcol=58;                           // the first trajectory
int jointAngle[maxrow+1][maxcol+1]=
                        {
{0,100,200,300,400,500,600,700,800,900,1000,1000,1200,1400,1500,1550,1600,1650,1700,1750,1800,1850,1900,1950,2000,2100,3400,3500,3550,3600,3650,3700,3750,3800,3850,3900,3950,4000,4100,4200,4350,4400,4500,4600,4700,4800,4900,5000,5100,5200,5300,5400,5500,5600,5700,5800,5900,6000,8000},
{47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,51,60,65,65,65,65,62,57,53,55,61,65,62,57,53,55,61,65,62,57,53,53,51,50,48,47},
{22,22,22,23,24,26,27,28,29,29,29,29,34,44,50,54,64,70,66,56,50,54,64,70,66,56,50,54,64,70,66,56,50,50,58,71,68,67,70,69,66,65,66,68,70,69,66,65,66,68,70,69,66,65,60,48,35,25,22},
{110,109,107,103,98,93,88,84,80,78,77,77,82,94,101,107,122,132,125,109,101,107,122,132,125,109,101,107,122,132,125,109,101,110,140,173,159,139,132,129,126,124,125,129,132,129,126,124,125,129,132,129,126,124,120,113,109,109,110},
{38,39,42,46,52,58,64,70,74,78,79,79,78,76,75,74,68,64,67,73,75,74,68,64,67,73,75,74,68,64,67,73,75,66,44,24,35,54,64,65,67,67,67,66,64,65,67,67,67,66,64,65,67,67,66,61,52,42,38},
{0,5,19,39,63,90,117,141,161,175,180,0,37,127,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
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
