
#include <PIDx.h>    // control the servo motors joint angles to reach a desired configuration
#include <Servo.h>

/* This piece of code is designed to compute the PID controller
 *  needs to add import trajactory joint angles from the laptop
 */
 
//--------------------------------------------------------------------
const int maxrow = 5;                                      //(# of rows-1) of float array to store time and joint angles for 6 servos
const int maxcol = 80;                                     //(# of columns-1) of float array to store time and joint angles for 6 servos
double Kp[4] = {  1.15,   1.25,     1.2,    1.3};             //Kp, Kd, Ki for all controllers
double Kd[4] = {  0.0005,   0.0005,     0.0005,    0.0005};
double Ki[4] = {  0.005,   0.005,     0.005,    0.005};
//double Kp[4] = {  0,   0,     1.01,    0};             //Kp, Kd, Ki for all controllers
//double Kd[4] = {  0,   0,     0.0005,    0};
//double Ki[4] = {  0,   0,     0.005,    0};
int PIDSampleTime = 2000;

//--------------------------------------------------------------------
int jointAngle[maxrow+1][maxcol+1] =
{{0,6000,12000,18000,24000,30000,36000,42000,48000,54000,60000,66000,72000,78000,84000,90000,96000,102000,108000,114000,120000,126000,132000,138000,144000,150000,156000,162000,168000,174000,180000,186000,192000,198000,204000,210000,216000,222000,228000,234000,240000,2.460000e+05,252000,258000,264000,270000,276000,282000,288000,294000,300000,306000,312000,318000,324000,330000,336000,342000,348000,354000,360000,366000,372000,378000,384000,390000,396000,402000,408000,414000,420000,426000,432000,438000,444000,450000,456000,462000,468000,474000,480000},
{45,45,45,45,45,45,45,45,45,45,45,44,42,38,33,28,30,35,43,52,62,60,55,47,38,28,29,32,36,40,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45},
{14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,16,17,19,22,24,26,28,29,29,32,38,47,58,70,69,66,61,56,50,51,54,58,64,70,69,66,61,56,50,51,54,58,64,70,69,66,61,56,50,49,46,42,37,32,27,22,18,15,14},
{117,117,117,117,117,117,117,117,117,117,117,116,116,114,113,111,111,111,111,111,111,111,111,111,111,111,111,112,113,115,117,116,113,108,103,97,91,85,81,78,77,79,86,97,112,132,130,125,117,109,101,102,107,113,122,132,130,125,117,109,101,102,107,113,122,132,130,125,117,109,101,101,102,104,106,109,111,113,115,116,117},
{23,23,23,23,23,23,23,23,23,23,23,24,24,26,28,30,30,30,30,30,30,30,30,30,30,30,29,28,27,25,23,25,29,35,43,51,59,67,73,77,79,79,78,76,72,64,65,67,70,73,75,75,74,71,68,64,65,67,70,73,75,75,74,71,68,64,65,67,70,73,75,74,70,64,57,49,41,34,29,25,23},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,4,9,15,21,27,33,38,41,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42}};
//{{0,6000,12000,18000,24000},
//{45,45,45,45,45},
//{50,50,50,50,50},
//{100,105,110,115,120},
//{70,70,70,70,70},
//{0,0,0,0,0}};


boolean isReady = false;                                                    //indicate when one piece of trajectory is done
double slope[6]     = { 0.4768,  0.4868,  0.4926,   0.515,  0.5104, 0};     //parameter to convert encoder signal to degree
double intercept[6] = {-59.954, -58.956, -58.561, -58.764, -57.508, 0};     //parameter to convert encoder signal to degree
                                                                            //servo # 2,3,4,5,1 from base frame origin to end-effector

double feedback_before[maxrow][maxcol+1];
double feedback_after[maxrow][maxcol+1];

int initialAngle[5] = {jointAngle[1][0], jointAngle[2][0], jointAngle[3][0], jointAngle[4][0], jointAngle[5][0]};

   Servo myservo1;                 // construct servo object as myservo1 and so on
   Servo myservo2;
   Servo myservo3;
   Servo myservo4;
   Servo myservo5;

int servoPin[5]   = {13,11,9,6,4};           // set the pin connected to  1 and so on

int encoderPin[4] = {1,4,6,9};         
                                           // set the analog pin connected to encoder 1 and so on
double Setpoint1, Setpoint2, Setpoint3, Setpoint4;        // variables to setup PID controllers
double    Input1,    Input2,    Input3,    Input4; 
double   Output1,   Output2,   Output3,   Output4;

PIDx myPID1(&Input1, &Output1, &Setpoint1, Kp[0], Ki[0], Kd[0]);         // create PID control objects
PIDx myPID2(&Input2, &Output2, &Setpoint2, Kp[1], Ki[1], Kd[1]);
PIDx myPID3(&Input3, &Output3, &Setpoint3, Kp[2], Ki[2], Kd[2]);
PIDx myPID4(&Input4, &Output4, &Setpoint4, Kp[3], Ki[3], Kd[3]);

int pushButton = 22;  //digital pin num of button
boolean clicks = false;   //count on and off status of button

void setup() {
   myservo1.attach(servoPin[0]);                        // attaches the servo on servoPin and so on
   myservo2.attach(servoPin[1]);
   myservo3.attach(servoPin[2]);
   myservo4.attach(servoPin[3]);
   myservo5.attach(servoPin[4]);
  
   myservo1.write(initialAngle[0]);                     // intialize the servo, EDIT
   myservo2.write(initialAngle[1]);
   myservo3.write(initialAngle[2]);
   myservo4.write(initialAngle[3]);
   myservo5.write(initialAngle[4]);
  
   Input1 = analogRead(encoderPin[0]);                   // initialize the PID controller
   Input2 = analogRead(encoderPin[1]);
   Input3 = analogRead(encoderPin[2]);
   Input4 = analogRead(encoderPin[3]);
                                                          // let Setpoints have initial value, EDIT
   Setpoint1 = initialAngle[0];
   Setpoint2 = initialAngle[1];
   Setpoint3 = initialAngle[2];
   Setpoint4 = initialAngle[3];

   myPID1.SetSampleTime(PIDSampleTime);       // set PID controller's sample time in ms
   myPID2.SetSampleTime(PIDSampleTime);
   myPID3.SetSampleTime(PIDSampleTime);
   myPID4.SetSampleTime(PIDSampleTime);

   myPID1.SetOutputLimits(-60,60);
   myPID2.SetOutputLimits(-60,60);
   myPID3.SetOutputLimits(-60,60);
   myPID4.SetOutputLimits(-60,60);
   pinMode(pushButton, INPUT_PULLUP);
   
   Serial.begin(9600);
   
   delay(2000);

   
}

void loop() {
  // put your main code here, to run repeatedly:

  //button
  int buttonState = digitalRead(pushButton);
  //delay(200);
  if (buttonState == 0) {
    clicks = true;
  }
  
  if (clicks == true){
    Serial.println("Motors are running");
    
    if(!isReady){
      //delay(1000);                             // after that delay 1s

      for(int i = 1; i <=  maxcol; i++)         //  We need setpoints from [i][1] ~ [i][maxColumn]
      {

        servoController(jointAngle[1][i], jointAngle[2][i], jointAngle[3][i], jointAngle[4][i], jointAngle[5][i]-jointAnle[5][i-1]);
     
        myservo5.write(jointAngle[5][i]);
        
        delay(10);
      }
    
      isReady = true;
      
  }
  clicks = false;
  
  //delay(500);
  }
}

void servoController(int desiredPos1, int desiredPos2, int desiredPos3, int desiredPos4, int dt)
{         
  // dt = time difference between two setpoints
  Setpoint1 = desiredPos1;                          // set this time interval desired position
  Setpoint2 = desiredPos2;
  Setpoint3 = desiredPos3;
  Setpoint4 = desiredPos4;
  
  int add[4] = {0,0,0,0};
  int var[4] = {0,0,0,0};
  int analog1;
  int analog2;
  int analog3;
  int analog4;
  
  unsigned long beginTime = millis();                   // time when the controller function starts 
  unsigned long nowTime;                                // time which the controller has been on
  unsigned long timeInterval = 0;
 
  
  while(timeInterval<dt)                                // loop until the time is reached
  {
    
    delay(1000);                                                   // variable to store encoder analog signal   
    var[0] = analogRead(encoderPin[0]);                 
    analog1 = var[0] * slope[0] + intercept[0];         
    Serial.print(Setpoint1-analog1);   
    Serial.print(" "); 
    if(abs(Setpoint1-analog1) <20){
      Input1 = analog1;
      if(myPID1.Compute()){                             
        add[0] = (int)Output1;                            
        servoTurn1(add[0]);                                               
      }
    }

    var[1] = analogRead(encoderPin[1]);                 
    analog2 = var[1] * slope[1] + intercept[1];         
    Serial.print(Setpoint2-analog2);   
    Serial.print(" "); 
    if(abs(Setpoint2-analog2) <20){
      Input2 = analog2;
      if(myPID2.Compute()){                             
        add[1] = (int)Output2;                            
        servoTurn2(add[1]);                                               
      }
    }

    
    var[2] = analogRead(encoderPin[2]);                 
    analog3 = var[2] * slope[2] + intercept[2];
    Serial.print(Setpoint3-analog3);   
    Serial.print(" ");         
    if(abs(Setpoint3-analog3) < 20){
      Input3 = analog3;
      if(myPID3.Compute()){                             
        add[2] = (int)Output3;                            
        servoTurn3(add[2]);                                         
      }
    }
//

    var[3] = analogRead(encoderPin[3]);                 
    analog4 = var[3] * slope[3] + intercept[3]; 
    Serial.println(Setpoint4-analog4);         
    if(abs(Setpoint4-analog4) <20){
      Input4 = analog4;
      if(myPID4.Compute()){                             
        add[3] = (int)Output4;                            
        servoTurn4(add[3]);                                               
      }
    }
 
    delay(50);
    
    nowTime = millis();
    timeInterval = nowTime - beginTime;
  }
}


void servoTurn1(int addAngle){
  // turn the servo to an addaptive angle
  int oldAng = myservo1.read();                            // read the angle of last time
  int newAng = oldAng + addAngle;                        // compute the new angle
  myservo1.write(newAng);                                  // turn the servo to the new position
}

void servoTurn2(int addAngle){
  int oldAng = myservo2.read();                            
  int newAng = oldAng + addAngle;                         
  myservo2.write(newAng);                               
}

void servoTurn3(int addAngle){
  int oldAng = myservo3.read();                            
  int newAng = oldAng + addAngle;                        
  myservo3.write(newAng);                               
}

void servoTurn4(int addAngle){
  int oldAng = myservo4.read();                          
  int newAng = oldAng + addAngle;                  
  myservo4.write(newAng);                               
}

