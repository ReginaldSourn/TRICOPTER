#include <Arduino.h>
#include "TeensyThreads.h"
// SERVO FOR testing Check how 
#include <Servo.h>
#include <Wire.h>
//#include <Adafruit_MPU6050.h>

#include "MPU6050_light.h"

#define S1 3
#define S2 4
#define S3 5
#define S4 6
#define BFR 9
#define BFL 10
#define BBM 23



Servo ser1, ser2, ser3, ser4, brushFR, brushFL, brushBM;

#define SRC_NEUTRAL 1500
#define SRC_MAX 2000
#define SRC_MIN 1000
#define TRC_NEUTRAL 1500
#define TRC_MAX 2000
#define TRC_MIN 1000
#define RC_DEADBAND 50
#define ERROR_center 50
#define pERROR 100




#define THROTTLE_FLAG 1
#define YAW_FLAG 2
#define ROLL_FLAG 3
#define PITCH_FLAG 4

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;


volatile uint16_t unThrottleInShared;
volatile uint16_t unRollInShared;
volatile uint16_t unYawInShared;
volatile uint16_t unPitchInShared;
//uint16_t unThrottleInShared;
//uint16_t unRollInShared;
//uint16_t unYawInShared;
//uint16_t unPitchInShared; 

float elapsedTime,time, timePrev;

uint16_t unSteeringMin = SRC_MIN + pERROR;
uint16_t unSteeringMax = SRC_MAX - pERROR;
uint16_t unSteeringCenter = SRC_NEUTRAL;

uint16_t unYawMin = TRC_MIN + pERROR;
uint16_t unYawMax = TRC_MAX - pERROR;
uint16_t unYawCenter = TRC_NEUTRAL;

uint16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;

uint16_t unRollMin = SRC_MIN + pERROR;
uint16_t unRollMax = SRC_MAX - pERROR;
uint16_t unRollCenter = SRC_NEUTRAL;

uint16_t unPitchMin = TRC_MIN + pERROR;
uint16_t unPitchMax = TRC_MAX - pERROR;
uint16_t unPitchCenter = TRC_NEUTRAL;

uint16_t ulThrottleStart, ulYawStart, ulPitchStart, ulRollStart;

#define THROTTLE_IN 12
#define YAW_IN  11
#define ROLL_IN 8
#define PITCH_IN 7

#define THROTTLE_FLAG 1
#define YAW_FLAG 2
#define ROLL_FLAG 3
#define PITCH_FLAG 4
#define PWM_MIN 0
#define PWM_MAX 2000
unsigned int ser1val ,ser2val, ser3val , ser4val, bfrVal, bflVal, bbmVal;
// holds the update flags defined above
//volatile uint8_t bUpdateFlagsShared;

int ThrottleVal, PitchVal, YawVal, RollVal;


float errorR = 0;
float errorP = 0;
float errorY = 0;

float previous_errorR =0;
float previous_errorP = 0;
float previous_errorY = 0;


float RollPID,PitchPID,YawPID, pwmleft, pwmRight, error, previous_error;



float pidR_p=0;
float pidR_i=0;
float pidR_d=0;



float pidP_p=0;
float pidP_i=0;
float pidP_d=0;

float pidY_p=0;
float pidY_i=0;
float pidY_d=0;




///////////////PID Constants ////////////////
double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05
/////////////////////////////////////////////

float angleX, angleY, angleZ;

double throttle=1300; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady
 
MPU6050 mpu(Wire);
unsigned long timer = 0;                   
void setup() {
  // put your setup code here, to run once:
  //   pinMode
  Wire.begin();
  Serial.begin(250000);

  
  ser1.attach(S1);
  ser2.attach(S2);
  ser3.attach(S3);
  ser4.attach(S4);
  brushFR.attach(BFR, 1000,2000);
  brushFL.attach(BFL, 1000,2000);
  brushBM.attach(BBM, 1000,2000);
  pinMode(THROTTLE_IN, INPUT);
  pinMode(YAW_IN, INPUT);
  pinMode(ROLL_IN, INPUT);
  pinMode(PITCH_IN, INPUT);

  // Calibration
  Serial.println("Calibrating gyro, place on level surface and do not move.");
 
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // Thread Initials
  threads.addThread(calcThrottleandYaw);
//  threads.addThread(calcYaw);
  delay(100);
  threads.addThread(calcRollandPitch);
  delay(100);
//  threads.addThread(calcRoll);
 //initial setup brushless motor 
  brushFR.writeMicroseconds(1000);
  brushFL.writeMicroseconds(1000);
  brushBM.writeMicroseconds(1000);

}

void loop() {
/*  
// put your main code here, to run repeatedly: 
//   unThrottleInShared =  pulseIn(THROTTLE_IN, HIGH, 10000);
//  unRollInShared =  pulseIn(ROLL_IN, HIGH, 30000);
//  unYawInShared =  pulseIn(YAW_IN, HIGH, 10000);
//   unPitchInShared =  pulseIn(PITCH_IN, HIGH, 10000);
//ser1val = map(unThrottleInShared,1100,1900,0,250);
//  ser2val = map(unYawInShared,1100,1900,0,250);
//  ser3val = map(unRollInShared,1100,1900,0,250);
//  ser4val = map(unPitchInShared,1100,1900,0,250);
//  ser1.write(ser1val);
//  ser2.write(ser2val);
//  ser3.write(ser3val);
//  ser4.write(ser4val);
*/
  mpu.update();
  elapsedTime = (time - timePrev) / 1000; 
  if((millis()-timer)>10){ // print data every 10ms
//  Serial.print("X : ");
//  Serial.print(mpu.getAngleX());
//  Serial.print("\tY : ");
//  Serial.print(mpu.getAngleY());
//  Serial.print("\tZ : ");
//  Serial.println(mpu.getAngleZ());
  angleX = mpu.getAngleX();
  angleY = mpu.getAngleY();
  angleZ = mpu.getAngleZ();
  timer = millis();  
    
  }
  
  // ------------ PID ROLL---------------------

  // angle x 
  errorR = angleX - desired_angle;
  pidR_p = kp*error;
  if(-3 < error){
    pidR_i = pidR_i+(ki*error);
  }
  pidR_d = kd *((error - previous_errorR)/elapsedTime);
  RollPID = pidR_p + pidR_i + pidR_d;
  // ------------------------------------------
  

  // ------------ PID PITCH---------------------

  // angle y 
  errorP = angleY - desired_angle;
  pidP_p = kp*error;
  if(-3 < error){
    pidR_i = pidP_i+(ki*error);
  }
  pidP_d = kd *((error - previous_errorR)/elapsedTime);
  PitchPID = pidP_p + pidP_i + pidP_d;
  // ------------------------------------------
  
  // ------------ PID YAW---------------------

  // angle x 
  errorP = angleX - desired_angle;
  pidR_p = kp*error;
  if(-3 < error){
    pidR_i = pidR_i+(ki*error);
  }
  pidR_d = kd *((error - previous_errorR)/elapsedTime);
  RollPID = pidR_p + pidR_i + pidR_d;
  // ------------------------------------------
  
  
  bfrVal = map(unThrottleInShared,1100,1900,1000,2000);
  bflVal = map(unThrottleInShared,1100,1900,1000,2000);
  bbmVal = map(unThrottleInShared,1100,1900,1000,2000); 
  brushFR.writeMicroseconds(bfrVal);
  brushFL.writeMicroseconds(bflVal);
  brushBM.writeMicroseconds(bbmVal);
  
  
}




// simple interrupt service routine
void calcThrottleandYaw()
{

//  delay(250);
while(1){

//  delay(250);
  ThrottleVal =  pulseIn(THROTTLE_IN, HIGH, 15000);
  YawVal =  pulseIn(YAW_IN, HIGH, 15000);


  if ((ThrottleVal < TRC_MAX) &&  (ThrottleVal > TRC_MIN)){
    unThrottleInShared = ThrottleVal;
    
  }
  if ((YawVal < TRC_MAX) &&  (YawVal > TRC_MIN)) {
    unYawInShared = YawVal;
  }
}
}



void calcRoll()
{
  while(1){

//  delay(250);
  unRollInShared =  pulseIn(ROLL_IN, HIGH, 30000);
//  delay(100);
  }
}

void calcRollandPitch()
{
  while(1){
  RollVal =  pulseIn(ROLL_IN, HIGH, 15000);
  PitchVal =  pulseIn(PITCH_IN, HIGH, 15000);


  if ((RollVal < TRC_MAX) &&  (RollVal > TRC_MIN)){
    unRollInShared = RollVal;
    
  }
  if ((PitchVal < TRC_MAX) &&  (PitchVal > TRC_MIN)) {
    unPitchInShared = PitchVal;
  }
}
}
