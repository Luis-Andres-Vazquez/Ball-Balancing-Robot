#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0


#define MPU6050_DEFAULT_ADDRESS     0x68 // address pin low (GND), default for InvenSense evaluation board
#define Three_Axis_Quaternions 3
#define Six_Axis_Quaternions 6  // Default

#define USE_TIMER_1 true

#include "Simple_MPU6050.h"
#include <String.h>
#include <SPI.h>
#include "TimerInterrupt.h"

double thetaX, thetaY; 
double integralX, integralY;
double lasterrX, lasterrY;

Simple_MPU6050 mpu(Three_Axis_Quaternions);

using namespace std;

class Utils{
public: 
  static double map(double min1, double max1, double min2, double max2, double val){
    double percent = (val - min2)/(max2 - min2);
    return min1 + (max1-min1)*percent;
  }
  static double bound(double val, double min, double max){
    if (val > max) return max; 
    if (val < min) return min; 
    return val;
  }
};

class StepperMotor{
public: 
    StepperMotor(int step_pin, int dir_pin){
      _step_pin = step_pin; 
      _dir_pin = dir_pin; 
      pinMode(_step_pin, OUTPUT);
      pinMode(_dir_pin, OUTPUT);
      _position = 0; 
      _delay = _MIN_DELAY;
      _last_step_time = 0; 
      _step_direction = 1;
      _speed = 0;
    }
    //speed from -1 to 1 
    void run(double speed){
      
      _direction = (speed < 0) ? 0 : 1;
      digitalWrite(_dir_pin, _direction);

      _speed = Utils::bound(abs(speed), 0.01, 1);
      //Serial.println(_speed);
      
      _delay = 250/_speed;
      //_delay = Utils::map(_MAX_DELAY, _MIN_DELAY, 0, 1, abs(_speed));
      //Serial.println((String)"delay: " +_delay + "speed: " + _speed);
      //Serial.println(_delay);

    }
    void timerHandler() { 
      _counter += _update_period;
      if (_counter < _delay) return;
      _counter = 0;
      step();
    }
    double getPosition(){
      return _position;  
    }
    void step() {
      //_position = (_step_direction == 0) ? _position - 0.5 : _position + 0.5; 
      if (_speed != 0){
        digitalWrite(_step_pin, !digitalRead(_step_pin));
      }
      
    }
    int _delay;
private:
    int _counter = 0; 
    int _update_period = 100; //microseconds 
    int _step_pin;
    int _dir_pin;
    int _MIN_DELAY = 250; //delay for speed uS 
    int _MAX_DELAY = 10000; //delay for speed 0, increase this is minimize lowest speed 
    
    int _position;
    int _direction; 
    int _last_step_time;
    bool _step_direction;  
    double _speed = 0;
};

const int dirA = 2;
const int dirB = 3;
const int dirC = 4;

const int stepPinA = 5; 
const int stepPinB = 6;
const int stepPinC = 7;

const int MS1A = 10;
const int MS2A = 9;
const int MS3A = 8;

const int MS1B = 13;
const int MS2B = 12;
const int MS3B = 11;

const int MS1C = 42;
const int MS2C = 44;
const int MS3C = 46;

bool MS1_VAL = HIGH;
bool MS2_VAL = HIGH;
bool MS3_VAL = LOW;

StepperMotor motorA(stepPinA, dirA); 
StepperMotor motorB(stepPinB, dirB); 
StepperMotor motorC(stepPinC, dirC);

int ticks = 0; 

void TimerHandler(){
  ticks++;
  //Serial.println("Stepping at " + (String)(millis()));
  motorA.timerHandler();
  motorB.timerHandler();
  motorC.timerHandler();
}

void IMUHandler (int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);
  thetaX = xyz[2];
  thetaY = -xyz[1];
  // thetaX = abs(thetaX)*abs(thetaX)*(thetaX/abs(thetaX));
  // thetaY = abs(thetaY)*abs(thetaY)*(thetaY/abs(thetaY));
  //Serial.println("thetaX: " + (String)thetaX + " thetaY: " + (String)thetaY);
  
}

void setup(){
  Serial.begin(9600);
  Serial.println(F("Start:"));

  // Setup the MPU
  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(100);          // Set the DMP output rate from 200Hz to 5 Minutes.
  //mpu.Set_DMP_Output_Rate_Seconds(10);   // Set the DMP output rate in Seconds
  //mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minute
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS); //Sets the address of the MPU.
  // Replace with your offsets
  //#define OFFSETS   -238,     540,    1616,     145,      33,      14
  #define OFFSETS   -234,     516,    1616,     146,      36,      14
  mpu.setOffset(OFFSETS);
  //mpu.CalibrateMPU();                    // Calibrates the MPU.
  mpu.CalibrateGyro();                     // Calibrates the Gyro Only.
  //mpu.CalibrateAccel();                  // Calibrates the Accel only.
  
  mpu.load_DMP_Image();                    // Loads the DMP image into the MPU and finish configuration.
  mpu.on_FIFO(IMUHandler);      

  pinMode(dirA,OUTPUT);
  pinMode(dirB,OUTPUT);
  pinMode(dirC,OUTPUT);

  pinMode(stepPinA,OUTPUT); 
  pinMode(stepPinB,OUTPUT); 
  pinMode(stepPinC,OUTPUT); 

  pinMode(MS1A,OUTPUT);
  pinMode(MS2A,OUTPUT);
  pinMode(MS3A,OUTPUT);

  pinMode(MS1B,OUTPUT);
  pinMode(MS2B,OUTPUT);
  pinMode(MS3B,OUTPUT);

  pinMode(MS1C,OUTPUT);
  pinMode(MS2C,OUTPUT);
  pinMode(MS3C,OUTPUT);

  //microstepping config
  digitalWrite(MS1A,MS1_VAL);
  digitalWrite(MS2A,MS2_VAL);
  digitalWrite(MS3A,MS3_VAL);

  digitalWrite(MS1B,MS1_VAL);
  digitalWrite(MS2B,MS2_VAL);
  digitalWrite(MS3B,MS3_VAL);

  digitalWrite(MS1C,MS1_VAL);
  digitalWrite(MS2C,MS2_VAL);
  digitalWrite(MS3C,MS3_VAL);

  ITimer1.init();

  //frequency is ticks per second (10000 => 100 uS delay)
  ITimer1.attachInterrupt(10000,  TimerHandler);

  Serial.println("Initializing complete");

}

double speed = 1;
int startTime = millis(); 

double kP = 0.31;
double kI = 0.00000;
double kD = 0.00;

double psi = 1.02; //0.87;//M_PI/4; //radians
double wz = 0.0;
double r = 0.11;
double Kz = -1*r*sin(psi);


void loop() {

  static unsigned long FIFO_DelayTimer;
  if ((millis() - FIFO_DelayTimer) >= (9)) { // 9ms instead of 10ms to start polling the MPU 1ms prior to data arriving.
    if( mpu.dmp_read_fifo(false)) FIFO_DelayTimer= millis() ; // false = no interrupt pin attachment required and When data arrives in the FIFO Buffer reset the timer
  }

  // thetaX = mpu.getAngleX();
  // thetaY = mpu.getAngleY(); 

  integralX += thetaX;
  integralY += thetaY;

  double derivativeX = thetaX - lasterrX;
  double derivativeY = thetaY - lasterrY;

  //thetaX - 3 is good for stability
  double vx = kP*(thetaX) + kI*integralX + kD*derivativeX;
  double vy = kP*thetaY + kI*integralY + kD*derivativeY;

  lasterrX = thetaX;
  lasterrY = thetaY;

  // if (abs(thetaX) < 1){
  //   vx = 0;
  // }

  // if (abs(thetaY) < 1) {
  //   vy = 0;
  // }

  double v1 = -1 * vy * cos(psi) + Kz*wz;
  double v2 = ((0.866)*vx + (0.5)*vy)*cos(psi) + Kz*wz;
  double v3 = (-1 * 0.866 *vx + (0.5)*vy)*cos(psi) + Kz*wz;

  // if(millis() - startTime > 500){
  //   startTime = millis(); 
  //   Serial.println("UPDATE: thetaX: " + (String)thetaX + " thetaY: " + (String)thetaY);
  //   // Serial.println("P " + (String)(kP*thetaX) + " I " + (String)(kI*integralX) + " D " + (String)(kD*derivativeX));
  //   // Serial.println("UPDATE: vy: " + (String)vx + " vx: " + (String)vy);
  //   // Serial.println("");
  // }

  motorA.run(v1); //1
  motorB.run(v2); //2
  motorC.run(v3); //3

}
