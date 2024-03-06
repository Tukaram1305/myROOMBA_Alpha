/*Markowiak Pawel
klasa dla zarzadzania motorami na mostku typu H*/
#ifndef ENGINE_H
#define ENGINE_H
#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>
#include <DFRobot_BMI160.h>

extern int16_t gyroData[3];
extern bool revInvert, revInvert_p;
extern bool RisBlck, RisBlck_p, LisBlck, LisBlck_p;

class Engine {
public:
 //  Lewy mot.   Prawy mot.
 // |FORW|BACK| |FORW|BACK| 
 // |BACK|FORW| |BACK|FORW|
Engine (); 
~Engine();

void motBegin (int LForw, int LBack, int RForw, int RBack, Adafruit_PWMServoDriver *adaPwmPtr, DFRobot_BMI160 *BMI);
void drive(byte LV, byte RH );
void stopMot();
void chngGear();
byte giveCGear();
void turnOnPID();
double calcPID(double tInterval, double setpoint, double procVar);
void giveLastGYRO(int16_t gy[]);
private:
Adafruit_PWMServoDriver *pwmptr = nullptr;
DFRobot_BMI160 *bmi = nullptr;
int LfPin, LbPin, RfPin, RbPin;
byte LV_p, RH_p;
int sterL, sterR;
int spdL, spdR;
//        defaultowe biegi    1     2     3     4
const uint16_t GEARS[4] = { 1024, 2048, 3072, 3800 };
byte cGEAR = 3;

// limity - histereza galek analogowych
const int lAnaMin = 120;
              // MID 132
const int lAnaMax = 144;
//--^Lvert------vRhor---
const int rAnaMin = 119;
              // MID 129
const int rAnaMax = 139;

const int AnaUPLim = 250; // gorny trshold
const int AnaDWLim = 5;   // dolny trshold

// PID vals
int16_t GYRO[3]{0,0,0};
bool PIDon = false;
uint16_t PIDMOT = 0;
double _max{4095}, _min{0};
double _Kp{0.1}, _Kd{0.01}, _Ki{0.5};
double _error_prev{0};
double _integral{0};
double SETPOINT = 0;
};

#endif
