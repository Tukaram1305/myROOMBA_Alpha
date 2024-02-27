#ifndef ENGINE_H
#define ENGINE_H
#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>

extern bool revInvert, revInvert_p;
extern bool RisBlck, RisBlck_p, LisBlck, LisBlck_p;

class Engine {
public:
 //  Lewy mot.   Prawy mot.
 // |FORW|BACK| |FORW|BACK| 
 // |BACK|FORW| |BACK|FORW|
Engine ();  /* bazowy konstruktor*/ 
~Engine();

void motBegin (int LForw, int LBack, int RForw, int RBack, Adafruit_PWMServoDriver *adaPwmPtr);
void drive(byte LV, byte RH );
void stopMot();
void chngGear();
byte giveCGear();

private:
Adafruit_PWMServoDriver *pwmptr = nullptr;
int LfPin, LbPin, RfPin, RbPin;
byte LV_p, RH_p;
int sterL, sterR;
int spdL, spdR;
//        defaultowe biegi    1     2     3     4
const uint16_t GEARS[4] = { 1024, 2048, 3072, 4095 };
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
};

#endif
