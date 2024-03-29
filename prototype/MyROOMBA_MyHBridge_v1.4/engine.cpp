#include "engine.h"

Engine::Engine() {}
Engine::~Engine(){ this->pwmptr=nullptr; }
void Engine::motBegin(int LForw, int LBack, int RForw, int RBack, Adafruit_PWMServoDriver *adaPwmPtr, DFRobot_BMI160 *BMI)
{
    pwmptr = adaPwmPtr;
    bmi = BMI;
    LfPin=LForw;
    LbPin=LBack;
    RfPin=RForw;
    RbPin=RBack;
    pwmptr->setPWM(LfPin, 0, 0);
    pwmptr->setPWM(LbPin, 0, 0);
    pwmptr->setPWM(RfPin, 0, 0);
    pwmptr->setPWM(RbPin, 0, 0);
}

void Engine::drive(byte LV, byte RH )
{
  if (LV_p != LV || RH_p != RH)
  {
      // Przelicz kierunek
      if (RH < rAnaMin)
      {
        if (RH < AnaDWLim) sterL = GEARS[cGEAR];
        else sterL = map(RH, rAnaMin-1, AnaDWLim, 0, GEARS[cGEAR]);
        sterR = 0;
      }
      else if (RH > rAnaMax)
      {
        sterL = 0;
        if (RH > AnaUPLim) sterR = GEARS[cGEAR];
        else sterR = map(RH, rAnaMax+1, AnaUPLim, 0, GEARS[cGEAR]);
      }
      else { sterL = 0; sterR = 0; }


      // Do przodu/tylu
      if (LV > lAnaMax) // >50% -> FORWARD
      {
        LAST_DIR = 1;
        pwmptr->setPWM(LbPin, 0, 0); // saveguard - nigdy Lf i Lb ON
        pwmptr->setPWM(RbPin, 0, 0);
        spdL = GEARS[cGEAR];
        spdR = GEARS[cGEAR];
        
        spdL = spdL - sterL;    // skret L/P
        if (spdL < 0) spdL = 0;
        spdR = spdR - sterR;
        if (spdR < 0) spdR = 0;

        if (LisBlck==false && RisBlck==false)
        {
          if (PIDon==true && sterL==0 && sterR==0)
          {
            bmi->getGyroData(GYRO);
            double DIF = calcPID(0.1, SETPOINT, double(GYRO[2]), mode_PI );
            PIDMOT = static_cast<uint16_t>(round(DIF));

            pwmptr->setPWM(LfPin, 0, spdL );
            pwmptr->setPWM(RfPin, 0, PIDMOT );
          }

          else
          {
          pwmptr->setPWM(LfPin, 0, spdL );
          pwmptr->setPWM(RfPin, 0, spdR );
          }
        }

        // +50% FORWARD + samoskrecanie przy blokadach
        else if (LisBlck==true && RisBlck==false) // L BLCK
        {
          pwmptr->setPWM(RfPin, 0, 0);          
          pwmptr->setPWM(RbPin, 0, GEARS[2]);

          pwmptr->setPWM(LbPin, 0, 0);          
          pwmptr->setPWM(LfPin, 0, GEARS[3]);
        }
        else if (RisBlck==true && LisBlck==false) // R BLCK
        {
          pwmptr->setPWM(RbPin, 0, 0);          
          pwmptr->setPWM(RfPin, 0, GEARS[3]);

          pwmptr->setPWM(LfPin, 0, 0);          
          pwmptr->setPWM(LbPin, 0, GEARS[2]);
        }
        
        else
        {
          pwmptr->setPWM(LfPin, 0, 0 );
          pwmptr->setPWM(RfPin, 0, 0 );
        }

      } // wychylenie galki do przodu
      
      else if (LV < lAnaMin) // <50% -> REVERSE
      {
        LAST_DIR = 2;
        pwmptr->setPWM(LfPin, 0, 0);
        pwmptr->setPWM(RfPin, 0, 0);

        spdL = GEARS[cGEAR];
        spdR = GEARS[cGEAR];

        if (revInvert==false)
        {
          spdL = spdL - sterL;    // skret L/P - normalnie
          if (spdL < 0) spdL = 0;
          spdR = spdR - sterR;
          if (spdR < 0) spdR = 0;
        }
        else
        {
          spdL = spdL - sterR;    // skret L/P - odwocony
          if (spdL < 0) spdL = 0;
          spdR = spdR - sterL;
          if (spdR < 0) spdR = 0;
        }
        if (PIDon==true && sterL==0 && sterR==0)
        {
          bmi->getGyroData(this->GYRO);
          double DIF = calcPID(0.1, -SETPOINT, double(GYRO[2]), mode_PI );
          PIDMOT = static_cast<uint16_t>(round(DIF));

          pwmptr->setPWM(LbPin, 0, spdL);
          pwmptr->setPWM(RbPin, 0, PIDMOT);
        }
        else
        {
          pwmptr->setPWM(LbPin, 0, spdL);
          pwmptr->setPWM(RbPin, 0, spdR);
        }
      } // wychylenie galki do tylu
      
      // Stoi ale obraca
      else if (LV <= lAnaMax && LV >= lAnaMin)  // ~50% +- L/R -> BRAKE or <-L/R->
      {
        if (RH > rAnaMax) // obrot w prawo
        {
          LAST_DIR = 0;
          pwmptr->setPWM(RfPin, 0, 0);          
          spdR = GEARS[cGEAR];
          pwmptr->setPWM(RbPin, 0, spdR);

          pwmptr->setPWM(LbPin, 0, 0);          
          spdL = GEARS[cGEAR];
          pwmptr->setPWM(LfPin, 0, spdL);
        }
        else if (RH < rAnaMin) // obrot w lewo
        {
          LAST_DIR = 0;
          pwmptr->setPWM(RbPin, 0, 0);          
          spdR = GEARS[cGEAR];
          pwmptr->setPWM(RfPin, 0, spdR);

          pwmptr->setPWM(LfPin, 0, 0);          
          spdL = GEARS[cGEAR];
          pwmptr->setPWM(LbPin, 0, spdL);
        }
        else { this->stopMot(); }
      }
      // NIC, wylacz motory
      else{ this->stopMot(); }

    LV_p=LV; RH_p=RH;
  }
}
void Engine::stopMot()
{
  // 'twarde' zatrzymanie
  if (LAST_DIR == 1)
  {
    pwmptr->setPWM(LfPin, 0, 0);
    pwmptr->setPWM(RfPin, 0, 0);
    pwmptr->setPWM(LbPin, 0, GEARS[cGEAR]);
    pwmptr->setPWM(RbPin, 0, GEARS[cGEAR]);
    delay(25);
    LAST_DIR = 0;
  }
  if (LAST_DIR == 2)
  {
    pwmptr->setPWM(LbPin, 0, 0);
    pwmptr->setPWM(RbPin, 0, 0);
    pwmptr->setPWM(LfPin, 0, GEARS[cGEAR]);
    pwmptr->setPWM(RfPin, 0, GEARS[cGEAR]);
    delay(10);
    LAST_DIR = 0;
  }
  pwmptr->setPWM(LfPin, 0, 0);
  pwmptr->setPWM(LbPin, 0, 0);
  pwmptr->setPWM(RfPin, 0, 0);
  pwmptr->setPWM(RbPin, 0, 0);
}
void Engine::chngGear()
{
  if (cGEAR + 1 > 3) cGEAR = 0 ;
  else cGEAR++;
  PIDMOT = GEARS[cGEAR];
}

byte Engine::giveCGear()
{ return this->cGEAR; }

double Engine::calcPID(double tInterval, double setpoint, double procVar, Engine::PIDMODE pidmode)
{
  Engine::PIDMODE cmPID = pidmode;
  double err = setpoint - procVar;
  double P_out = _Kp * err;
  double I_out{0};
  double D_out{0};
  if (cmPID==mode_PI || cmPID==mode_PID)
  {
    _integral += err * tInterval;
    I_out = _Ki * _integral;
  }
  if (cmPID==mode_PID)
  {
    double derivative = (err - _error_prev) / tInterval;
    D_out = _Kd * derivative;
  }

  double PID_OUTPUT = P_out + I_out + D_out;
  if( PID_OUTPUT > _max ) PID_OUTPUT = _max;
  else if( PID_OUTPUT < _min ) PID_OUTPUT = _min;
  _error_prev = err;
  return PID_OUTPUT;
}

void Engine::turnOnPID() { this->PIDon ? this->PIDon=false : this->PIDon=true; }

void Engine::giveLastGYRO(int16_t gy[])
{
  gy[0] = this->GYRO[0];
  gy[1] = this->GYRO[1];
  gy[2] = this->GYRO[2];
}

void Engine::setPID_SP(int newSP) { this->SETPOINT = static_cast<double>(newSP); }

double Engine::givePID_SP() { return this->SETPOINT; }
