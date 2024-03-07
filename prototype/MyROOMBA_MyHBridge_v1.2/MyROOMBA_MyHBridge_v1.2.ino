// v.1.2 - prototypowe dodanie BMI i PID
// Radio NRF
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(2, 3); // CE, CSN
const byte address[6] = "00008";

#include "Kronos.h"

// BMI160 Accel/Gyro
#include <DFRobot_BMI160.h>
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;
int16_t gyroData[3]={0}; // gyro
int16_t accelData[3]={0}; // accel
Kronos BMIdel;

#define SLAVE_ADDR 9 // ADRES I2C slave UNO+TFT

//PWM DRIVER
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 90

//DS18bxx
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NonBlockingDallas.h>
#define ONE_WIRE_BUS 8
#define TIME_INTERVAL 2000
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasTemp(&oneWire);
NonBlockingDallas sensorDs18b20(&dallasTemp);

bool DEBUG = 0;
byte radioVals[8];
byte LVert{127},  LHor{127},  RVert{127},  RHor{127};
byte LVert_p{127},  LHor_p{127},  RVert_p{127},  RHor_p{127};

byte LbtnStateS = 0, RbtnStateS = 0, LbtnStateS_p = 0, RbtnStateS_p = 0;
byte LbtnStateL = 0, RbtnStateL = 0, LbtnStateL_p = 0, RbtnStateL_p = 0;
byte potL, potR, potL_p, potR_p;

int SterL = 0, SterR = 0;
int SterFOR = 0, SterREV = 0;

byte BINARY, BINARYp;
bool pullBinary(byte bitNUM)
{
  bool WYNIK = 0;
  switch(bitNUM)
  {
    case 0: { WYNIK = (BINARY & 0x1); break; }
    case 1: { WYNIK = (BINARY & 0x2)>>1; break; }
    case 2: { WYNIK = (BINARY & 0x4)>>2; break; }
    case 3: { WYNIK = (BINARY & 0x8)>>3; break; }
    case 4: { WYNIK = (BINARY & 0x10)>>4; break; }
    case 5: { WYNIK = (BINARY & 0x20)>>5; break; }
    case 6: { WYNIK = (BINARY & 0x40)>>6; break; }
    case 7: { WYNIK = (BINARY & 0x80)>>7; break; }
    default: break;
  }
  return WYNIK;
}

//[i]     0     |   1   |    2   |    3   |     4     |    5    |   6    |            7                   |    8   |   9    |
//    SIDEBRUSH | BRUSH | BLOWER | LIGHTS | CurrMOTOR | BATTERY | cGEAR  | BLCK(bit 0,1) invRev(bit 2)LSB | Lsonar | Rsonar(BAT TEMP NOW) |
byte LCD_DATA[10] = {0,0,0,0,0,0,3,0,0,22}; // 12-bit -> 4096 -> 8bit 256 -> coef: 16 

// PWM CH    0        1           2           3
uint16_t BLOWER{0}, BRUSH{0}, SIDEBRUSH{0}, LIGHTS{0};

// PINY NANO
int Rblock = 5;
int Lblock = 6;
int LSonarPin{14}; /*A0*/ //RSonarPin{15};
int RevInvertPin = 7;
int adj_1pot = 15, /*A1*/ adj_2pot = 16; /*A2*/
//DS18 PIN 8
//RF24 PIN CE 2, PIN CSN 3
//BAT V 17 (A3)

// MAKS wartosci PWM / 4095
const uint16_t SBRUSH_MAXPWM = 2048; // 50% MAKS
const uint16_t BRUSH_MAXPWM  = 1080; // 26% / 25% MAKS 1020
const uint16_t BLOWER_MAXPWM = 4020; // 98% MAKS
const int FRONTLIGHTSCHAN = 3;
 bool revInvert = false, revInvert_p = false;
 bool RisBlck = false, RisBlck_p = false;
 bool LisBlck = false, LisBlck_p = false;
byte bitLRblck = 0;
byte selectedMotor = 0; // 0-all, 1-sidebrush, 2-brush, 3-blower
int adjPot_1_val, adjPot_2_val;
int adjPot_1_val_p, adjPot_2_val_p;

const uint16_t CAMSER_MIN = 210; // -- min(dolne), max(gorne) wychylenia serCam
const uint16_t CAMSER_MAX = 400;
uint16_t camSer=CAMSER_MIN;
Kronos camSerDel;
bool ENABLE_CamSer = false;

#include "engine.h"
Engine motors; // glowna instancja

#include "rleds.h"
//  PWM CHANS R  G  B
rleds rearLed(13,14,15);

void setup() {
Serial.begin(115200);

  // Radio
  if (radio.begin()) {Serial.println("RADIO ROBO OK --- Receiver!");}
  else Serial.println("RADIO ROBO ERROR !");
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.powerUp();
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.startListening(); 
  delay(100);
  
  Serial.println("Start DS18 Temp");
  //Initialize the sensor passing the resolution, unit of measure and reading interval [milliseconds]
  sensorDs18b20.begin(NonBlockingDallas::resolution_9, NonBlockingDallas::unit_C, TIME_INTERVAL);
  //Callbacks
  sensorDs18b20.onIntervalElapsed(handleIntervalElapsed);
  sensorDs18b20.onTemperatureChange(handleTemperatureChange);
  sensorDs18b20.onDeviceDisconnected(handleDeviceDisconnected);
  //Call the following function whenever you want to request a new temperature reading without waiting for TIME_INTERVAL to elapse
  sensorDs18b20.requestTemperature();

  Serial.println("Start BMI160");
  if (bmi160.softReset() != BMI160_OK){
    Serial.println("BMI160 s.r. false");
  }
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("BMI160 ERROR");
  }
  else Serial.println("BMI160 OK");
  
  Serial.println("Start PWM");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  Serial.println("Start ENGINE");
  //            /__mot_L__\ /__mot_R__\
  //            AForw ARev | BForw BRev| PCAM*
  motors.motBegin(4,   5,     6,    7, &pwm, &bmi160);
  pinMode(17, INPUT); // A3 - V z bat(16,8v)
  pinMode(Rblock, INPUT_PULLUP);
  pinMode(Lblock, INPUT_PULLUP);
  pinMode(RevInvertPin, INPUT_PULLUP);
  pwm.setPWM(12, 0, camSer);
  rearLed.initLeds();

  pinMode(adj_1pot,INPUT);
  pinMode(adj_2pot,INPUT);
  Serial.println("SETUP END");
} // SETUP END

volatile float FTEMP = 0;
void handleIntervalElapsed(float temperature, bool valid, int deviceIndex){
  FTEMP = temperature;
}

void handleTemperatureChange(float temperature, bool valid, int deviceIndex){ /*tylko przy zmianie TEMP*/}

void handleDeviceDisconnected(int deviceIndex){
  Serial.print("Sensor ");
  Serial.print(deviceIndex);
  Serial.println(" disconnected.");
}

byte readUltrasonicDistance(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  float CM = 0.01723*pulseIn(echoPin, HIGH, 15000); // czekaj tylko 15ms (~2,5725m MAX)
  byte CMBYTE = static_cast<byte>(round(CM));
  return CMBYTE;
}

void sendLCD()
{
  Wire.beginTransmission(SLAVE_ADDR);
  for (int i=0; i<sizeof(LCD_DATA); i++)
  { Wire.write(LCD_DATA[i]); }
  Wire.endTransmission();
}

Kronos sndLcdDel;
float batV = 0.0;
const uint16_t BAT_V_ABS_MIN = 410; // ABSOLUTNE MINIMUM BATv ~13,0v+
uint16_t bCurrAnaVal = 0;
bool BATDANGERV = false;
byte _BAT_ITER = 0;

bool isRearIndicating = false;
#include "aiBlock.h"

void calcBat()
{
  int rawBat = analogRead(17); // ~ tutaj coef = 32,3 => V=32,3 * x / 1024 ==>> x = 1024*v / 32,3
  bCurrAnaVal += rawBat;
  
  if (_BAT_ITER > 7)
  {
    bCurrAnaVal/=8;
    if (bCurrAnaVal <= BAT_V_ABS_MIN) 
    {
      BATDANGERV = true;
      rearLed.setIndiVals(4, 40, 250, 250, 0, 0);
      isRearIndicating = true;
    }
    else { BATDANGERV = false; }
    _BAT_ITER=0;
    bCurrAnaVal=0;
  }
  _BAT_ITER++;
  
  // DLA LCD
  byte batVMapped = map(rawBat, 0, 1023, 0, 255);
  if (DEBUG) Serial.println("BatRAW: "+String(rawBat)+", mapped: "+String(batVMapped));
  LCD_DATA[5] = batVMapped;
}

void LR_collision_check()
{
    // Sprawdzanie blokady prawej i lewej
    digitalRead(Rblock)==LOW ? RisBlck=true : RisBlck=false;
    digitalRead(Lblock)==LOW ? LisBlck=true : LisBlck=false;
    digitalRead(RevInvertPin)==LOW ? revInvert=true : revInvert=false;
    // Akcja po blokadzie jednorazowe
    if (RisBlck_p != RisBlck)
    {
      if (RisBlck==true) { /*Serial.println("PRAWY BLOKADA");*/ bitLRblck|=0x1; } // bit LSB 0
      else { /*Serial.println("PRAWY WOLNY");*/ bitLRblck&=0xFE; }
      LCD_DATA[7] = bitLRblck;
      RisBlck_p = RisBlck;
    }
    if (LisBlck_p != LisBlck)
    {
      if (LisBlck==true) { /*Serial.println("PRAWY BLOKADA");*/ bitLRblck|=0x2; } // bit LSB 1
      else { /*Serial.println("PRAWY WOLNY");*/ bitLRblck&=0xFD;}
      LCD_DATA[7] = bitLRblck;
      LisBlck_p = LisBlck;
    }
    // Akcja: odwrucenie sterowania L/R 'do tylu'
    if (revInvert_p != revInvert)
    {
      if (revInvert==true) { bitLRblck|=0x4; } // bit LSB 2
      else { bitLRblck&=0xFB;}
      LCD_DATA[7] = bitLRblck;
      revInvert_p = revInvert;
    }
}

void radioIncomming()
{
  if (radio.available()) 
    {
      radio.read(&radioVals, sizeof(radioVals));
      // Analogi  -                                      BAZOWE 50% dla L/R analoga
      if (isAIinControll==false) LVert = radioVals[0];   // 132  ROBO: FORWARD/BACKWARD
      LHor  = radioVals[1];                              // 127  - - -
      RVert = radioVals[2];                              // 128  CAM: UP/DOWN
      if (isAIinControll==false) RHor  = radioVals[3];   // 129  ROBO: TURN LEFT/RIGHT
      // Buttony i poty
      // BIT LSB  |   0    |    1    |    2   |    3   |     4     |     5    |    6    |    7    |
      //          |SHORT-L | SHORT-R | LONG-L | LONG-R |  *wolne   |  *wolne  | *wolne  | *wolne  |
      BINARY = radioVals[4]; // BINARY LB-S, LB-L, RB-S, RB-L
      LbtnStateS = pullBinary(0);
      RbtnStateS = pullBinary(1);
      LbtnStateL = pullBinary(2);
      RbtnStateL = pullBinary(3);
      //RbtnStateS = radioVals[5]; // DUMMY
      potL = radioVals[6];      // plynne 0-255 POT L
      potR = radioVals[7];      // plynne 0-255 POT R
  
      //swiatla cofania
      if (LVert < 100) rearLed.writeR(255);
      if (LVert > 120) rearLed.writeR(0);
      
      // Prowadzenie / kontrola robota
      if (isAIinControll==false && BATDANGERV==false) motors.drive(LVert, RHor);
      
      if (DEBUG==1)
      {
        Serial.print("LV: "+String(LVert)+", "); Serial.print("LH: "+String(LHor)+", "); Serial.print("RV: "+String(RVert)+", ");
        Serial.print("RH: "+String(RHor)+", "); Serial.print("Lbtn: "+String(LbtnStateS)+", "); Serial.println("Rbtn: "+String(RbtnStateS));
      }
      
    } // END RADIO AVAILABLE
}

void LR_anaBtns_check()
{
  // Zmiana L BTN SHORT --- GEAR
  if (LbtnStateS_p != LbtnStateS)
  {
    selectedMotor++;
    if (selectedMotor > 3) selectedMotor = 0;
    LCD_DATA[4] = selectedMotor;
    LbtnStateS_p = LbtnStateS;
    switch(LCD_DATA[4])
    {         // Indykacja rled  ticks ONms OFFms R     G   B
      case 0: {rearLed.setIndiVals(6, 120, 100,   250, 220, 0); break;}
      case 1: {rearLed.setIndiVals(4,  90,  80,   0,   180, 250); break;}
      case 2: {rearLed.setIndiVals(4,  90,  80,   0,   180, 250); break;}
      case 3: {rearLed.setIndiVals(4,  90,  80,   0,   180, 250); break;}
      default: break;
    }
    isRearIndicating = true;
  }
  // Zmiana R BTN SHORT --- GEAR
  if (RbtnStateS_p != RbtnStateS)
  {
    motors.chngGear();
    LCD_DATA[6] = motors.giveCGear();
    RbtnStateS_p = RbtnStateS;
    switch(LCD_DATA[6])
    {
      case 0: {rearLed.setIndiVals(2, 160, 100, 20, 220, 0); break;}
      case 1: {rearLed.setIndiVals(4, 140, 80, 20, 220, 0); break;}
      case 2: {rearLed.setIndiVals(6, 120, 60, 20, 220, 0); break;}
      case 3: {rearLed.setIndiVals(8, 100, 50, 20, 220, 0); break;}
      default: break;
    }
    isRearIndicating = true;
  }
  // Zmiana L BTN LONG --- LOCK/UNLOCK SERCAM
  if (LbtnStateL_p != LbtnStateL && LHor > 25)
  {
    
    if (ENABLE_CamSer==false) {ENABLE_CamSer=true; rearLed.setIndiVals(4, 60, 60, 178, 32, 190); isRearIndicating = true; }
    else {ENABLE_CamSer=false; rearLed.setIndiVals(4, 60, 60, 112, 32, 190); isRearIndicating = true; }
    LbtnStateL_p = LbtnStateL;
  }
  // Zmiana R BTN LONG --- wlacz PID
  if (RbtnStateL_p != RbtnStateL)
  {
    motors.turnOnPID();
    rearLed.setIndiVals(6, 200, 50, 255, 0, 0);
    isRearIndicating = true;
    if (RbtnStateL) rearLed.writeG(64);
    else rearLed.writeG(0);
    RbtnStateL_p = RbtnStateL;
  }
    // Zmiana L BTN LONG + R-ANA-HOR -> Aktywacja AI
  if (LbtnStateL_p != LbtnStateL && LHor <= 25)
  {
    bitLRblck|=0x8;
    isAIinControll = true;
    LCD_DATA[7] = bitLRblck;
    rearLed.writeG(150);
    LbtnStateL_p = LbtnStateL;
  }
  if (LHor > 225) // wyl.AI
  {
    bitLRblck&=0xF7;
    isAIinControll = false;
    LCD_DATA[7] = bitLRblck;
    rearLed.writeG(0);
  }
}

void R_pot_check()
{
if ((potR_p != potR) && BATDANGERV==false)
  {
    /*Vout z PWM -> PWMmax(dla Vout) = Vout/(Vin/4095)*/
    switch(selectedMotor)
    {
      case 0: // WSZYSTKIE
      {
        if (potR < 15) 
        {
          SIDEBRUSH = 0; pwm.setPWM(0, 0, SIDEBRUSH);
          BRUSH = 0; pwm.setPWM(1, 0, BRUSH);
          BLOWER = 0; pwm.setPWM(2, 0, BLOWER);
        } 
        else  
        { 
          SIDEBRUSH = map(potR, 15, 255, 32, SBRUSH_MAXPWM); pwm.setPWM(0, 0, SIDEBRUSH);
          BRUSH =     map(potR, 15, 255, 32, BRUSH_MAXPWM);  pwm.setPWM(1, 0, BRUSH);
          BLOWER =    map(potR, 15, 255, 32, BLOWER_MAXPWM); pwm.setPWM(2, 0, BLOWER);
        }
        
        LCD_DATA[0] = map(SIDEBRUSH, 0, SBRUSH_MAXPWM, 0, 255);
        LCD_DATA[1] = map(BRUSH, 0, BRUSH_MAXPWM, 0, 255);
        LCD_DATA[2] = map(BLOWER, 0, BLOWER_MAXPWM, 0, 255);
        
        potR_p = potR;
        break;
      }
      case 1: // SIDEBRUSH
      {
        if (potR < 15) { SIDEBRUSH = 0; pwm.setPWM(0, 0, SIDEBRUSH); } // ~11v
        else  { SIDEBRUSH = map(potR, 15, 255, 32, SBRUSH_MAXPWM); pwm.setPWM(0, 0, SIDEBRUSH); }
        LCD_DATA[0] = map(SIDEBRUSH, 0, SBRUSH_MAXPWM, 0, 255);
        potR_p = potR;
        break;
      }
      case 2: // MAIN BRUSH - Vout = 12.0v ~ PWMmax = 3071 / dla 6.0v = 1535
      {
        if (potR < 15) { BRUSH = 0; pwm.setPWM(1, 0, BRUSH); } // ~16v
        else  { BRUSH = map(potR, 15, 255, 32, BRUSH_MAXPWM); pwm.setPWM(1, 0, BRUSH); }
        LCD_DATA[1] = map(BRUSH, 0, BRUSH_MAXPWM, 0, 255);
        potR_p = potR;
        break;
      }
      case 3: // BLOWER - Vout = 15.0v ~ PWMmax = 3839
      {
        if (potR < 15) { BLOWER = 0; pwm.setPWM(2, 0, BLOWER); } // ~ 16v
        else  { BLOWER = map(potR, 15, 255, 32, BLOWER_MAXPWM); pwm.setPWM(2, 0, BLOWER); }
        LCD_DATA[2] = map(BLOWER, 0, BLOWER_MAXPWM, 0, 255);
        potR_p = potR;
        break;
      }
      default: break;
    } // switch
  } // if  
if (BATDANGERV==true)
  {
    SIDEBRUSH = 0; pwm.setPWM(0, 0, SIDEBRUSH);
    BRUSH = 0; pwm.setPWM(1, 0, BRUSH);
    BLOWER = 0; pwm.setPWM(2, 0, BLOWER);
  }
}

void L_pot_check()
{
if ((potL_p-5 < potL || potL_p+5 > potL) && BATDANGERV==false)
  {
    if (potL < 15) 
    {
      LIGHTS = 0; 
      pwm.setPWM(FRONTLIGHTSCHAN, 0, LIGHTS); rearLed.writeB(0);
    }
    else           
    { 
      LIGHTS = map(potL, 15, 255, 256, 4095);
      pwm.setPWM(FRONTLIGHTSCHAN, 0, LIGHTS); rearLed.writeB(map(potL, 15, 255, 64, 255));
    }
    LCD_DATA[3] = map(LIGHTS, 0, 4095, 0, 255);
    potL_p = potL;
  }
  
  else if (BATDANGERV==true) { pwm.setPWM(FRONTLIGHTSCHAN, 0, 0); rearLed.writeRGB(0,0,0); } // BAT LOW
}

void R_vert_analog_check()
{
  if (RVert > 220 && camSerDel.del(30) && ENABLE_CamSer==true)
  {
    pwm.setPWM(12, 0, camSer);
    if (camSer<400) camSer+=4;
  }
  if (RVert < 30 && camSerDel.del(30) && ENABLE_CamSer==true)
  {
    pwm.setPWM(12, 0, camSer);
    if (camSer>210) camSer-=4;
  }  
}

void L_hor_analog_check(){/*Chwilowo nic*/}

void handle_adjPots()
{
  adjPot_1_val = analogRead(adj_1pot);
  //adjPot_2_val = analogRead(adj_2pot);

  if (adjPot_1_val > adjPot_1_val_p+15 || adjPot_1_val < adjPot_1_val_p-15 ) // histereza
  { 
    int pSP = map(adjPot_1_val, 0, 1024, -100, 100);
    motors.setPID_SP(pSP);
    Serial.println("CurrSP: "+String(motors.givePID_SP()));
    adjPot_1_val_p = adjPot_1_val; 
  }
  
  //if (adjPot_2_val > adjPot_2_val_p+15 || adjPot_2_val < adjPot_2_val_p-15) 
  //{ 
  //  Serial.println("Adj2: "+String(adjPot_2_val));
  //  adjPot_2_val_p = adjPot_2_val; 
  //}
}
void loop() {

  radioIncomming();
  handle_adjPots();
  // A.I. Drive
  if (isAIinControll==true && BATDANGERV==false) { AI_drive(); }

  // Sprawdzanie l/r kolizji + inversRev
  LR_collision_check();
  
  // Zmiana L/R ana BTNs --- wybor motora / bieg
  LR_anaBtns_check();

  // Prawy POT - MOC motorow
  R_pot_check();
    
  // Lewy POT - headlights/rearlighs
  L_pot_check();

  //Wysylanie na EKRAN
  if (sndLcdDel.del(200))
  { 
    calcBat();
    LCD_DATA[8] = readUltrasonicDistance(LSonarPin,LSonarPin);
    LCD_DATA[9] = static_cast<byte>(round(FTEMP*2.f));
    //LCD_DATA[9] = readUltrasonicDistance(RSonarPin,RSonarPin); // chwilowo nie podlaczony
    sendLCD();
  }
  
  //Zmiana LEWA galka HORYZONTALNA - narazie NIC
  //L_hor_analog_check();
  
  // Zmiana PRAWA galka VERTYKALNA - U/D camera ~ PWM down 210 / up 400 (/4095)
  R_vert_analog_check();

  //Indicator rear swiatel
  if (isRearIndicating==true) { if (rearLed.indicate()==false) { isRearIndicating=false; } }
  // Temp. BAT
  sensorDs18b20.update();
  
  // BMI A/G prototypowe
  if (BMIdel.del(150)) {
    
    //bmi160.getGyroData(gyroData); 
    //bmi160.getAccelData(accelData);
    //float accelX = accelData[0]/16384.0;
    //float accelY = accelData[1]/16384.0;
    //float accelZ = accelData[2]/16384.0;
    //Serial.println(accelZ);
    //Serial.println(String(accelX)+","+String(accelY));
    //if (gyroData[2] <= 5 && gyroData[2] >= -5) gyroData[2] = 0;
    //motors.giveLastGYRO(gyroData);
    // Reakcja na wstrzas vertykalny
    //if (accelZ > 10.0 || accelZ < -9.6 )
    //{ rearLed.setIndiVals(10, 50, 50, 250, 250, 0); isRearIndicating = true; }
    //Serial.println(gyroData[2]);
    
    }
  

} // loop END
