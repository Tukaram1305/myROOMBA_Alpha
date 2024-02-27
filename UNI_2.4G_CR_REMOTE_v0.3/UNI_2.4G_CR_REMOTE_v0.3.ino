
// Radio NRF
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(2, 3); // CE, CSN
const byte address[6] = "00008";

bool DEBUG = 0;
const byte R_bPin = 6;
const byte L_bPin = 7;
class Kronos
{
  public:
  Kronos() : mili(0), rev(true), already(false) {};
  ~Kronos(){};
  bool del (unsigned int delx)
  {
    if (millis()-mili > delx)
    {
      this->mili = millis();
      return true;
    }
    else return false;
  }
  bool delMicro (unsigned int delx)
  {
    if (micros()-mili > delx)
    {
      this->mili = micros();
      return true;
    }
    else return false;
  }
  bool startDel (unsigned int delx)
  {
    if (rev == true)
    {
      this->mili = millis();
      this->rev = false;
      return false;
    }
    if (this->rev == false)
    {
      if (millis()<mili+delx) return false;
      else
      {
        this->rev = true;
        return true;
      }
    }
  }
  void CNT ()
  {
    this->mili = millis();
  }
  unsigned long STP ()
  {
    this->miliNew = millis();
    return (this->miliNew-(this->mili));
  }
  bool ONCE ()
  {
    if (this->already == true) return false;
    else 
    {
      this->already = true;
      return true;
    }
  }
  void RESET ()
  {
    if (this->already == true) this->already = false;
  }
  private:
  bool already = false;
  bool rev = true;
  unsigned long mili;
  unsigned long miliNew;
};

void setup() {

pinMode(14,INPUT); // A0
pinMode(15,INPUT); // A1
pinMode(16,INPUT); // A2
pinMode(17,INPUT); // A3
pinMode(A4,INPUT); // A4 potL
pinMode(A5,INPUT); // A5 potR
pinMode(6,INPUT_PULLUP);
pinMode(7,INPUT_PULLUP);
  Serial.begin(115200);

  // Radio
  if (radio.begin()) { Serial.println("Radio TX OK!"); }
  else Serial.println("Radio ERROR");
  
  radio.openWritingPipe(address); delay(50);
  radio.setPALevel(RF24_PA_LOW); delay(50);
  radio.stopListening(); delay(50);
  radio.setRetries(15, 15); delay(50);
  radio.powerUp(); delay(50);
  radio.setDataRate(RF24_250KBPS); delay(50);
  radio.enableDynamicPayloads();
}
Kronos b1, b2;

byte radioVals[8];

int raw1, raw2, raw3, raw4, raw5, raw6;
byte LVert{127},  LHor{127},  RVert{127},  RHor{127};
byte LVertp{127}, LHorp{127}, RVertp{127}, RHorp{127};
byte potL, potR;
byte potLp, potRp;
//byte LbtnState = 0, RbtnState = 0;
//byte LbtnStatep = 0, RbtnStatep = 0;

bool b1Rel = true;
bool b2Rel = true;

// BIT LSB  |   0    |    1    |    2   |    3   |DO DODANIA->      4     |     5    |    6    |    7    |
//          |SHORT-L | SHORT-R | LONG-L | LONG-R |DO DODANIA->  R+L SHORT | R+L LONG |
byte BINARY = 0;
byte BINARYp = 0;
uint32_t longPrsCounter_L = 0;
uint32_t longPrsCounter_R = 0;
bool DOUBLE_PRESS = false;
bool DUMMY1p, DUMMY1;

void pushBinary(byte bitNUM, bool bitVAL)
{
  switch(bitNUM)
  {
    case 0: { BINARY &=0xFE; break; }
    case 1: { BINARY &=0xFD; break; }
    case 2: { BINARY &=0xFB; break; }
    case 3: { BINARY &=0xF7; break; }
    case 4: { BINARY &=0xEF; break; }
    case 5: { BINARY &=0xDF; break; }
    case 6: { BINARY &=0xBF; break; }
    case 7: { BINARY &=0x7F; break; }
    default: break;
  }
  BINARY |= (bitVAL<<bitNUM);
}
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

void chkBtns()
{
  if (digitalRead(R_bPin)==LOW && b1Rel==true && b1.del(200) )
  { b1Rel = false; longPrsCounter_R=millis(); }
  
  if (digitalRead(R_bPin)==HIGH && b1Rel==false) 
  { 
    uint32_t PRSTIME = millis()-longPrsCounter_R;
    if (PRSTIME < 800)
    {
      if (digitalRead(L_bPin)==HIGH) (pullBinary(0)==1) ? pushBinary(0,0) : pushBinary(0,1);
      b1Rel = true;
    }
    else 
    {
      if (digitalRead(L_bPin)==HIGH) (pullBinary(2)==1) ? pushBinary(2,0) : pushBinary(2,1);
      b1Rel = true;
    }
  }


if (digitalRead(L_bPin)==LOW && b2Rel==true && b2.del(200))
  { b2Rel = false; longPrsCounter_L=millis(); }
  
  if (digitalRead(L_bPin)==HIGH && b2Rel==false) 
  {
    uint32_t PRSTIME = millis()-longPrsCounter_L;
    if (PRSTIME < 800)
    {
      //SHORT PRESS
      if (digitalRead(R_bPin)==HIGH) (pullBinary(1)==1) ? pushBinary(1,0) : pushBinary(1,1);
      b2Rel = true;
    }
    else 
    {
      //LONG PRESS
      if (digitalRead(R_bPin)==HIGH) (pullBinary(3)==1) ? pushBinary(3,0) : pushBinary(3,1);
      b2Rel = true;
    }
     
  }

  
}

void showBTNdebug()
{
  Serial.println("BINARY: "+String(BINARY,BIN));
}
void mapAnalog()
{
  raw1 = analogRead(14);
  raw2 = analogRead(15);
  raw3 = analogRead(16);
  raw4 = analogRead(17);
  raw5 = analogRead(A4);
  raw6 = analogRead(A5);
  // BAZOWO
  LVert = map(raw1, 0, 1023, 0 , 255); // 132
  RHor =  map(raw2, 0, 1023, 0 , 255); // 129
  RVert = map(raw3, 0, 1023, 0 , 255); // 128
  LHor =  map(raw4, 0, 1023, 0 , 255); // 126/127
  potL = map(raw5, 1023, 0, 0 , 255);
  potR = map(raw6, 0, 1023, 0 , 255);

  if (DEBUG==1)
  {
    Serial.print("LV: "+String(LVert)+", ");
    Serial.print("LH: "+String(LHor)+", ");
    Serial.print("RV: "+String(RVert)+", ");
    Serial.print("RH: "+String(RHor)+", ");
    Serial.print("PotL: "+String(potL)+", ");
    Serial.println("PotR: "+String(potR));
  }
}

void loop() {

  chkBtns();
  mapAnalog();

  //if ( BINARYp != BINARY) { showBTNdebug(); BINARYp = BINARY; }
  
  if (
  LVert > LVertp+2 || LVert < LVertp-2 ||
  LHor  > LHorp+2  || LHor < LHorp-2   ||
  RVert > RVertp+2 || RVert < RVertp-2 ||
  RHor  > RHorp+2  || RHor < RHorp-2   ||
  potL > potLp+1   || potL < potLp-1   ||
  potR > potRp+1   || potR < potRp-1   ||
  BINARYp != BINARY || 
  DUMMY1p != DUMMY1 )
  {
        radioVals[0] = LVert;
        radioVals[1] = LHor;
        radioVals[2] = RVert;
        radioVals[3] = RHor;
        radioVals[4] = BINARY;
        radioVals[5] = DUMMY1;
        radioVals[6] = potL;
        radioVals[7] = potR;
        bool report = radio.write(&radioVals, sizeof(radioVals)); 
        if (DEBUG==1) {Serial.print("Wys. stan: "+String(report));}
        LVertp = LVert;
        LHorp  = LHor;
        RVertp = RVert;
        RHorp  = RHor;
        BINARYp = BINARY;
        DUMMY1p = DUMMY1;
        potRp = potR;
        potLp = potL;
  }

}
