#ifndef AIBLOCK_H
#define AIBLOCK_H

bool isAIinControll = false;
byte OBSTACLEDIS = 0;
Kronos AIatractor;

bool chngDirFlag = false;
byte DIRR = 1;
void AI_drive()
{
  OBSTACLEDIS = readUltrasonicDistance(LSonarPin,LSonarPin);
  delay(50);
  if ( (OBSTACLEDIS < 3 || OBSTACLEDIS > 25 && (LisBlck==false && RisBlck==false)) )
  {
    if (AIatractor.del(random(4000, 13000))) { motors.drive(132, random(50,250)); delay(random(200,600)); } // rand dla drobnych zmian
    motors.drive(250, 129);
    chngDirFlag=false;
  }
  else if ((OBSTACLEDIS >=3 && OBSTACLEDIS <= 25) )
  {
    if (chngDirFlag==false)
    {
      chngDirFlag=true;
      DIRR = random(0,1);
    }
    if (DIRR == 0) { motors.drive(132, random(200,250)); }
    else if (DIRR == 1) { motors.drive(132, random(5,100)); }
    else { motors.drive(132, random(5,250)); }
    delay(random(300,900));
    motors.drive(132, 129);
    OBSTACLEDIS = readUltrasonicDistance(LSonarPin,LSonarPin);
    delay(300);
  }
  else if ((LisBlck==true || RisBlck==true)  )
  {
    chngDirFlag=false;
    if (LisBlck==true && RisBlck==false  )
    {
      motors.drive(50, 129);
      delay(random(400,700));
      motors.drive(132, random(200,250));
      delay(random(400,900));
    }
    else if ((LisBlck==false && RisBlck==true)  )
    {
      motors.drive(50, 129);
      delay(random(400,700));
      motors.drive(132, random(5,100));
      delay(random(400,900));
    }
    else
    {
      motors.drive(50, 129);
      delay(random(400,700));
      motors.drive(132, random(50,200));
      delay(random(400,900));
    }
    
  }
  else motors.drive(132, 129); // STOP jezeli nie wiadomo co sie dzieje
}

#endif
