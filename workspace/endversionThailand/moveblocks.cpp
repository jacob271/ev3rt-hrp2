#include "blocks.h"
#include "timer.h"
#include "stalldetection.h"

// ACTION BLOCKS
//-----------------------------------------------------------------------------

void turn1_rot(motor_port_t turnMotor, int startSpeed, int maxSpeed, double angle, int endSpeed)
{
  angle = 0.99 * angle * (17.6 * 2) / wheelDiameter; //0.98
  int resetTurnMotor;
  if (turnMotor == motor_right)
    resetTurnMotor = resetRightDegree;
  else
    resetTurnMotor = resetLeftDegree;
  motor_port_t othermotor;
  int othermotorreset;
  if (turnMotor == motor_right)
  {
    ev3_motor_stop(motor_left, true);
    othermotor = motor_left;
    othermotorreset = resetLeftDegree;
  }
  else
  {
    ev3_motor_stop(motor_right, true);
    othermotor = motor_right;
    othermotorreset = resetRightDegree;
  }
  double togo = abs(angle);
  cSpeed = startSpeed;
  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  startSpeed = speedLevel(startSpeed);
  StallDetection stall;

  Stopwatch move;
  bool bremst = false;
  int counter = 0;
  int lasttime = 0;
  std::vector<int> test;
  while (togo > 0) // && !stall.detectStall())
  {
    togo = abs(angle) - abs(ev3_motor_get_counts(turnMotor) - resetTurnMotor) + abs(ev3_motor_get_counts(othermotor) - othermotorreset);
    cSpeed = accDec(togo, bfTurn1, move, startSpeed, minSpeed, maxSpeed, endSpeed, stall, bremst); //13
    // Stillstandserkennung
    if (move.getTime() != lasttime)
    {
      stall.measure(abs(ev3_motor_get_counts(turnMotor)));
      if (stall.detectStall())
        break;
    }
    lasttime = move.getTime();

    if (turnMotor == motor_right)
      ev3_motor_set_power(turnMotor, cSpeed);
    else
      ev3_motor_set_power(turnMotor, cSpeed * (-1));
    counter++;
    int brakespeed = abs(endSpeed) - 0 - togo;
    if (brakespeed > 0)
    {
      if (othermotor == motor_right)
        ev3_motor_set_power(othermotor, brakespeed * (endSpeed / abs(endSpeed)));
      else
        ev3_motor_set_power(othermotor, brakespeed * (endSpeed / abs(endSpeed)) * (-1));
    }
    else
    {
      int distanceToBrake = abs(startSpeed) * (bfTurn1 * 2);

      int braketogo = distanceToBrake - abs(ev3_motor_get_counts(othermotor) - othermotorreset);
      if (braketogo >= 0)
      {
        if (othermotor == motor_left)
          ev3_motor_set_power(othermotor, (braketogo / (bfTurn1 * 2)) * -1 * (endSpeed / abs(endSpeed)));
        else
          ev3_motor_set_power(othermotor, (braketogo / (bfTurn1 * 2)) * (endSpeed / abs(endSpeed)));
      }
    }
  }

  if (turnMotor == motor_right)
  {
    if (maxSpeed > 0)
    {
      resetRightDegree = resetRightDegree + (int)(angle) + abs(ev3_motor_get_counts(othermotor) - othermotorreset);
      resetLeftDegree = resetLeftDegree - abs(ev3_motor_get_counts(othermotor) - othermotorreset);
    }
    else
    {
      resetRightDegree = resetRightDegree - ((int)(angle) + abs(ev3_motor_get_counts(othermotor) - othermotorreset));
      resetLeftDegree = resetLeftDegree + abs(ev3_motor_get_counts(othermotor) - othermotorreset);
    }
  }
  else if (maxSpeed > 0)
  {
    resetLeftDegree = resetLeftDegree - ((int)(angle) + abs(ev3_motor_get_counts(othermotor) - othermotorreset));
    resetRightDegree = resetRightDegree + abs(ev3_motor_get_counts(othermotor) - othermotorreset);
  }
  else
  {
    resetLeftDegree = resetLeftDegree + ((int)(angle) + abs(ev3_motor_get_counts(othermotor) - othermotorreset));
    resetRightDegree = resetRightDegree - abs(ev3_motor_get_counts(othermotor) - othermotorreset);
  }

  for (int i = 0; i < test.size(); i++)
    std::cout << test[i] << std::endl;
  brakeAtEnd(endSpeed); // Motoren anhalten
  char buffer[10];
  itoa(counter / (move.getTime() / 1000.), buffer, 10);
  ev3_lcd_draw_string(buffer, 20, 50);
}

//Drehung auf zwei Rädern mit Rotationssensoren
void turn2_rot(int startSpeed, int maxSpeed, double angle, int endSpeed)
{

  // Rotationssensoren zurücksetzen

  //int leftdegree = ev3_motor_get_counts(motor_left);
  //int rightdegree = ev3_motor_get_counts(motor_right);

  angle = 0.98 * angle * (17.6) / wheelDiameter;
  double togo = abs(angle);
  cSpeed = startSpeed;
  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;

  int pCorrection = 0;
  Stopwatch move;
  bool bremst = false;

  while (togo > 0)
  {
    togo = abs(angle) - (abs(ev3_motor_get_counts(motor_left) - resetLeftDegree) + abs(ev3_motor_get_counts(motor_right) - resetRightDegree)) / 2;
    cSpeed = accDec(togo, bfTurn2, move, startSpeed, minSpeed, maxSpeed, endSpeed, stall, bremst); //9
    stall.measure(abs(ev3_motor_get_counts(motor_left)));
    pCorrection = 1.5 * ((ev3_motor_get_counts(motor_right) - resetRightDegree) - (ev3_motor_get_counts(motor_left) - resetLeftDegree));
    ev3_motor_set_power(motor_left, cSpeed + pCorrection);
    ev3_motor_set_power(motor_right, cSpeed - pCorrection);
  }
  brakeAtEnd(endSpeed); // Motoren anhalten
  if (maxSpeed > 0)
  {
    resetRightDegree = resetRightDegree + (int)(angle);
    resetLeftDegree = resetLeftDegree + (int)(angle);
  }
  else
  {
    resetRightDegree = resetRightDegree - (int)(angle);
    resetLeftDegree = resetLeftDegree - (int)(angle);
  }
  //std::cout << "angle:  " << (int) (angle) << " " << std::endl;
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;
}

//Paulturn mit Rotationssensoren
void paulturn_rot(int startSpeed, int maxSpeed, double angle, double leftratio, double rightratio, int endSpeed)
{
  angle = angle * (17.6) / wheelDiameter;
  double togo = abs(angle);
  cSpeed = startSpeed;
  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;

  int pCorrection = 0;
  Stopwatch move;
  bool bremst = false;

  while (togo > 0)
  {
    togo = abs(angle) - (abs(ev3_motor_get_counts(motor_left) - resetLeftDegree) + abs(ev3_motor_get_counts(motor_right) - resetRightDegree)) / 2;
    cSpeed = accDec(togo, bfTurn2, move, startSpeed, 35, maxSpeed, endSpeed, stall, bremst); //9
    stall.measure(abs(ev3_motor_get_counts(motor_left)));
    pCorrection = 1.5 * ((ev3_motor_get_counts(motor_right) - resetRightDegree) / rightratio - (ev3_motor_get_counts(motor_left) - resetLeftDegree) / leftratio);
    ev3_motor_set_power(motor_left, cSpeed * leftratio + pCorrection);
    ev3_motor_set_power(motor_right, cSpeed * rightratio - pCorrection);
  }
  brakeAtEnd(endSpeed); // Motoren anhalten

  resetRightDegree = ev3_motor_get_counts(motor_right);
  resetLeftDegree = ev3_motor_get_counts(motor_left);
}

//Move mit Rotationssensoren
int moveRotDeg(int startSpeed, int maxSpeed, int distance, int endSpeed, bool colorSearch)
{
  distance = distance * wheelConverter; 
  cSpeed = startSpeed;

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;
  int togo = distance;
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe
  Stopwatch move;

  int counter = 0; 
  bool bremst = false;

  while (togo > 0)
  {
    tslp_tsk(1);
    counter++;// Counter um Schleifendurchläufe zu berechnen
    togo = distance - (abs(ev3_motor_get_counts(motor_left) - resetLeftDegree) + abs(ev3_motor_get_counts(motor_right) - resetRightDegree)) / 2;
    cSpeed = accDec(togo, bfMove, move, startSpeed, minSpeed, maxSpeed, endSpeed, stall, bremst);
    motorCorrection(pGain, iGain, iCorrection, cSpeed, resetRightDegree, resetLeftDegree);

    if (colorSearch)
    {
      colorCounter[colorDetection_rgb(LSaussen)]++;
      tslp_tsk(4);
    }
  }

  brakeAtEnd(endSpeed); // Motoren anhalten

  // Zurücksetzen der Rotationssensoren
  if (maxSpeed > 0)
  {
    resetRightDegree = resetRightDegree + distance;
    resetLeftDegree = resetLeftDegree - distance;
  }
  else
  {
    resetRightDegree = resetRightDegree - distance;
    resetLeftDegree = resetLeftDegree + distance;
  }

  // Durchschnittliche Schleifendurchläufe auf dem Display anzeigen
  char buffer[10];
  itoa(counter / (move.getTime() / 1000.), buffer, 10);
  ev3_lcd_draw_string(buffer, 20, 50);

  // Ggbf. Erkannte Farbe ausgeben
  if (colorSearch)
  {
    return frequencyDistribution(colorCounter);
  }
  return 0;
}


// Move mit Rotationssensoren ohne Synchronisation
void moveRotDegNoSync(int startSpeed, int maxSpeed, int distance, int endSpeed)
{
  distance = distance * wheelConverter;
  cSpeed = startSpeed;
  //double localPGain = pGain * speedLevel(abs(maxSpeed));

  //resetLeftDegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  //resetRightDegree = ev3_motor_get_counts(motor_right);

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;
  int togo = distance;
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe
  Stopwatch move;

  int counter = 0;
  bool bremst = false;

  while (togo > 0)
  {
    tslp_tsk(1);

    counter++;
    togo = distance - (abs(ev3_motor_get_counts(motor_left) - resetLeftDegree) + abs(ev3_motor_get_counts(motor_right) - resetRightDegree)) / 2;
    cSpeed = accDec(togo, bfMove, move, startSpeed, minSpeed, maxSpeed, endSpeed, stall, bremst);
    //if (togo / 3 + (minSpeed*0.5) <= maxSpeed && brake == true)
    ev3_motor_set_power(motor_left,-cSpeed);
    ev3_motor_set_power(motor_right,cSpeed);
    int counter2 = counter / (move.getTime() / 1000);
  }

  brakeAtEnd(endSpeed); // Motoren anhalten

  resetRightDegree = ev3_motor_get_counts(motor_right);
  resetLeftDegree = ev3_motor_get_counts(motor_left);
  // Durchschnittliche Schleifendurchläufe auf dem Display anzeigen
  char buffer[10];
  itoa(counter / (move.getTime() / 1000.), buffer, 10);
  ev3_lcd_draw_string(buffer, 20, 50);

}



//Move mit Rotationssensoren und Verhältnis
int moveRotDegRatio(int startSpeed, int maxSpeed, int distance, double rratio, double lratio, int endSpeed, bool colorSearch)
{
  distance = distance * wheelConverter;
  cSpeed = startSpeed;
  //double localPGain = pGain * speedLevel(abs(maxSpeed));

  //int leftdegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  //int rightdegree = ev3_motor_get_counts(motor_right);

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;
  int togo = distance;
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe
  Stopwatch move;
  double _pGain = pGain;

  int counter = 0;
  bool bremst = false;

  while (togo > 0)
  {
    tslp_tsk(1);

    counter++;
    togo = distance - (abs(ev3_motor_get_counts(motor_left) - resetLeftDegree) / lratio + abs(ev3_motor_get_counts(motor_right) - resetRightDegree) / rratio) / 2;
    cSpeed = accDec(togo, bfMove, move, startSpeed, minSpeed, maxSpeed, endSpeed, stall, bremst);
    //if (togo / 3 + (minSpeed*0.5) <= maxSpeed && brake == true)
    double pCorrection;
    pCorrection = ((ev3_motor_get_counts(motor_left) - resetLeftDegree) / lratio + (ev3_motor_get_counts(motor_right) - resetRightDegree) / rratio) * (abs(_pGain * cSpeed) + 0.4);
    ev3_motor_set_power(motor_left, (-1) * (cSpeed + pCorrection) * lratio); //-100.
    ev3_motor_set_power(motor_right, (cSpeed - pCorrection) * rratio);
    int counter2 = counter / (move.getTime() / 1000);
    char msgbuf[10];
    sprintf(msgbuf, "count: %d", counter2);
    ev3_lcd_draw_string(msgbuf, 10, 10);
    if (colorSearch)
    {
      colorCounter[colorDetection_rgb(LSaussen)]++;
      tslp_tsk(4);
    }
  }

  brakeAtEnd(endSpeed); // Motoren anhalten

  resetRightDegree = ev3_motor_get_counts(motor_right);
  resetLeftDegree = ev3_motor_get_counts(motor_left);
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;

  // Durchschnittliche Schleifendurchläufe auf dem Display anzeigen
  char buffer[10];
  itoa(counter / (move.getTime() / 1000.), buffer, 10);
  ev3_lcd_draw_string(buffer, 20, 50);

  if (colorSearch)
  {
    return frequencyDistribution(colorCounter);
  }
  return 0;
}

//Move mit Rotationssensoren und Verhältnis ohne Synchronisation
int moveRotDegRatioNoSync(int startSpeed, int maxSpeed, int distance, double rratio, double lratio, int endSpeed, bool colorSearch)
{
  distance = distance * wheelConverter;
  cSpeed = startSpeed;
  //double localPGain = pGain * speedLevel(abs(maxSpeed));

  //int leftdegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  //int rightdegree = ev3_motor_get_counts(motor_right);

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;
  int togo = distance;
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe
  Stopwatch move;
  double _pGain = pGain;

  int counter = 0;
  bool bremst = false;

  while (togo > 0)
  {
    tslp_tsk(1);

    counter++;
    togo = distance - (abs(ev3_motor_get_counts(motor_left) - resetLeftDegree) / lratio + abs(ev3_motor_get_counts(motor_right) - resetRightDegree) / rratio) / 2;
    cSpeed = accDec(togo, bfMove, move, startSpeed, minSpeed, maxSpeed, endSpeed, stall, bremst);
    //if (togo / 3 + (minSpeed*0.5) <= maxSpeed && brake == true)
    ev3_motor_set_power(motor_left, (-1) * (cSpeed) * lratio); //-100.
    ev3_motor_set_power(motor_right, (cSpeed) * rratio);
    int counter2 = counter / (move.getTime() / 1000);
    char msgbuf[10];
    sprintf(msgbuf, "count: %d", counter2);
    ev3_lcd_draw_string(msgbuf, 10, 10);
    if (colorSearch)
    {
      colorCounter[colorDetection_rgb(LSaussen)]++;
      tslp_tsk(4);
    }
  }

  brakeAtEnd(endSpeed); // Motoren anhalten

  resetRightDegree = ev3_motor_get_counts(motor_right);
  resetLeftDegree = ev3_motor_get_counts(motor_left);
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;

  // Durchschnittliche Schleifendurchläufe auf dem Display anzeigen
  char buffer[10];
  itoa(counter / (move.getTime() / 1000.), buffer, 10);
  ev3_lcd_draw_string(buffer, 20, 50);

  if (colorSearch)
  {
    return frequencyDistribution(colorCounter);
  }
  return 0;
}

//Move mit Rotationssensoren und medium Motoren
int moveRotDegMediumMotor(int startSpeed, int maxSpeed, int distance, int endSpeed, bool colorSearch, motor_port_t motor, int speed, int degree, bool stop)
{
  distance = distance * wheelConverter;
  cSpeed = startSpeed;
  ev3_motor_reset_counts(motor);

  //int leftdegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  //int rightdegree = ev3_motor_get_counts(motor_right);

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;
  int togo = distance;
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe
  Stopwatch move;

  int counter = 0;
  bool bremst = false;

  while (togo > 0)
  {
    tslp_tsk(1);

    counter++;
    togo = distance - (abs(ev3_motor_get_counts(motor_left) - resetLeftDegree) + abs(ev3_motor_get_counts(motor_right) - resetRightDegree)) / 2;
    cSpeed = accDec(togo, bfMove, move, startSpeed, minSpeed, maxSpeed, endSpeed, stall, bremst);
    motorCorrection(pGain, 0, iCorrection, cSpeed, resetRightDegree, resetLeftDegree);
    ev3_motor_set_power(motor, speed);

    if (abs(ev3_motor_get_counts(motor)) > degree)
      ev3_motor_stop(motor, stop);

    if (colorSearch)
    {
      colorCounter[colorDetection(LSaussen)]++;
      tslp_tsk(4);
    }
  }

  brakeAtEnd(endSpeed); // Motoren anhalten

  if (maxSpeed > 0)
  {
    resetRightDegree = resetRightDegree + distance;
    resetLeftDegree = resetLeftDegree - distance;
  }
  else
  {
    resetRightDegree = resetRightDegree - distance;
    resetLeftDegree = resetLeftDegree + distance;
  }
  //std::cout<< "distance: " << distance << " "<< std::endl;
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;

  // Durchschnittliche Schleifendurchläufe auf dem Display anzeigen
  char buffer[10];
  itoa(counter / (move.getTime() / 1000.), buffer, 10);
  ev3_lcd_draw_string(buffer, 20, 50);

  if (colorSearch)
  {
    return frequencyDistribution(colorCounter);
  }
  return 0;
}

//Move mit Rotationssensoren und medium Motoren auf Zeit
int moveRotDegMediumMotorTime(int startSpeed, int maxSpeed, int distance, int endSpeed, bool colorSearch, motor_port_t motor, int speed, int time, bool stop)
{
  distance = distance * wheelConverter;
  cSpeed = startSpeed;
  ev3_motor_reset_counts(motor);

  //int leftdegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  //int rightdegree = ev3_motor_get_counts(motor_right);

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;
  int togo = distance;
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe
  Stopwatch move;

  int counter = 0;
  bool bremst = false;

  while (togo > 0)
  {
    tslp_tsk(1);

    counter++;
    togo = distance - (abs(ev3_motor_get_counts(motor_left) - resetLeftDegree) + abs(ev3_motor_get_counts(motor_right) - resetRightDegree)) / 2;
    cSpeed = accDec(togo, bfMove, move, startSpeed, minSpeed, maxSpeed, endSpeed, stall, bremst);
    motorCorrection(pGain, 0, iCorrection, cSpeed, resetRightDegree, resetLeftDegree);
    ev3_motor_set_power(motor, speed);

    if (move.getTime() > time)
      ev3_motor_stop(motor, stop);

    if (colorSearch)
    {
      colorCounter[colorDetection(LSaussen)]++;
      tslp_tsk(4);
    }
  }

  brakeAtEnd(endSpeed); // Motoren anhalten

  if (maxSpeed > 0)
  {
    resetRightDegree = resetRightDegree + distance;
    resetLeftDegree = resetLeftDegree - distance;
  }
  else
  {
    resetRightDegree = resetRightDegree - distance;
    resetLeftDegree = resetLeftDegree + distance;
  }
  //std::cout << "distance:  " << distance << " " << std::endl;
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;

  char buffer[10];
  itoa(counter / (move.getTime() / 1000.), buffer, 10);
  ev3_lcd_draw_string(buffer, 20, 50);

  if (colorSearch)
  {
    return frequencyDistribution(colorCounter);
  }
  return 0;
}

// Move bis Linie mit Rotationssensoren
int moveRotLine(int startSpeed, int maxSpeed, std::string mode, int endSpeed, bool colorSearch)
{
  cSpeed = startSpeed;
  //int leftdegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  //int rightdegree = ev3_motor_get_counts(motor_right);

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;

  int lineCounter = 0;       // Zähler, wie oft die Linie hintereinander gesehen wurde
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe
  Stopwatch move;
  bool bremst = false;

  while (lineCounter < 1)
  {
    tslp_tsk(1);

    if (lineDetection(mode))
    {
      lineCounter++;
    }
    else
      lineCounter = 0;

    cSpeed = accDec(0, 0, move, startSpeed, 0, maxSpeed, 100, stall, bremst);
    motorCorrection(pGain, 0, iCorrection, cSpeed, resetRightDegree, resetLeftDegree);
    stall.measure(ev3_motor_get_counts(motor_left));
    if (colorSearch)
    {
      colorCounter[colorDetection(LSaussen)]++;
      tslp_tsk(4);
    }
  }

  brakeAtEnd(endSpeed); // Motoren anhalten

  resetRightDegree = ev3_motor_get_counts(motor_right);
  resetLeftDegree = ev3_motor_get_counts(motor_left);
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;

  if (colorSearch)
  {
    return frequencyDistribution(colorCounter);
  }

  return 0;
}

int moveRotLineMediumMotor(int startSpeed, int maxSpeed, std::string mode, int endSpeed, bool colorSearch, motor_port_t motor, int speed, int degree, bool stop)
{
  cSpeed = startSpeed;
  //int leftdegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  //int rightdegree = ev3_motor_get_counts(motor_right);
  ev3_motor_reset_counts(motor);

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;

  int lineCounter = 0;       // Zähler, wie oft die Linie hintereinander gesehen wurde
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe
  Stopwatch move;
  bool bremst = false;

  while (lineCounter < 1) // 1
  {
    tslp_tsk(1);

    if (lineDetection(mode))
    {
      lineCounter++;
    }
    else
      lineCounter = 0;

    if (abs(ev3_motor_get_counts(motor)) > degree)
      ev3_motor_stop(motor, stop);

    cSpeed = accDec(0, 0, move, startSpeed, 0, maxSpeed, 100, stall, bremst);
    motorCorrection(pGain, 0, iCorrection, cSpeed, resetRightDegree, resetLeftDegree);
    stall.measure(ev3_motor_get_counts(motor_left));
    if (colorSearch)
    {
      colorCounter[colorDetection(LSaussen)]++;
      tslp_tsk(4);
    }
  }
  brakeAtEnd(endSpeed);

  resetRightDegree = ev3_motor_get_counts(motor_right);
  resetLeftDegree = ev3_motor_get_counts(motor_left);
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;
}

// Move auf Zeit mit Rotationsssensoren
void moveRotTime(int startSpeed, int maxSpeed, double duration, int endSpeed)
{
  cSpeed = startSpeed;
  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;
  Stopwatch movetime;
  bool bremst = false;

  while (movetime.getTime() <= duration && !stall.detectStall()) // davor ohne stall
  {
    tslp_tsk(2); //1
    cSpeed = accDec(0, 0, movetime, startSpeed, 0, maxSpeed, 100, stall, bremst);
    //motorCorrection(pGain,0,iCorrection,cSpeed,rightdegree,leftdegree);
    ev3_motor_set_power(motor_left, (-1) * cSpeed);
    ev3_motor_set_power(motor_right, cSpeed);
    stall.measure(ev3_motor_get_counts(motor_left));
  }

  brakeAtEnd(endSpeed); // Motoren anhalten

  resetRightDegree = ev3_motor_get_counts(motor_right);
  resetLeftDegree = ev3_motor_get_counts(motor_left);
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;
}

// Linienfolger auf Gradzahl
int linedeg(int maxSpeed, double pGain, double iGain, double speedReductionFactor, int distance, int endSpeed)
{
  distance = distance * wheelConverter;

  int leftdegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  int rightdegree = ev3_motor_get_counts(motor_right);

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  int togo = distance;
  double iCorrection = 0;
  double pCorrection = 0;
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe

  Stopwatch move;

  while (togo > 0)
  {
    togo = distance - abs(ev3_motor_get_counts(motor_right) - rightdegree + ev3_motor_get_counts(motor_left) - leftdegree) / 2;
    pCorrection = ev3_color_sensor_get_reflect(LSl) - ev3_color_sensor_get_reflect(LSr);
    iCorrection = move.getTime() * pCorrection + iCorrection;
    move.reset();
    if (pCorrection == 0)
      iCorrection = 0;
    if (abs(pCorrection) >= 40)
      cSpeed = maxSpeed * speedReductionFactor;
    else if (cSpeed > maxSpeed)
      cSpeed--;
    ev3_motor_set_power(motor_right, cSpeed - pCorrection * pGain - iCorrection * iGain);
    ev3_motor_set_power(motor_left, cSpeed + pCorrection * pGain + iCorrection * iGain);
  }

  brakeAtEnd(endSpeed); // Motoren anhalten

  return frequencyDistribution(colorCounter);
}

// Linienfolger bis Querlinie
int lineline(int maxSpeed, double pGain, double iGain, double speedReductionFactor, int endSpeed)
{
  int lineCounter = 0;
  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);

  double iCorrection = 0;
  double pCorrection = 0;
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe

  Stopwatch move;

  while (lineDetection("blackboth") == false)
  {
    pCorrection = ev3_color_sensor_get_reflect(LSl) - ev3_color_sensor_get_reflect(LSr);
    iCorrection = move.getTime() * pCorrection + iCorrection;
    move.reset();
    if (pCorrection == 0)
      iCorrection = 0;
    if (abs(pCorrection) >= 40)
      cSpeed = maxSpeed * speedReductionFactor;
    else if (cSpeed > maxSpeed)
      cSpeed--;
    ev3_motor_set_power(motor_right, cSpeed - pCorrection * pGain - iCorrection * iGain);
    ev3_motor_set_power(motor_left, cSpeed + pCorrection * pGain + iCorrection * iGain);
  }

  brakeAtEnd(endSpeed); // Motoren anhalten
  return frequencyDistribution(colorCounter);
}

// Abweichung von Linie berechnen und zurückgeben
int alignmentangle(int startSpeed, int maxSpeed, std::string color, int endSpeed)
{
  cSpeed = startSpeed;
  //int leftdegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  //int rightdegree = ev3_motor_get_counts(motor_right);

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  endSpeed = speedLevel(endSpeed);
  StallDetection stall;

  int counter = 0; // Zähler für Schleifendurchläufe
  Stopwatch move;
  bool bremst = false;
  bool angleDirection = false;

  int degreeLeft = 0, degreeRight = 0;
  bool rightSet = false, leftSet = false;

  while (rightSet == false || leftSet == false)
  {
    if (lineDetection(color + "right"))
    {
      if (!rightSet)
      {
        degreeRight = (abs(ev3_motor_get_counts(motor_left)) + abs(ev3_motor_get_counts(motor_right))) / 2;
        rightSet = true;
        if (leftSet == false)
          angleDirection = true;
      }
    }
    if (lineDetection(color + "left"))
    {
      if (!leftSet)
      {
        degreeLeft = (abs(ev3_motor_get_counts(motor_left)) + abs(ev3_motor_get_counts(motor_right))) / 2;
        leftSet = true;
      }
    }

    /*}else if (color == "white")
    {
      if (ev3_color_sensor_get_reflect(LSr) > 50 && rightSet == false)
      {
        degreeRight = (abs(ev3_motor_get_counts(motor_left)) + abs(ev3_motor_get_counts(motor_right))) / 2;
        rightSet = true;
        angleDirection = true;

      }
      if (ev3_color_sensor_get_reflect(LSl) > 50 && leftSet == false)
      {
          degreeLeft = (abs(ev3_motor_get_counts(motor_left)) + abs(ev3_motor_get_counts(motor_right))) / 2;
        leftSet = true;
      }
    }*/

    cSpeed = accDec(0, 0, move, startSpeed, 0, maxSpeed, startSpeed, stall, bremst);
    motorCorrection(pGain, 0, iCorrection, cSpeed, resetRightDegree, resetLeftDegree);
    counter++;
  }

  brakeAtEnd(endSpeed); // Motoren anhalten

  resetRightDegree = ev3_motor_get_counts(motor_right);
  resetLeftDegree = ev3_motor_get_counts(motor_left);
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;

  if (angleDirection == true)
  {
    return ((atan(((degreeLeft - degreeRight) / 360. * wheelDiameter * pi) / lsDistance) * 180 / pi) * (1));
  }
  else
  {
    return (atan(((degreeLeft - degreeRight) / 360. * wheelDiameter * pi) / lsDistance) * 180 / pi);
  }
}

void mediumMotorDeg(motor_port_t motor, int speed, int degree, bool stop)
{
  ev3_motor_reset_counts(motor);
  while (abs(ev3_motor_get_counts(motor)) < degree)
  {
    ev3_motor_set_power(motor, speed);
  }
  ev3_motor_stop(motor, stop);
}

void mediumMotorTime(motor_port_t motor, int speed, int time, bool stop)
{
  Stopwatch mediumMotor;
  while (mediumMotor.getTime() < time)
  {
    ev3_motor_set_power(motor, speed);
  }
  ev3_motor_stop(motor, stop);
}

void twoMediumMotorsTime(int speed_kuhl, int time_kuhl, int speed_greif, int time_greif)
{
  Stopwatch kuhlMotor;
  Stopwatch greifMotor;
  ev3_motor_set_power(kuhlmotor, speed_kuhl);
  ev3_motor_set_power(greifmotor, speed_greif);
  while (kuhlMotor.getTime() < time_kuhl && greifMotor.getTime() < time_greif)
  {
    if (greifMotor.getTime() > time_greif)
      ev3_motor_stop(greifmotor, true);
    if (kuhlMotor.getTime() > time_kuhl)
      ev3_motor_stop(kuhlmotor, true);
  }
}

void old_align(int cSpeed, std::string mode)
{
  cSpeed = speedLevel(cSpeed);
  int rightSpeed = cSpeed;
  int leftSpeed = cSpeed;
  int leftline = 0;
  int rightline = 0;
  int rightmin = 42;
  int rightmax = 58;
  int leftmin = 42;
  int leftmax = 58;
  if (mode == "blue")
  {
    rightmin = 100;
    rightmax = 130;
  }
  Stopwatch align;
  bool linenotseen = true;

  while ((leftline < 10 || rightline < 10) && (align.getTime() < 800 || linenotseen == true))
  {
    int rightval = lightSensor(LSr, mode);
    if (rightval < rightmin)
    {
      rightline = 0;
      rightSpeed = cSpeed / abs(cSpeed) * 30;
      ev3_motor_set_power(motor_right, -rightSpeed);
      if (linenotseen)
      {
        linenotseen = false;
        align.reset();
      }
    }
    if (rightval > rightmin && rightval < rightmax)
    {
      ev3_motor_set_power(motor_right, 0);
      rightline++;
    }
    if (rightval > rightmax)
    {
      rightline = 0;
      ev3_motor_set_power(motor_right, rightSpeed);
    }
    int leftval = lightSensor(LSl, mode);
    if (leftval < leftmin)
    {
      leftline = 0;
      leftSpeed = cSpeed / abs(cSpeed) * 30;
      ev3_motor_set_power(motor_left, leftSpeed);
      if (linenotseen)
      {
        linenotseen = false;
        align.reset();
      }
    }
    if (leftval > leftmin && leftval < leftmax)
    {
      ev3_motor_set_power(motor_left, 0);
      leftline++;
    }
    if (leftval > leftmax)
    {
      leftline = 0;
      ev3_motor_set_power(motor_left, -leftSpeed);
    }
  }
  brakeAtEnd(0);
  resetRightDegree = ev3_motor_get_counts(motor_right);
  resetLeftDegree = ev3_motor_get_counts(motor_left);
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;
  //std::cout << "old align finished after " << align.getTime() / 1000 << " seconds" << std::endl;
}

// Abbremsen aus einer unbestimmten Geschwindigkeit, Rückgabe der benötigten Strecke
int brake()
{
  int maxSpeed = cSpeed;
  int startpos = abs(ev3_motor_get_counts(motor_left) + (-1) * ev3_motor_get_counts(motor_right)) / 2;
  double localPGain = pGain;
  //int degreeright = ev3_motor_get_counts(motor_right);
  //int degreeleft = ev3_motor_get_counts(motor_left);
  std::cout << "Bremse ab von Geschwindigkeit " << cSpeed << std::endl;
  int distanceToBrake = (cSpeed - minSpeed) / (bfMove * 2);
  int togo = distanceToBrake;
  double iCorrection = 0;
  while (togo >= 0)
  {
    cSpeed = std::max(int(togo * (bfMove * 1.3) + minSpeed), minSpeed);
    motorCorrection(localPGain, 0, iCorrection, cSpeed, resetRightDegree, resetLeftDegree);
    togo = startpos + distanceToBrake - abs(ev3_motor_get_counts(motor_left) + (-1) * ev3_motor_get_counts(motor_right)) / 2;
  }

  brakeAtEnd(0);

  if (maxSpeed > 0)
  {
    resetRightDegree = resetRightDegree + (int)(distanceToBrake);
    resetLeftDegree = resetLeftDegree - (int)(distanceToBrake);
  }
  else
  {
    resetRightDegree = resetRightDegree - (int)(distanceToBrake);
    resetLeftDegree = resetLeftDegree + (int)(distanceToBrake);
  }
  //std::cout << "distanceToBrake:  " << distanceToBrake << " " << std::endl;
  //std::cout << "motor_left:  " << resetLeftDegree << " " << std::endl;
  //std::cout << "motor_right:  " << resetRightDegree << " " << std::endl;

  std::cout << "Needed " << (ev3_motor_get_counts(motor_left) + ev3_motor_get_counts(motor_right)) / 2 - startpos << " to brake" << std::endl;
  return abs(ev3_motor_get_counts(motor_left) + (-1) * ev3_motor_get_counts(motor_right)) / 2 - startpos;
}

// ACTION BLOCKS LINE FOLLOWER - NOT USABLE - WRONG LOGIC
//-----------------------------------------------------------------------------
/*
// Linienfolger auf Gradzahl über Differenz - nicht nutzbar
int linedeg(int maxSpeed, double pGain, double iGain, double speedReductionFactor, int distance, bool brake)
{
  distance = distance * wheelConverter;

  int leftdegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  int rightdegree = ev3_motor_get_counts(motor_right);

  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  int togo = distance;
  double iCorrection = 0;
  double pCorrection = 0;
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe

  Stopwatch move;
  while (togo > 0)
  {
    togo = distance - abs(ev3_motor_get_counts(motor_right) - rightdegree + ev3_motor_get_counts(motor_left) - leftdegree) / 2;
    pCorrection = ev3_color_sensor_get_reflect(LSl) - ev3_color_sensor_get_reflect(LSr);
    iCorrection = move.getTime() * pCorrection + iCorrection;
    move.reset();
    if (pCorrection == 0)
      iCorrection = 0;
    if (abs(pCorrection) >= 40)
      cSpeed = maxSpeed * speedReductionFactor;
    else if (cSpeed > maxSpeed)
      cSpeed--;
    ev3_motor_set_power(motor_right, cSpeed - pCorrection * pGain - iCorrection * iGain);
    ev3_motor_set_power(motor_left, cSpeed + pCorrection * pGain + iCorrection * iGain);
  }

  brakeAtEnd(brake); // Motoren anhalten
  return frequencyDistribution(colorCounter);
}

// Linienfolger bis Querlinie über Differenz - nicht nutzbar
int lineline(int maxSpeed, double pGain, double iGain, double speedReductionFactor, bool brake)
{
  int lineCounter = 0;
  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);

  double iCorrection = 0;
  double pCorrection = 0;
  int colorCounter[8] = {0}; // für Farberkennung der Schiffe

  Stopwatch move;
  while (lineDetection("blackboth") == false)
  {
    pCorrection = ev3_color_sensor_get_reflect(LSl) - ev3_color_sensor_get_reflect(LSr);
    iCorrection = move.getTime() * pCorrection + iCorrection;
    move.reset();
    if (pCorrection == 0)
      iCorrection = 0;
    if (abs(pCorrection) >= 40)
      cSpeed = maxSpeed * speedReductionFactor;
    else if (cSpeed > maxSpeed)
      cSpeed--;
    ev3_motor_set_power(motor_right, cSpeed - pCorrection * pGain - iCorrection * iGain);
    ev3_motor_set_power(motor_left, cSpeed + pCorrection * pGain + iCorrection * iGain);
  }

  brakeAtEnd(brake); // Motoren anhalten
  return frequencyDistribution(colorCounter);
}
*/

// ACTION BLOCKS MIT GYRO - NOT NEEDED ANY MORE
//-----------------------------------------------------------------------------

// Spezielle Drehung auf der Stelle mit Verhältnis, sodass der Roboter am Ende seitlich versetzt steht
/*void paulturn_gyro(int startSpeed, int maxSpeed, double angle, bool brake)
{
  int rightdegree = 0;
  int leftdegree = 0;
  if(cSpeed==0) {
    ev3_motor_reset_counts(motor_left);
    ev3_motor_reset_counts(motor_right);
  }
  else {
  leftdegree = ev3_motor_get_counts(motor_left); // Rotationssensoren zurücksetzen
  rightdegree = ev3_motor_get_counts(motor_right);
  }
  int gyroStart = measureGyro(); // Startwinkel definieren
  angle = gyroDeviationFactor * angle;
  int togo = abs(angle);
  cSpeed = startSpeed;
  maxSpeed = speedLevel(maxSpeed);
  cSpeed = speedLevel(cSpeed);
  StallDetection stall;

  int pCorrection = 0;
  Stopwatch move;
  bool bremst = false;

  while (togo > 0)
  {
    togo = abs(angle) - abs(measureGyro() - gyroStart);
    cSpeed = accDec(togo,bfTurn2,move,startSpeed,minSpeed*1.,maxSpeed,brake,stall,bremst);
    stall.measure(abs(ev3_motor_get_counts(motor_left)));
    pCorrection = (ev3_motor_get_counts(motor_right)-rightdegree) * 1.2 + ev3_motor_get_counts(motor_left)-leftdegree;
   ev3_motor_set_power(motor_left,cSpeed - pCorrection);
   ev3_motor_set_power(motor_right,-cSpeed - pCorrection);
  }

  brakeAtEnd(brake); // Motoren anhalten
  resetGyro = angle + resetGyro;
}*/