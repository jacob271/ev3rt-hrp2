#include "blocks.h"
#include "timer.h"
#include "stalldetection.h"

// SUPPORT BLOCKS
//-----------------------------------------------------------------------------

// get RGB value of EV3 color sensor
int getRGB(sensor_port_t port, int color)
{
  rgb_raw_t val;
  ev3_color_sensor_get_rgb_raw(port, &val);
  switch (color)
  {
  case 1:
    return val.r;
  case 2:
    return val.g;
  case 3:
    return val.b;
  }
  return 0;
}

// get RGB value of HT NXT color sensor
int getHTRGB(sensor_port_t sensor, int mode)
{
  if (mode == 0)
  {
    uint8_t color = 0;
    bool_t valC = ht_nxt_color_sensor_measure_color(sensor, &color);
    assert(valC);
    return int(color);
  }
  else
  {
    rgb_raw_t rgb;
    bool_t valRgb = ht_nxt_color_sensor_measure_rgb(sensor, &rgb);
    assert(valRgb);
    switch (mode)
    {
    case 1:
      return rgb.r;
    case 2:
      return rgb.g;
    case 3:
      return rgb.b;
    }
  }
  return 0;
}

// Geschwindigkeiten konvertieren
int speedLevel(int level)
{
  switch (abs(level))
  {
  case 1:
    return (10 * batteryFactor * (level / abs(level))); //Anfahren speed
  case 2:
    return (50 * batteryFactor * (level / abs(level)));
  case 3:
    return (70 * batteryFactor * (level / abs(level))); //Standard Drive Speed
  case 4:
    return (100 * batteryFactor * (level / abs(level))); //fast drive speed
  case 5:
    return (80 * batteryFactor * (level / abs(level))); //turn2 speed
  default:
    return (level * batteryFactor);
  }
}

// Rotationsssensorkorrektur für movees
void motorCorrection(double _pGain, double _iGain, double &iCorrection, double cSpeed, int rightreset, int leftreset)
{
  double pCorrection;
  pCorrection = ((ev3_motor_get_counts(motor_left) - leftreset) + (ev3_motor_get_counts(motor_right) - rightreset)) * (abs(_pGain * cSpeed) + 0.4);
  ev3_motor_set_power(motor_left, (-1) * (cSpeed + pCorrection));
  ev3_motor_set_power(motor_right, cSpeed - pCorrection);
}

double accDec(int togo, double brakeFactor, Stopwatch move, double startSpeed, double minSpeed, int maxSpeed, int endSpeed, StallDetection &stall, bool &bremst)
{
  int accFactor = 4;
  //Abbremsen am Ende der Bewegung
  if ((togo * brakeFactor + minSpeed <= abs(cSpeed)) && (abs(cSpeed) > abs(endSpeed)))
  {
    bremst = true;
    cSpeed = std::max(togo * brakeFactor + minSpeed, minSpeed);
    if (maxSpeed < 0)
      cSpeed = cSpeed * (-1);
  }
  //Beschleunigen am Anfnag der Bewegung
  else if (abs(cSpeed) < abs(maxSpeed) && bremst == false)
  {
    if (maxSpeed < 0)
    {
      cSpeed = std::max(maxSpeed * 1., startSpeed - move.getTime() / accFactor);
    }
    else
    {
      cSpeed = std::min(maxSpeed * 1., startSpeed + move.getTime() / accFactor);
    }
    stall.resetStall();
  }
  //Abbremsen am Ende der Bewegung
  else if (abs(cSpeed) > abs(maxSpeed) && bremst == false)
  {
    if (maxSpeed < 0)
    {
      cSpeed = std::max((maxSpeed * 1.), (startSpeed + move.getTime() / accFactor));
    }
    else
    {
      cSpeed = std::min(maxSpeed * 1., startSpeed - move.getTime() / accFactor);
    }
  }
  return cSpeed;
}

// Am Ende einer Bewegung bei Bedarf anhalten
void brakeAtEnd(int endSpeed)
{
  if (endSpeed == 0)
  {
    ev3_motor_set_power(motor_left, 0);
    ev3_motor_stop(motor_left, true);
    ev3_motor_set_power(motor_right, 0);
    ev3_motor_stop(motor_right, true);
    cSpeed = 0;
  }
  else
  {
    cSpeed = endSpeed;
  }
}

// Auswerten der Häufigkeitsverteilung für das Erkennen von Schiffen und Lagern
int frequencyDistribution(int colorCounter[])
{
  int temp = 1;
  for (int i = 2; i < 8; i++)
  {
    if (colorCounter[i] > colorCounter[temp] && colorCounter[i] > 1)
      temp = i;
    std::cout << i << "-" << colorCounter[i] << " | ";
  }

  std::cout << "Detected color is " << temp << " and was seen " << colorCounter[temp] << " times." << std::endl;
  return temp;
}

int lightSensor(sensor_port_t sensor, std::string mode)
{
  if (mode == "blue")
    return getRGB(sensor, 3);
  else if (mode == "black")
    return ev3_color_sensor_get_reflect(sensor);
  return 0;
}

// Meldet zurück, ob die Linie nach gewünschten Modus gesehen wurde
bool lineDetection(std::string mode)
{
  if (mode == "blackright")
    return ev3_color_sensor_get_reflect(LSr) < 40; // 25
  else if (mode == "blackleft")
    return ev3_color_sensor_get_reflect(LSl) < 40; // 25
  else if (mode == "green")
    return ev3_color_sensor_get_reflect(LSl) < 40;
  else if (mode == "blueright")
    return getRGB(LSr, 3) < averageBlueRight;
  else if (mode == "blueleft")
    return getRGB(LSl, 3) < averageBlueLeft;
  else if (mode == "blackboth")
    return ev3_color_sensor_get_reflect(LSr) < 40 && ev3_color_sensor_get_reflect(LSl) < 40;
  else if (mode == "blackone")
    return ev3_color_sensor_get_reflect(LSr) < 40 || ev3_color_sensor_get_reflect(LSl) < 40;
  else if (mode == "whiteboth")
    return ev3_color_sensor_get_reflect(LSr) > 50 || ev3_color_sensor_get_reflect(LSl) > 50;
  return false;
}

// Warten, bis die Taste nach links gedrückt wurde
void waitForButton()
{
  while (!ev3_button_is_pressed(ENTER_BUTTON))
  {
  }
}

// Konvertierung der HiTechnic Farbwerte in normale EV3 Farbwerte
int colorDetection(sensor_port_t sensor)
{
  int htColor = getHTRGB(sensor, 0);
  std::cout << htColor << " ";
  switch (htColor)
  {
  case 0:
    return 0;
  case 1:
    return 0;
  case 2:
    return 2;
  case 3:
    return 2;
  case 4:
    return 3;
  case 5:
    return 4;
  case 6:
    return 4;
  case 7:
    return 5;
  case 8:
    return 5;
  case 9:
    return 5;
  case 10:
    return 5;
  //case 11:
  //return 2;
  case 13:
    return 3;
  case 14:
    return 5;
  case 17:
    return 0; //5
  default:
    //std::cout << "Keine Farbe erkannt: " << htColor << std::endl; ####
    return 0;
  }
}

// Kompliziertere Farbwerterkennung
int colorDetection_rgb(sensor_port_t sensor)
{
  rgb_raw_t rgb;
  bool_t valRgb = ht_nxt_color_sensor_measure_rgb(sensor, &rgb);
  assert(valRgb);

  int red = rgb.r;
  int green = rgb.g;
  int blue = rgb.b;

  tslp_tsk(7);

  /*  int htColor = int(color);
  tslp_tsk(4);*/

  std::cout << "C:" << rgb.r << " " << rgb.g << " " << rgb.b << " "; // << "R:" << color;
  if (red < 5 && blue < 5 && green < 5)
    return 0;
  if (red > blue && red > green && red > 10)
  {
    if (red > green * 2) //1.8
      return 5;
    else
    {
      //if (blue > 10)
      //return 6;
      //else
      return 4;
    }
  }
  if (blue > green && blue > red && blue > 10)
    return 2;
  //if ( htColor == 4 || green > blue && green > red && green > 7)
  if (green > blue && green > red && green > 5)
    return 3;
  return -1;
}

int colorDetection_rgb_lager(sensor_port_t sensor){
  rgb_raw_t rgb;
  bool_t valRgb = ht_nxt_color_sensor_measure_rgb(sensor, &rgb);
  assert(valRgb);

  int red = rgb.r;
  int green = rgb.g;
  int blue = rgb.b;

  tslp_tsk(7);
    std::cout << "C:" << rgb.r << " " << rgb.g << " " << rgb.b << " "; // << "R:" << color;

  if (blue < 29 && green < 29 && red < 29)
    return 0;
  if (blue > 130 && green > 130 && red > 130)
    return 7;
  if (green > 100)
    return 4;
  if (red > 100)
    return 5;
  if (blue > 80)
    return 2;
  if ( (green - blue) > 8 && (green - red) > 10)
    return 3;
  else
    return 1;
}

int colorDetection_rgb_shipbw(sensor_port_t sensor){
  rgb_raw_t rgb;
  bool_t valRgb = ht_nxt_color_sensor_measure_rgb(sensor, &rgb);
  assert(valRgb);

  int red = rgb.r;
  int green = rgb.g;
  int blue = rgb.b;

  tslp_tsk(7);

  std::cout << "C:" << rgb.r << " " << rgb.g << " " << rgb.b << " "; // << "R:" << color;

  if (blue < 7 && green < 7 && red < 7)
    return 0;
  if (blue > 100 && green > 100 && red > 100)
    return 7;
  if (green > 90)
    return 4;
  if (red > 50)
    return 5;
  if (blue > 50)
    return 2;
  if ( (green - blue) > 13 && (green - red) > 13)
   return 3;
  else
    return 1;
}

int colorDetection_rgb_ship(sensor_port_t sensor){
  rgb_raw_t rgb;
  bool_t valRgb = ht_nxt_color_sensor_measure_rgb(sensor, &rgb);
  assert(valRgb);

  int red = rgb.r;
  int green = rgb.g;
  int blue = rgb.b;

  tslp_tsk(7);

  std::cout << "C:" << rgb.r << " " << rgb.g << " " << rgb.b << " "; // << "R:" << color;

  if (blue < 29 && green < 29 && red < 29)
    return 0;
  if (blue > 200 && green > 200 && red > 200)
    return 7;
  if (green > 120)
    return 4;
  if (red > 100)
    return 5;
  if (blue > 100)
    return 2;
  if ( (green - blue) > 15 && (green - red) > 15)
    return 3;
  else
    return 1;
}

int colorDetection_rgb_food(sensor_port_t sensor)
{
  rgb_raw_t rgb;
  bool_t valRgb = ht_nxt_color_sensor_measure_rgb(sensor, &rgb);
  assert(valRgb);

  int red = rgb.r;
  int green = rgb.g;
  int blue = rgb.b;

  tslp_tsk(7);

  /*uint8_t color = 0;
  bool_t valC = ht_nxt_color_sensor_measure_color(sensor ,&color);
  assert(valC);
  
  int htColor = int(color);
  tslp_tsk(4);*/

  std::cout << "C:" << rgb.r << " " << rgb.g << " " << rgb.b << " "; // << "R:" << color;
  if (red < 30 && blue < 30 && green < 30)
    return 0;
  if (red > blue && red > green && red > 15)
  {
    if (red > green * 2) //1.8
      return 5;
    else
    {
      //if (blue > 10)
      //return 6;
      //else
      return 4;
    }
  }
  if (blue > green && blue > red && blue > 15)
    return 2;
  //if ( htColor == 4 || green > blue && green > red && green > 7)
  if (green > blue && green > red && green > 7)
    return 3;
  return -1;
}

void nationalAnthem()
{
  for (int i = 0; i < 2; i++)
  {
    ev3_speaker_play_tone(NOTE_F4, 750);
    tslp_tsk(750);
    ev3_speaker_play_tone(NOTE_G4, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_A4, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_G4, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_AS4, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_A4, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_G4, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_E4, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_F4, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_D5, 500);
    tslp_tsk(500);
    return;
    ev3_speaker_play_tone(NOTE_C5, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_AS4, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_A4, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_G4, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_A4, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_F4, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_C5, 1000);
    tslp_tsk(1000);
  }
  ev3_speaker_play_tone(NOTE_G4, 500);
  tslp_tsk(500);
  ev3_speaker_play_tone(NOTE_A4, 500);
  tslp_tsk(500);
  ev3_speaker_play_tone(NOTE_G4, 250);
  tslp_tsk(250);
  ev3_speaker_play_tone(NOTE_E4, 250);
  tslp_tsk(250);
  ev3_speaker_play_tone(NOTE_C4, 500);
  tslp_tsk(500);
  ev3_speaker_play_tone(NOTE_AS4, 500);
  tslp_tsk(500);
  ev3_speaker_play_tone(NOTE_A4, 500);
  tslp_tsk(500);
  ev3_speaker_play_tone(NOTE_G4, 250);
  tslp_tsk(250);
  ev3_speaker_play_tone(NOTE_E4, 250);
  tslp_tsk(250);
  ev3_speaker_play_tone(NOTE_C4, 500);
  tslp_tsk(500);
  ev3_speaker_play_tone(NOTE_C5, 500);
  tslp_tsk(500);
  ev3_speaker_play_tone(NOTE_AS4, 500);
  tslp_tsk(500);
  ev3_speaker_play_tone(NOTE_A4, 750);
  tslp_tsk(750);
  ev3_speaker_play_tone(NOTE_A4, 250);
  tslp_tsk(250);
  ev3_speaker_play_tone(NOTE_B4, 500);
  tslp_tsk(500);
  ev3_speaker_play_tone(NOTE_B4, 250);
  tslp_tsk(250);
  ev3_speaker_play_tone(NOTE_C5, 250);
  tslp_tsk(250);
  ev3_speaker_play_tone(NOTE_C5, 1000);
  tslp_tsk(1000);
  for (int i = 0; i < 2; i++)
  {
    ev3_speaker_play_tone(NOTE_F5, 750);
    tslp_tsk(750);
    ev3_speaker_play_tone(NOTE_E5, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_E5, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_D5, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_C5, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_D5, 750);
    tslp_tsk(750);
    ev3_speaker_play_tone(NOTE_C5, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_C5, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_AS4, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_A4, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_G4, 750);
    tslp_tsk(750);
    ev3_speaker_play_tone(NOTE_A4, 125);
    tslp_tsk(125);
    ev3_speaker_play_tone(NOTE_AS4, 125);
    tslp_tsk(125);
    ev3_speaker_play_tone(NOTE_C5, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_D5, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_AS4, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_G4, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_F4, 500);
    tslp_tsk(500);
    ev3_speaker_play_tone(NOTE_A4, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_G4, 250);
    tslp_tsk(250);
    ev3_speaker_play_tone(NOTE_F4, 1000);
    tslp_tsk(1000);
  }
}

void initializeSpeeds(int &speed1, int &speed2, int &speed3){
  speed1=speedLevel(speed1);
  speed2=speedLevel(speed2);
  speed3=speedLevel(speed3);
}

void updateRotationSensors(int distance, int maxSpeed){
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
}

/*
int colorDetection_rgbw_food()
{
  int htColor = getHTRGB(LSaussen, 0);
  int red = getHTRGB(LSaussen, 1);
  int green = getHTRGB(LSaussen, 2);
  int blue = getHTRGB(LSaussen, 3);
  detectedColors.push_back(red);
  detectedColors.push_back(green);
  detectedColors.push_back(blue);
  detectedColors.push_back(htColor);
  if (red > 100 && blue > 100 && green > 100)
    return 6;
  if (red < 5 && blue < 5 && green < 5)
    return 0;
  if (red > blue && red > green && red > 10)
    if (red > green * 1.8)
      return 5;
    else
      return 4;
  if (blue > green && blue > red && blue > 10)
    return 2;
  if (green > blue && green > red)
    return 3;
  return 0;
}

int colorDetection_rgbw_ship()
{
  int htColor = getHTRGB(LSaussen, 0);
  int red = getHTRGB(LSaussen, 1);
  int green = getHTRGB(LSaussen, 2);
  int blue = getHTRGB(LSaussen, 3);
  detectedColors.push_back(red);
  detectedColors.push_back(green);
  detectedColors.push_back(blue);
  detectedColors.push_back(htColor);
  if (htColor == 14 || htColor == 17)
    return 6;
  if (red < 5 && blue < 5 && green < 5)
    return 0;
  if (red > blue && red > green && red > 10)
    if (red > green * 1.8)
      return 5;
    else
      return 4;
  if (blue > green && blue > red && blue > 10)
    return 2;
  if (green > blue && green > red)
    return 3;
  return 0;
}

int colorDetection_rgbb_food()
{
  int htColor = getHTRGB(LSaussen, 0);
  int red = getHTRGB(LSaussen, 1);
  int green = getHTRGB(LSaussen, 2);
  int blue = getHTRGB(LSaussen, 3);
  detectedColors.push_back(red);
  detectedColors.push_back(green);
  detectedColors.push_back(blue);
  detectedColors.push_back(htColor);
  int rgbMax = std::max(std::max(red, green), blue);
  if ((rgbMax - ((green + red + blue) / 3)) < 10)
    return 1; // muss 1 sein
  if (red < 5 && blue < 5 && green < 5)
    return 0;
  if (red > blue && red > green && red > 10)
    if (red > green * 1.8)
      return 5;
    else
      return 4;
  if (blue > green && blue > red && blue > 10)
    return 2;
  if (green > blue && green > red)
    return 3;
  return 0;
}
*/

void logColor()
{
  for (int i = 0; i < detectedColors.size(); i += 4)
  {
    //std::cout << " R" << detectedColors[i] << "G" << detectedColors [i+1] << "B" << detectedColors[i+2] << "C" << detectedColors [i+3]; ####
    if (detectedColors[i + 4] == 999)
    {
      //std::cout << " ship done" << std::endl;####
      i++;
    }
  }
  detectedColors.clear();
}

// SUPPORT BLOCKS WITH GYRO - NOT NEEDED ANYMORE
//-----------------------------------------------------------------------------

// Gyrowert messen, der vom manuellen Reset bereinigt ist
/*int measureGyro()
{
  return (ev3_gyro_sensor_get_angle(gyro) + resetGyro);
}*/

// Gyrowert zurücksetzen
/*void resetGyrosensor (int resetvalue = 0){
    resetGyro = ev3_gyro_sensor_get_angle(gyro) + resetvalue;
}*/

// Gyrokorrektur für movees
/*void gyroCorrection(double _pGain, double _iGain, double &iCorrection, double cSpeed)
{
  double pCorrection = measureGyro() * _pGain;
  if (pCorrection == 0)
    iCorrection = 0;
  else
    iCorrection += (pCorrection / _pGain) * _iGain;
    char buffer[10];
  itoa (cSpeed,buffer,10);
  ev3_lcd_draw_string(buffer,0,30);
 ev3_motor_set_power(motor_left, cSpeed + pCorrection + iCorrection); //-100.
 ev3_motor_set_power(motor_right, cSpeed - pCorrection - iCorrection);
}*/

// SUPPORT BLOCKS WITH HT SENSOR HELLIGKEIT - NOT WORKING WITH EV3RT
//-----------------------------------------------------------------------------

//Funktioniert aktuell nicht, da Helligkeitswert vorausgesetzt!
/*int colorDetection_adv(){
  int htColor = getHTRGB(0);
  std::cout << htColor << " ";
  switch (htColor)
  {
  case 0:
    if (getHTRGB(4) > 20 && htsensor.value(0) + htsensor.value(1) + htsensor.value(2) >= 25)
      return 3;
    else
      return 0;
  case 1:
    return 5;
  case 2:
    return 2;
  case 3:
    std::cout << " g:" << htsensor.value(2) << " b:" << htsensor.value(3);
    if (htsensor.value(2) >= htsensor.value(3))
      return 3;
    else
      return 2;
  case 4:
    return 3;
  case 5:
    return 4;
  case 6:
    return 4;
  case 7:
    return 5;
  case 8:
    return 5;
  case 9:
    return 5;
  case 10:
    return 5;
  //case 11:
    //return 2;
  case 13:
    return 3;
  case 14:
    return 3;
  case 17:
    return 5;
  default:
    std::cout << "Keine Farbe erkannt: " << htColor << std::endl;
    return 0;
  }
}*/
