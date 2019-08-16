#pragma once

#include "ev3api.h"
#include "app.h"

//#include <thread>
//#include <assert.h>
#include <string.h>
#include <vector>
//#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <fstream> //logfile
//#include <stdio.h> //date
//#include <stdlib.h> //date
#include "timer.h" //custom timer
#include "stalldetection.h"
#include <iostream>


//-----------------------------------------------------------------------------

// SUPPORT BLOCKS
int getRGB(sensor_port_t port, int color);
int getHTRGB(sensor_port_t sensor, int mode);
int speedLevel(int level);
//void setMotorDirection(int v);
//int measureGyro();
//void resetGyrosensor(int resetvalue);
void waitForButton();
int colorDetection(sensor_port_t sensor);
int colorDetection_rgb(sensor_port_t sensor);
int colorDetection_rgb_food(sensor_port_t sensor);
int colorDetection_rgb_lager(sensor_port_t sensor);
int colorDetection_rgb_ship(sensor_port_t sensor);
int colorDetection_rgb_shipbw(sensor_port_t sensor);
//int colorDetection_rgbw_food();
//int colorDetection_rgbw_ship();
//int colorDetection_rgbb_food();
int lightSensor(sensor_port_t sensor, std::string mode);

//int colorDetection_adv();
//void showColor(int color);
void logColor();
void brakeAtEnd(int endSpeed);
int frequencyDistribution(int colorCounter[]);
bool lineDetection(std::string mode);
//void gyroCorrection(double _pGain, double _iGain, double &iCorrection, double cSpeed);
void motorCorrection(double _pGain, double _iGain, double &iCorrection, double cSpeed, int rightreset, int leftreset);
double accDec(int togo, double brakeFactor, Stopwatch move, double startSpeed, double minSpeed, int maxSpeed, int endSpeed, StallDetection &stall, bool &bremst);
void nationalAnthem();
void initializeSpeeds(int &speed1, int &speed2, int &speed3);
void updateRotationSensors(int distance, int maxSpeed);

// ACTION BLOCKS
void turn1_gyro(motor_port_t turnMotor, int startSpeed, int maxSpeed, double angle, int endSpeed);
void turn1_rot(motor_port_t turnMotor, int startSpeed, int maxSpeed, double angle, int endSpeed);
void turn2_gyro(int startSpeed, int maxSpeed, double angle, int endSpeed);
void turn2_rot(int startSpeed, int maxSpeed, double angle, int endSpeed);
void paulturn_rot(int startSpeed, int maxSpeed, double angle, double leftratio, double rightratio, int endSpeed);
void paulturn_gyro(int startSpeed, int maxSpeed, double angle, int endSpeed);
int movedeg(int startSpeed, int maxSpeed, int distance, int endSpeed, bool colorSearch);
int moveRotDeg(int startSpeed, int maxSpeed, int distance, int endSpeed, bool colorSearch);
void moveRotDegNoSync(int startSpeed, int maxSpeed, int distance, int endSpeed);
int moveRotDegRatio(int startSpeed, int maxSpeed, int distance, double rratio, double lratio, int endSpeed, bool colorSearch);
int moveRotDegRatioNoSync(int startSpeed, int maxSpeed, int distance, double rratio, double lratio, int endSpeed, bool colorSearch);
int moveRotDegMediumMotor(int startSpeed, int maxSpeed, int distance, int endSpeed, bool colorSearch, motor_port_t motor, int speed, int degree, bool stop);
int moveRotDegMediumMotorTime(int startSpeed, int maxSpeed, int distance, int endSpeed, bool colorSearch, motor_port_t motor, int speed, int time, bool stop);
int moveline(int startSpeed, int maxSpeed, std::string mode, int endSpeed, bool colorSearch);
int moveRotLine(int startSpeed, int maxSpeed, std::string mode, int endSpeed, bool colorSearch);
int moveRotLineMediumMotor(int startSpeed, int maxSpeed, std::string mode, int endSpeed, bool colorSearch, motor_port_t motor, int speed, int degree, bool stop);
void movetime(int startSpeed, int maxSpeed, double duration, int endSpeed);
void moveRotTime(int startSpeed, int maxSpeed, double duration, int endSpeed);
int linedeg(int maxSpeed, double pGain, double iGain, double speedReductionFactor, int distance, int endSpeed);
int lineline(int maxSpeed, double pGain, double iGain, double speedReductionFactor, int endSpeed);
int alignmentangle(int startSpeed, int maxSpeed, std::string color, int endSpeed);
void old_align(int cSpeed, std::string mode);
void mediumMotorDeg(motor_port_t motor, int speed, int degree, bool stop);
void mediumMotorTime(motor_port_t motor, int speed, int time, bool stop);
void twoMediumMotorsTime(int speed_kuhl, int time_kuhl, int speed_greif, int time_greif);
int brake();

// Global Vars
extern motor_port_t motor_left;
extern motor_port_t motor_right;
extern motor_port_t greifmotor;
extern motor_port_t kuhlmotor;
extern sensor_port_t LSinnen;
extern sensor_port_t LSaussen;
extern sensor_port_t LSr;
extern sensor_port_t LSl;
extern double cSpeed;
extern const int minSpeed;
extern int resetGyro;
extern int gyroend;
extern int resetLeftDegree;
extern int resetRightDegree;
extern const double gyroDeviationFactor;
extern const double bfMove;
extern const double bfTurn1;
extern const double bfTurn2;
extern const double lsDistance;
extern const double pi;
extern const double wheelDiameter;
extern const double wheelConverter;

extern double iCorrection;
extern double pGain;
extern double iGain;
extern const int averageWhite;
extern const int averageBlueLeft;
extern const int averageBlueRight;
extern double batteryFactor;
extern std::vector<int> detectedColors;