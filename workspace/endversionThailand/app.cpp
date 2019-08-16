 /**
 * This sample program balances a two-wheeled Segway type robot such as Gyroboy in EV3 core set.
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */

#pragma once
//#include <thread>
#include <chrono>
#include <cstdio>

#include <string.h>
#include <stdlib.h>
#include "blocks.h"
#include "timer.h"

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

//Ports
motor_port_t motor_left = EV3_PORT_A;
motor_port_t kuhlmotor = EV3_PORT_B;
motor_port_t greifmotor = EV3_PORT_C;
motor_port_t motor_right = EV3_PORT_D;
sensor_port_t LSinnen = EV3_PORT_4;
sensor_port_t LSr = EV3_PORT_2;
sensor_port_t LSl = EV3_PORT_1;
sensor_port_t LSaussen = EV3_PORT_3;

//Speeds
double cSpeed = 0;
const int minSpeed = 25;

//const std::vector<bool> xCoolerPosition = {true, true, true, false, true, false, true};
//const std::vector<bool> yCoolerPosition = {true, true, false, false, true, true, true};

// Variables for Moves
double iCorrection = 0;
double pGain = 0.02; //0.017
double iGain = 0.0;  //0.02
int resetGyro = 0;
int resetRightDegree = 0;
int resetLeftDegree = 0;
const double gyroDeviationFactor = 1.01112; // 1.00556; //1.01111
const double bfMove = 0.2;                  // 0.1   6
const double bfTurn1 = 0.5;                 // 9
const double bfTurn2 = 0.4;                 // 0.3

// Variables for Linefollower
const int averageWhite = 50;
const int averageBlueLeft = 40;
const int averageBlueRight = 100;
const bool competitionMode = false;

// Constants for vertical line alignment
const double lsDistance = 11.3; //11.7;
const double pi = 3.14159265358979324;
const double wheelDiameter = 6.24; //6.24;
const double wheelConverter = 6.24 / wheelDiameter;
const double wheelLSdist = 3.6 * 2 * (360 / (6.24 * pi));

// Degrees for greifmotor
const int up = 230;
const int down = -230;
const int middle = -115; // 120

//Speeds for medium motors
const int fast = 100;
const int medium = 50;
const int slow = 50;

// Distances for kuhlmotor
const int out = 180;
const int in = -180; //-180;

const int shipDistance = 340; //330
const int wallDistance = 0;   //6

double batteryFactor = 1;
// Variables for main
int color;
int scannedLager = 0;
int shipColors[6] = {-1, -1, -1, -1, -1, -1};
int foodColors[3] = {0, 0, 0};
bool firstShip = true;
int currentFoodPosition = 0;
int currentFood = 0;
int robotShipPosition = 0;
int nextShipPosition = -1;
int shipCounter = 0;
int shipsFound = 0;
bool directionSud = false;
bool justScanned = true;
bool gapFound = false;
std::vector<int> detectedColors;

void align(int startSpeed, int maxSpeed, std::string color)
{
    int angle = alignmentangle(startSpeed, maxSpeed, color, 0);
    if ((angle < 0 && maxSpeed > 0) || (angle > 0 && maxSpeed < 0))
    {
        turn1_rot(motor_right, 1, 4, abs(angle), 0);
    }
    else
    {
        turn1_rot(motor_left, 1, 4, abs(angle), 0);
    }
}

void absetzen()
{
    moveRotDeg(cSpeed, 2, 70, 3, false);
    moveRotLine(cSpeed, 3, "blueleft", 3, false);
    moveRotDegMediumMotor(3, 80, 105, 0, false, greifmotor, 50, 40, true);
    tslp_tsk(100);
    moveRotDeg(0, -60, 5, -60, false);
    moveRotDegMediumMotor(cSpeed, -60, 275, -60, false, kuhlmotor, 15, 85, true); //265    return;
    /*
    moveRotDeg(cSpeed, 2, 70, 70, false);
    moveRotLine(cSpeed, 3, "blueleft", 3, false);
    moveRotDegMediumMotor(3, 80, 105, 0, false, greifmotor, 50, 40, true);
    tslp_tsk(100);
    moveRotDeg(0, -3, 40, -3, false);
    moveRotDegMediumMotor(0, -3, 245, -3, false, kuhlmotor, 30, 85, true); //265
    */
}

void scanLager()
{
    colorDetection_rgb_food(LSinnen);
    tslp_tsk(10);
    color = colorDetection_rgb_food(LSinnen);

    if (color > 1 || gapFound == true)
    {

        currentFood++;
        // Falls zum 2. Mal nichts gefunden, prüfe, ob sich die Farbe aus den Schiffen ableiten lässt
        if (color <= 1)
        {
            for (int i = 0; i < 6; i++)
                if (shipColors[i] > 1)
                {
                    bool colorfound = false;
                    for (int j = 0; j < 3; j++)
                    {
                        if (foodColors[j] == shipColors[i])
                            colorfound = true;
                    }
                    if (colorfound == false)
                    {
                        color = shipColors[i];
                        std::cout << "Again no color... :-( Calculated color depending on ship is: " << color << std::endl;
                    }
                }
        }
        // Wenn nicht möglich, nimm höchstmögliche Zahl
        if (color <= 1)
        {
            for (int i = 2; i < 6; i++)
            {
                bool colorfound = false;
                for (int j = 0; j < 3; j++)
                {
                    if (foodColors[j] == i)
                        colorfound = true;
                }
                if (colorfound == false)
                {
                    color = i;
                    std::cout << "Again no color... :-( Calculated color depending on highest number is: " << color << std::endl;
                }
            }
        }
        foodColors[currentFood - 1] = color;
        //mediumMotorTime(kuhlmotor, 50, 300, true);
        //mediumMotorTime(greifmotor, -100, 350, true);
        twoMediumMotorsTime(50, 350, -100, 350);
        //twoMediumMotorsTime(100,200,-100,200);
    }
    char msgbuf[10];
    sprintf(msgbuf, "Color: %d", color);
    ev3_lcd_draw_string(msgbuf, 10, 10);
    std::cout << "LagerColor: " << color << " " << std::endl;
    scannedLager++;
}

void autoFindShips()
{
    int shipLocation = -1;
    for (int i = 0; i < sizeof(shipColors) / sizeof(int); i++)
    { // Prüfen, ob es ein Schiff gibt, das die selbe Farbe hat
        if (shipColors[i] == color)
            shipLocation = i;
    }
    if (shipLocation == -1)
    { // Wenn nein, die höchste leere Schiffsposition finden und ihm die Farbe des Essens zuweisen
        for (int i = sizeof(shipColors) / sizeof(int) - 1; i >= 0; i--)
        {
            if (shipColors[i] < 2)
            {
                shipColors[i] = color;
                break;
            }
        }
    }
    for (int i = 0; i < 6; i++)
        std::cout << shipColors[i] << " ";
    std::cout << std::endl;
}

void findNextShip()
{
    nextShipPosition = -1;
    for (int x = 0; x < 6; x++)
    {
        if (color == shipColors[x])
        {
            nextShipPosition = x;
        }
    }
    if (nextShipPosition == -1)
    {
        nextShipPosition = 5;
    }
}

void resetMotors()
{
    resetRightDegree = ev3_motor_get_counts(motor_right);
    resetLeftDegree = ev3_motor_get_counts(motor_left);
}

void verschiffen(std::string position)
{
    moveRotLine(cSpeed, 4, "blackleft", 4, false);
    if (firstShip == true)
    {
        firstShip = false;
        bool verschifft = false;
        moveRotDeg(cSpeed, 4, 360, 0, false);
        colorDetection(LSinnen);
        tslp_tsk(10);
        shipColors[0] = colorDetection(LSinnen);
        if (shipColors[0] == color)
        {
            absetzen();
            verschifft = true;
        }
        else
        {
            moveRotDeg(-1, -4, 120, -3, false); //140
        }
        resetMotors();

        //Fährt in Startposition zum Scannen
        turn1_rot(motor_left, cSpeed, -4, 90, 0);
        moveRotDeg(2, 2, 20, 0, false);
        moveRotTime(-3, -3, 400, 0);
        tslp_tsk(200);
        moveRotDeg(0, 3, 200 + wallDistance, 3, false);
        int scannedShips = 0;
        while (scannedShips < 4)
        {
            scannedShips++;
            if (scannedShips == 4)
                if (verschifft == true)
                    shipColors[scannedShips] = moveRotDeg(cSpeed, 80, shipDistance - 90, 80, true);
                else
                    shipColors[scannedShips] = moveRotDeg(cSpeed, 80, shipDistance, 0, true);
            else
                shipColors[scannedShips] = moveRotDeg(cSpeed, 80, shipDistance, 80, true);
            std::cout << "shipColor" << shipColors[scannedShips] << std::endl;

            // Schleife und Essen werden verschifft
            if (color == shipColors[scannedShips] && verschifft == false)
            {
                moveRotDeg(3, 2, 70, 0, false); //80 distance
                turn1_rot(motor_right, cSpeed, -3, 90, 0);
                //moveRotLine(1, 3, "blackleft", 3, false); ###
                moveRotDeg(3, 3, 150, 3, false); //50
                absetzen();
                // In Position zum Scannen fahren
                old_align(-2, "blue");
                //tslp_tsk(200); ###
                //align(0,-40,"blue");
                moveRotDeg(1, 2, 80, 0, false); //100
                turn2_rot(1, 5, 90, 0);
                // moveRotDeg(1, 3, 60, 3, false);
                verschifft = true;
            }
            if (scannedShips == 4)
            {
                if (verschifft == true)
                {
                    moveRotDeg(cSpeed, 4, 0 + wallDistance, 4, false); //25
                    turn1_rot(motor_right, cSpeed, 4, 78, 4);
                    moveRotDeg(cSpeed, 4, 540, 4, false);
                }
                else
                {
                    autoFindShips();
                    moveRotDeg(-1, -3, 115 - wallDistance, 0, false); //115 //135
                    turn1_rot(motor_right, cSpeed, 4, 35, 0);         //30
                    turn1_rot(motor_left, cSpeed, 4, 120, 2);         //115
                    moveRotDeg(2, 3, 140, 3, false);
                    absetzen();
                    moveRotLine(cSpeed, -3, "blueright", -3, false);
                    turn1_rot(motor_right, cSpeed, -4, 90, 0);
                    moveRotTime(-2, -3, 500, 0);
                    turn1_rot(motor_left, 1, 4, 93, 4);
                    moveRotDeg(4, 4, 350, 4, false);
                    resetMotors();
                }
            }
        }
        std::cout << "Alle Farben (Schiffe): ";
        for (int i = 0; i < 6; i++)
        {
            std::cout << shipColors[i] << " ";
        }
        std::cout << std::endl;
    }
    else
    {
        // Hat bereits gescannt
        if ((position == "right" && nextShipPosition == 0) || (position == "left" && nextShipPosition == 5))
        {
            moveRotDeg(cSpeed, 4, 360, 2, false);
            absetzen();
            moveRotDeg(-3, -3, 80, -2, false); //110
            resetMotors();
            if (nextShipPosition == 0)
                turn1_rot(motor_left, cSpeed, -4, 90, 0);
            else
                turn1_rot(motor_right, cSpeed, -4, 90, 0);
            moveRotTime(-2, -3, 300, 0);
        }
        else
        {
            int distanceToShip;
            moveRotDeg(cSpeed, 4, 60, 0, false); //100 //80
            resetMotors();
            if (position == "right")
            {
                turn1_rot(motor_left, -2, -4, 90, 0);
                distanceToShip = nextShipPosition * (shipDistance - 20) - 80 + wallDistance;
                //distanceToShip = nextShipPosition * (shipDistance - 10) - 25 + wallDistance;
            }
            else
            {
                turn1_rot(motor_right, -2, -4, 90, 0);
                distanceToShip = (5 - nextShipPosition) * (shipDistance - 20) - 80 + wallDistance;
                //distanceToShip = (5 - nextShipPosition) * (shipDistance - 10) - 25 + wallDistance;
            }
            moveRotDeg(2, 2, 10, 0, false);
            moveRotTime(-4, -4, 400, 0);
            moveRotDeg(1, 4, distanceToShip, 3, false); // zum Schiff fahren
            if (position == "right")
            {
                turn1_rot(motor_left, cSpeed, 4, 90, 0);
            }
            else
            {
                turn1_rot(motor_right, cSpeed, 4, 90, 0);
            }
            moveRotLine(cSpeed, 4, "blackone", 4, false);
            moveRotDeg(4, 3, 200, 3, false);
            absetzen();
            //old_align(-2, "blue");###
            moveRotLine(cSpeed, -3, "blueright", -2, false);
            //tslp_tsk(200);
            moveRotDeg(cSpeed, -4, 100, 0, false);
            std::cout << "Current Food: " << currentFood << " " << std::endl;
            if (currentFood < 3)
            {
                turn2_rot((-2 * (nextShipPosition > 2) + 1), (-2 * (nextShipPosition > 2) + 1) * 5, 90, 0);
            }
        }
        if (currentFood == 3)
        {
            std::cout << "Driving back from ship" << nextShipPosition << std::endl;
            if (nextShipPosition == 0)
            {
                turn1_rot(motor_right, 1, 4, 97, 4);
                resetMotors();
            }
            else if (nextShipPosition == 5)
            {
                std::cout << "position 5" << std::endl;
                turn1_rot(motor_left, 1, 4, 97, 4);
                resetMotors();
            }
            else if (nextShipPosition == 1 || nextShipPosition == 4)
            {
                turn2_rot((-2 * (nextShipPosition <= 2) + 1), (-2 * (nextShipPosition <= 2) + 1) * 5, 150, 0);//(-2 * (nextShipPosition <= 2) + 1) für endSpeed
            }
            else if (nextShipPosition == 2 || nextShipPosition == 3)
            {
                turn2_rot((-2 * (nextShipPosition <= 2) + 1), (-2 * (nextShipPosition <= 2) + 1) * 5, 130, 0);//(-2 * (nextShipPosition <= 2) + 1) für endSpeed
            }
        }
    }
}

void schleifenSammeln(std::string position)
{
    if (position == "left" || position == "right") // von außen kommend
    {
        moveRotLine(cSpeed, 4, "blackright", 4, false);
        if (color == 4 || color == 5)
            moveRotDeg(4, 3, 45, 0, false);
        else
            moveRotDeg(4, 4, 635, 0, false);
        //resetMotors();
        if (position == "left")
            turn1_rot(motor_left, 0, -4, 90, 0);
        else
            turn1_rot(motor_right, 0, -4, 90, 0);
        moveRotDeg(1, 4, 15, 0, false);
        moveRotTime(-2, -3, 300, 0);
        moveRotDeg(1, 3, 100, 3, false);
    }
    else //aus der Mitte kommend
    {
        if (color == 4  || color == 2) // gelb oder blau
            position = "centerRight";
        else if (color == 3 || color == 5) // grün oder rot
            position = "centerLeft";






        old_align(2, "black"); // richtet sich an der Linie aus
        if (color == 3 || color == 2)
        {
            //resetMotors();
            moveRotDeg(1, 4, 225, 4, false);
            if (color == 3)
                turn1_rot(motor_left, cSpeed, 4, 90, 0);
            else
                turn1_rot(motor_right, cSpeed, 4, 90, 0);
        }
        else if (color == 4 || color == 5)
        {   
            moveRotDeg(-1, -4, 30, 0, false);
            if (color == 4)
                turn1_rot(motor_left, 0, -4, 90, 0);
            else
                turn1_rot(motor_right, 0, -4, 90, 0);
        }
        tslp_tsk(100);
    }
    if ((color == 5 || color == 4) ^ (position == "left" || position == "centerRight"))
        moveRotLine(cSpeed, 2, "blackleft", 2, false);
    else
        moveRotLine(cSpeed, 2, "blackright", 2, false);





    moveRotDeg(2, 2, 40, 0, false);
    mediumMotorTime(kuhlmotor, -80, 350, true); //-70, 550
    //Lager abgesetzt und Schleife gesammelt
    if (position == "left" || position == "right")
    {
        moveRotDeg(-1, -4, 200, -4, false);
        if (color == 4 || color == 3)
            moveRotLine(-4, -4, "blackleft", -4, false);
        else
            moveRotLine(-4, -4, "blackright", -4, false);
        moveRotTime(-4, -4, 250, 0);
        if (position == "left")
        {
            turn1_rot(motor_right, 0, 4, 95, 4);
            moveRotDeg(1, 4, 500, 4, false);
            moveRotLine(4, 4, "blackleft", 4, false);
            moveRotDeg(4, 4, 950, 4, false);
        }
        else
        {
            turn1_rot(motor_left, 0, 4, 95, 4);
            moveRotDeg(4, 4, 500, 4, false);
            moveRotLine(4, 4, "blackright", 4, false);
            moveRotDeg(4, 4, 950, 4, false);
        }
    }
    else
    {
        if (color == 4)
        {
            moveRotDeg(-1, -4, 500, 0, false); //350
            tslp_tsk(100);
            if (nextShipPosition < 3)
            {
                turn1_rot(motor_right, 1, 4, 48, 3);
                moveRotDeg(3, 4, 100, 4, false);
            }
            else
            {
                turn1_rot(motor_right, 1, 4, 48 + 90, 3);
            }
        }
        else if (color == 5)
        {
            moveRotDeg(-1, -4, 500, 0, false);
            tslp_tsk(100);
            if (nextShipPosition < 3)
            {
                turn1_rot(motor_left, 1, 4, 48 + 90, 3);
            }
            else
            {
                turn1_rot(motor_left, 1, 4, 48, 3);
                moveRotDeg(3, 4, 100, 4, false);
            }
        }
        else if (color == 3)
        {
            moveRotDeg(-1, -4, 150, -4, false);
            turn1_rot(motor_right, -4, -4, 90, 0);
            tslp_tsk(100);
        }
        else
        {
            moveRotDeg(-1, -4, 150, -4, false);
            turn1_rot(motor_left, -4, -4, 90, 0);
            tslp_tsk(100);
        }
        if (color == 3 || color == 2)
        {
            moveRotLine(1, 4, "blackleft", 4, false);
            moveRotDeg(4, 4, 370, 4, false); //470
            if (nextShipPosition < 3)
                turn1_rot(motor_left, cSpeed, 4, 48, 4); //48
            else
                turn1_rot(motor_right, cSpeed, 4, 48, 4); //48
        }
        moveRotDeg(cSpeed, 4, 680, 4, false);
        if (nextShipPosition < 3)
            turn1_rot(motor_right, 4, 4, 30, 4);
        else
            turn1_rot(motor_left, 4, 4, 30, 4);
        moveRotDegNoSync(4, 4, 750, 4);
    }
}

void test()
{

    moveRotDeg(1, 4, 130, 4, false); //100
            turn1_rot(motor_left, 4, 4, 80, 4);
            return;
    tslp_tsk(500);
    moveRotDeg(100,100,600,100,false);
    ev3_motor_set_power(motor_left, 0);
    ev3_motor_stop(motor_left, true);
    ev3_motor_set_power(motor_right, 0);
    ev3_motor_stop(motor_right, true);
    waitForButton();
    moveRotDeg(1,100,600,0,false);
    waitForButton();
    return;
}

void main_task(intptr_t unused)
{
    std::ofstream out("AA_LOG_.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf());                //redirect std::cout to out.txt!*/
    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_motor_config(motor_left, UNREGULATED_MOTOR);
    ev3_motor_config(motor_right, UNREGULATED_MOTOR);
    ev3_motor_config(kuhlmotor, UNREGULATED_MOTOR);
    ev3_motor_config(greifmotor, UNREGULATED_MOTOR);
    ev3_sensor_config(LSinnen, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(LSaussen, HT_NXT_COLOR_SENSOR);
    ev3_sensor_config(LSl, COLOR_SENSOR);
    ev3_sensor_config(LSr, COLOR_SENSOR);
    ev3_speaker_play_tone(NOTE_C4, 500);
    ev3_speaker_play_tone(NOTE_E4, 500);
    tslp_tsk(500);
    waitForButton();
    test();
    return;
    std::cout << "Battery at: " << ev3_battery_voltage_mV() << "Volt" << std::endl;
    char buffer[10];
    itoa(ev3_battery_voltage_mV(), buffer, 10);
    ev3_lcd_draw_string(buffer, 20, 50);
    waitForButton();
    resetMotors();
    tslp_tsk(100);
    Stopwatch run;
    //Zum 1. Lager fahren
    moveRotDegMediumMotorTime(1, 4, 700, 4, false, greifmotor, 100, 400, true);
    moveRotDegMediumMotorTime(cSpeed, 4, 680, 4, false, kuhlmotor, -100, 200, true);
    moveRotDeg(cSpeed, 4, 220, 3, false);
    turn1_rot(motor_left, cSpeed, 3, 90, 3);
    moveRotDeg(cSpeed, 3, 180, 0, false);
    scanLager();

    //Lager 1
    if (color > 1)
    {
        //Lager wegbringen und Schleife sammeln
        if (color == 4 || color == 2)
        {
            moveRotDeg(1, 4, 130, 4, false); //100
            turn1_rot(motor_left, 4, 4, 80, 4);
            schleifenSammeln("right");
        }
        else
        {
            turn1_rot(motor_right, -1, -4, 108, 0);
            moveRotDeg(1, 3, 700, 2, false);
            schleifenSammeln("center");
        }
    }
    else
    {
        gapFound = true;
        //Fährt zu Lager auf Position 2
        paulturn_rot(1, 5, 177, 0.45, 1, 0);
        moveRotDeg(1, 4, 380, 3, false);
        moveRotDeg(cSpeed, 2, 400, 0, false);
        //Scannt Farbe
        scanLager();
        //moveRotDeg(-2,-4,200,4,false);
        turn1_rot(motor_left, cSpeed, -4, 108, 0); //105, 108
        moveRotDeg(1, 4, 730, 3, false);           //730
        schleifenSammeln("center");
    }
    //Schleife und Essen verschiffen und alle Schiffe scannen
    verschiffen("right");
    // 2. Lager
    if (scannedLager == 1)
    {
        //Fährt zu Lager 2
        moveRotLine(cSpeed, 4, "blackright", 4, false);
        moveRotDeg(4, 4, 660, 0, false);
        tslp_tsk(100);
        turn1_rot(motor_left, -1, -4, 90, 0);
        tslp_tsk(100);
        moveRotDeg(1, 4, 15, 0, false);
        moveRotTime(-3, -3, 200, 0);
        resetMotors();
        moveRotDegMediumMotor(cSpeed, 4, 190, 4, false, kuhlmotor, -70, 120, true);
        moveRotDeg(cSpeed, 2, 400, 0, false);
        scanLager();
        if (color > 1)
        {
            //Lager an Position 2 gefunden
            turn1_rot(motor_left, 1, 4, 70, 0); //70
            tslp_tsk(100);
            moveRotDeg(cSpeed, 3, 400, 3, false); //350
        }
        else
        {
            gapFound = true;
            //Kein Lager an Position 2, fahre weiter zu Position 3
            moveRotDeg(-1, -4, 220, 0, false); //260
            turn1_rot(motor_right, 1, 4, 90, 3);
            moveRotDeg(cSpeed, 3, 340, 3, false);
            moveRotDeg(3, 2, 285, 2, false);
            turn1_rot(motor_left, cSpeed, 4, 175, 0);
            scanLager();
            moveRotDeg(1, 4, 1350, 0, false);
        }
    }
    else
    {
        //Fährt direkt zu Lager auf Position 3; dort muss etwas stehen
        moveRotLine(cSpeed, 2, "blackright", 0, false);
        //mediumMotorTime(kuhlmotor, -50, 150, true);
        moveRotDegMediumMotorTime(-1, -2, 100, -2, false, kuhlmotor, -70, 150, true);
        turn1_rot(motor_left, cSpeed, -4, 90, 0);
        moveRotTime(-2, -3, 300, 0);
        moveRotDeg(1, 3, 300, 3, false);
        moveRotDeg(3, 2, 300, 0, false);
        scanLager();
        //moveRotDeg(1, 2, 110, 2, false); ###
        turn1_rot(motor_left, cSpeed, 4, 84, 2); //85
        moveRotDeg(cSpeed, 4, 1250, 0, false);
    }
    findNextShip();
    std::cout << "NSP: " << nextShipPosition << " " << std::endl;
    schleifenSammeln("center");
    // 2. Lager wegbringen
    if (nextShipPosition < 3)
        verschiffen("right");
    else
        verschiffen("left");
    // 3. Lager

    if (nextShipPosition == 0 && scannedLager == 3)
    {
        //direkt zur 4. Position fahren
        moveRotDeg(1, 4, 380, 2, false); //380
        turn1_rot(motor_right, cSpeed, 4, 90, 2);
    }
    else if (nextShipPosition == 5 && scannedLager == 2)
    {
        //direkt zur 3. Position fahren
        moveRotDeg(1, 4, 270, 2, false); //270
        turn1_rot(motor_left, cSpeed, 4, 90, 2);
    }
    else
    {
        //Über die Mitte zum nächsten Lager fahren
        if (nextShipPosition == 2 || nextShipPosition == 3)
            //Gefahr, dass er schon auf der Linie steht
            moveRotDeg(-1, -4, 120, 0, false);
        moveRotLine(cSpeed, 4, "blackboth", 4, false);

        if ((scannedLager == 2) ^ (nextShipPosition < 3))
        {
            //Richtungswechsel
            std::cout << "scannedLager: " << scannedLager << " " << std::endl;
            int brakedistance = brake();
            moveRotDeg(-1, -4, brakedistance + 175 + (125 * (scannedLager == 2)), 0, false);
        }
        else
        {
            //Geradeaus weiter
            moveRotDeg(4, 4, 495 + (145 * (scannedLager == 2)), 0, false);
        }
        if (nextShipPosition < 3)
            //nach links drehen
            turn2_rot(1, 5, 90, 0);
        else
            //nach rechts drehen
            turn2_rot(-1, -5, 90, 0);
    }
    //mediumMotorTime(kuhlmotor, -100, 300, true);
    //mediumMotorTime(greifmotor, 100, 500, true);
    //Senkrecht an der Linie ausrichten
    moveRotDegMediumMotorTime(1, 2, 150, 0, false, kuhlmotor, -100, 300, true); // 50
    old_align(-2, "black");
    // Auf Lager zu fahren
    //tslp_tsk(150);
    moveRotDegMediumMotorTime(1, 3, 240, 2, false, greifmotor, 100, 500, true);
    moveRotDeg(cSpeed, 2, 410, 0, false);
    scanLager();
    bool missed3 = false;
    if (scannedLager == 3 && color < 2)
    {
        //Fahre zu Lager auf Position 4
        gapFound = true;
        //moveRotDeg(-1, -4, 50, -2, false);
        turn1_rot(motor_left, cSpeed, -3, 90, 0);
        moveRotDeg(0, 4, 500, 4, false);
        moveRotDeg(cSpeed, 2, 500, 0, false);
        scanLager();
        turn2_rot(-1, -5, 90, 0);
        missed3 = true;
        //moveRotDeg(cSpeed, 4, 100, 4, false);
    }
    //moveRotDeg(cSpeed, 4, 200, 2, false);
    if (scannedLager == 3)
    {
        //turn1_rot(motor_right, 1, 4, 30, 0);
        moveRotDegRatio(1, 4, 705, 1, 0.6, 4, false);
        moveRotDegRatio(cSpeed, 4, 705, 0.6, 1, 3, false);
    }
    else
    {
        if (missed3)
        {
            moveRotDegRatio(cSpeed, 4, 755, 0.67, 1, 4, false);
            moveRotDegRatio(cSpeed, 4, 755, 1, 0.67, 3, false);
        }
        else
        {
            moveRotDegRatio(cSpeed, 4, 705, 0.75, 1, 4, false);
            moveRotDegRatio(cSpeed, 4, 705, 1, 0.75, 3, false);
        }
    }
    findNextShip();
    schleifenSammeln("center");
    if (nextShipPosition < 3)
        verschiffen("right");
    else
        verschiffen("left");
    if (nextShipPosition == 0)
    {
        moveRotDegRatioNoSync(cSpeed, 4, 1800, 1, 0.99, 4, false);
    }
    else if (nextShipPosition == 5)
    {
        moveRotDegRatioNoSync(cSpeed, 4, 1800, 0.99, 1, 4, false);
    }
    else if (nextShipPosition == 1 || nextShipPosition == 4)
    {
        moveRotDegRatioNoSync(cSpeed, 4, 1600, 1 - (nextShipPosition == 1) * 0.1, 1 - (nextShipPosition == 4) * 0.1, 4, false);
    }
    else if (nextShipPosition == 2 || nextShipPosition == 3)
    {
        moveRotDegRatioNoSync(cSpeed, 4, 1000, 1 - (nextShipPosition == 2) * 0.2, 1 - (nextShipPosition == 3) * 0.2, 4, false);
        moveRotDegNoSync(4, 4, 700, 4);
    }
    moveRotLine(cSpeed, 4, "blackboth", 4, false);
    //moveRotDeg(cSpeed, 4, 510, 4, false);
    if (nextShipPosition < 3)
        moveRotDegRatioNoSync(cSpeed, 4, 510, 1, 0.99, 4, false);
    else
        moveRotDegRatioNoSync(cSpeed, 4, 510, 0.99, 1, 4, false);
    moveRotDegRatio(4, 4, 800, 1 - (nextShipPosition < 3) * 0.5, 1 - (nextShipPosition >= 3) * 0.5, 4, false);
    moveRotDeg(4, 4, 80, 4, false);
    if (nextShipPosition > 2)
        moveRotLine(4, 4, "blackleft", 4, false);
    else
        moveRotLine(4, 4, "blackright", 4, false);
    moveRotDegMediumMotor(4, 4, 350, 0, false, kuhlmotor, 100, 30, false);
    //moveRotDeg(100,100,480,true,false);
    int neededTime = run.getTime();
    std::cout << "Needed time: " << neededTime << std::endl;
    char msgbuf[10];
    sprintf(msgbuf, "Time: %d", neededTime);
    ev3_lcd_draw_string(msgbuf, 10, 10);
    nationalAnthem();
    tslp_tsk(10000);
    ev3_motor_stop(motor_left, true);
    ev3_motor_stop(motor_right, true);
    //moveRotDeg(10, 50, 100, true, false);
    tslp_tsk(1000);
    ev3_motor_stop(motor_left, false);
    ev3_motor_stop(motor_right, false);
    //return;
    std::cout.rdbuf(coutbuf); //reset to standard output again


}
