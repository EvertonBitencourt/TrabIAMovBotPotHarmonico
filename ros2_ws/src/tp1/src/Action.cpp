#include "Action.h"
#include "Utils.h"
#include "graphics.hpp"
#include "Mapping.hpp"
#include "PotentialField.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

extern std::vector<std::vector<float>> potentialField;  // PotentialField.cpp
extern std::vector<std::vector<float>> worldMatrix;  // mapping.cpp
extern GridInfo grid;

Position botPosition = {0.0f, 0.0f, 0.0f};
extern float scaleFactor;

std::vector<Position> positionArray;
std::vector<float> sonares;

float yawGradiente(
    const std::vector<std::vector<float>>& potential,
    float xPosition, float yPosition
) {
    MatrixPosition positionStruct = findCell(xPosition, yPosition, grid.inicio, grid.passo);
	int x = positionStruct.coluna, y = positionStruct.linha;
	int width = potentialField[0].size();
    int height  = potentialField.size();

    if (x <= 0 || x >= width - 1 || y <= 0 || y >= height - 1) {
        std::cout << "Fora de alcance" << std::endl;
        return 0.0f;
    }

    float dx = potential[y][x + 1] - potential[y][x - 1];
    float dy = potential[y + 1][x] - potential[y - 1][x];

    return std::atan2(-dy, -dx);
}

Controle controleRobo(
    float currentYaw,
    float targetYaw,
    PID& pid,
    float interval = 0.05f,
    float tolerance = 0.1f
) {
    float error = targetYaw - currentYaw;
    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;

    pid.erroAcumulado += error * interval;
    float derivate = (error - pid.erroAnterior) / interval;
    pid.erroAnterior = error;

    Controle control;
    control.angVel = pid.kp * error + pid.ki * pid.erroAcumulado + pid.kd * derivate;

    if (std::abs(error) < tolerance) {
        control.linVel = 0.8f;
    } else {
        control.linVel = 0.2f;
    }

    return control;
}

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

void Action::avoidObstacles(const std::vector<float> lasers, const std::vector<float> sonars)
{
    if (sonars.size() < 8) {
        std::cerr << "Erro: vetor de sonares com tamanho insuficiente: " << sonars.size() << std::endl;
        linVel = 0.0;
        angVel = 0.0;
        return;
    }

    float frontLeft = sonars[3]; 
    float frontRight = sonars[4]; 

    float sideLeft = (sonars[0] + sonars[1]) / 2.0;
    float sideRight = (sonars[6] + sonars[7]) / 2.0;

    float threshold = 1;

    if (frontLeft < threshold || frontRight < threshold)
    {
        if (sideLeft > sideRight)
        {
            linVel = 0.0;
            angVel = 0.5;
        }
        else
        {
            linVel = 0.0;
            angVel = -0.5;
        }
    }
    else
    {
        linVel = 5.0;
        angVel = 0.0;
    }
}

void Action::keepAsCloseAsPossibleToTheWalls(std::vector<float> lasers, std::vector<float> sonars)
{
    if (sonars.size() < 8) {
        std::cerr << "Erro: vetor de sonares com tamanho insuficiente: " << sonars.size() << std::endl;
        linVel = 0.0;
        angVel = 0.0;
        return;
    }

    float frontLeft = sonars[3]; 
    float frontRight = sonars[4]; 
    float sideLeft = (sonars[0] + sonars[1]) / 2.0;
    float backLeft = (sonars[0] + sonars[15]) / 2.0;
    float sideRight = (sonars[6] + sonars[7]) / 2.0;
    float threshold = 0.8;

    if(frontLeft < threshold){
        linVel = 0.0;
        angVel = -1.0;
    }
    else if(sideLeft > 0.8 && backLeft <= frontLeft){
        linVel = 0.5;
        angVel = 0.5;
    }
    else if(sonars[0] < 0.8)
    {
        linVel = 1.0;
        angVel = -0.5;
    }
    else{
        linVel = 1.0;
        angVel = 0.0;
    }        
}

void Action::keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars)
{
    if (sonars.size() < 8) {
        std::cerr << "Erro: vetor de sonares com tamanho insuficiente: " << sonars.size() << std::endl;
        linVel = 0.0;
        angVel = 0.0;
        return;
    }

    float frontLeft = sonars[3]; 
    float frontRight = sonars[4]; 

    float sideLeft = (sonars[0] + sonars[1]) / 2.0;
    float sideRight = (sonars[6] + sonars[7]) / 2.0;

    float threshold = 1;
    float diff = sideLeft - sideRight;
    
    if (frontLeft < threshold || frontRight < threshold)
    {
        if (sideLeft > sideRight)
        {
            linVel = 0.0;
            angVel = 0.5;
        }
        else
        {
            linVel = 0.0;
            angVel = -0.5;
        }
    }
    else
    {   
        linVel = 5.0;
        if (diff != 0){
            angVel = diff > 0 ? 8.0 : -8.0;
        }
        else
        {
            angVel = 0.0;
        }
    }
}

float distPontos(Position p1, Position p2){
    return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

Position detectarParede(const std::vector<float>& pose, float sonarDistance, bool left) {

    float xWall = pose[0], yWall = pose[1], sinAng = sin(pose[2]), cosAng = cos(pose[2]);

    if (left){
        xWall -=  sonarDistance * sinAng;
        yWall +=  sonarDistance * cosAng;
    } else{
        xWall +=  sonarDistance * sinAng;
        yWall -=  sonarDistance * cosAng;
    }

    Position wallPos = {xWall, yWall};
    return {wallPos};  
}

void Action::testMode(std::vector<float> lasers, std::vector<float> sonars, std::vector<float> pose)
{
    botPosition = {pose[0], pose[1], pose[2]};
    sonares = sonars;
    positionArray.push_back(botPosition);

    PID pid = { 0.02f, 0.0f, 0.01f }; // parâmetros do PID

    float idealYaw = yawGradiente(potentialField, (pose[0] * scaleFactor), (pose[1] * scaleFactor));

    Controle control = controleRobo(pose[2], idealYaw, pid);
    linVel = control.linVel;
    angVel = control.angVel;
}

void Action::manualRobotMotion(MovingDirection direction, std::vector<float> sonars, std::vector<float> pose)
{
    botPosition = {pose[0], pose[1], pose[2]};
    sonares = sonars;
    positionArray.push_back(botPosition);

    if(direction == FRONT){
        linVel= 0.5; angVel= 0.0;
    }else if(direction == BACK){
        linVel=-0.5; angVel= 0.0;
    }else if(direction == LEFT){
        linVel= 0.0; angVel= 0.5;
    }else if(direction == RIGHT){
        linVel= 0.0; angVel=-0.5;
    }else if(direction == STOP){
        linVel= 0.0; angVel= 0.0;
    }
    
    if (sonars[3] <= 1.1 || sonars[4] <= 1.1){
        linVel= 0.0; 
    }
}

void Action::correctVelocitiesIfInvalid()
{
    float MULTIPLIER=0.38, MAX_SPEED = 0.5;

    float leftVel  = linVel - angVel*MULTIPLIER/(2.0),
		rightVel = linVel + angVel*MULTIPLIER/(2.0);

    float absLeft = fabs(leftVel),
		absRight = fabs(rightVel);

    if(absLeft>absRight){
        if(absLeft > MAX_SPEED){
            leftVel *= MAX_SPEED/absLeft;
            rightVel *= MAX_SPEED/absLeft;
        }
    }else{
        if(absRight > MAX_SPEED){
            leftVel *= MAX_SPEED/absRight;
            rightVel *= MAX_SPEED/absRight;
        }
    }
    
    linVel = (leftVel + rightVel)/2.0;
    angVel = (rightVel - leftVel)/MULTIPLIER;
}

float Action::getLinearVelocity()
{
    return linVel;
}

float Action::getAngularVelocity()
{
    return angVel;
}

bool mapSaved = false;
bool mapLoaded = false;
MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;

    if(key=='1'){
        mc.mode=MANUAL;
        mc.direction=STOP;
    }else if(key=='2'){
        mc.mode=WANDER;
        mc.direction=AUTO;
    }else if(key=='3'){
        mc.mode=FARFROMWALLS;
        mc.direction=AUTO;
    }else if(key=='4'){
        mc.mode=FOLLOWWALLS;
        mc.direction=AUTO;
    }else if(key=='5'){
        mc.mode=TESTMODE;
        mc.direction=AUTO;
    }else if(key=='w' or key=='W'){
        mc.mode=MANUAL;
        mc.direction = FRONT;
    }else if(key=='s' or key=='S'){
        mc.mode=MANUAL;
        mc.direction = BACK;
    }else if(key=='a' or key=='A'){
        mc.mode=MANUAL;
        mc.direction = LEFT;
    }else if(key=='d' or key=='D'){
        mc.mode=MANUAL;
        mc.direction = RIGHT;
    }else if(key==' '){
        mc.mode=MANUAL;
        mc.direction = STOP;
    }else if(key=='v' or key=='V'){
        if(!mapSaved){
            salvaMatriz(worldMatrix, "matriz.txt");
        }
		mapSaved = !mapSaved;
    }else if(key=='c' or key=='C'){
        if(!mapLoaded){
            worldMatrix = loadMatrix("matriz.txt");
        }
		mapLoaded = !mapLoaded;
    }
    
    return mc;
}