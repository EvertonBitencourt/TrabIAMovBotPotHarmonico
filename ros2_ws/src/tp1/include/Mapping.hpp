// mapping.hpp
#ifndef MAPPING_HPP
#define MAPPING_HPP

#include <cmath>
#include <vector>
#include <string>

void* mappingThreadFunction(void* arg);

struct Position {
    float x, y, theta;
    bool isEqual(const Position& other, float epsilon = 1e-4f) const {
        return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
    }
};

struct GridInfo {
    float inicio;
    float fim;
    float passo;
};

struct MatrixPosition {
    int linha;
    int coluna;
};

struct CellCenter {
    float x;
    float y;
};

struct Robot {
    MatrixPosition gridPos;
    Position pos;
    CellCenter cellCenter;
    double s;
    float beta = 10.0f; 
};

struct CellRelativeInfo {
    float distancia;
    float anguloRelativo; // em radianos
};

struct CellInfo {
    MatrixPosition celula;
    CellCenter centro;
    CellRelativeInfo relations;
};

float round2(float valor);
float bayes(float R, float r, float s, float beta, float alpha, float max, float pOcup);
int himm();

void atualizaMatrizHIMM(
    std::vector<std::vector<int>>& matriz,
    const Robot& robot,
    float sensorAngle,
    float maxRange = 2.0f,
    int incremento = 3,
    int decremento = 1,
    int minValor = 0,
    int maxValor = 15
);
void atualizaMatrizBayes(std::vector<std::vector<float>>& matriz, Robot robot, float sensorAngle);
void salvaMatriz(const std::vector<std::vector<float>>& matriz, const std::string& nomeArquivo);
std::vector<std::vector<float>> loadMatrix(const std::string& nomeArquivo);

bool isValidPosition(const MatrixPosition& pos, int lines, int columns);

MatrixPosition findCell(float x, float y, float inicio, float passo);

CellCenter getCellCenter(const MatrixPosition& pos, float begin, float step);

CellRelativeInfo getRelativeInfo(const CellCenter& botMiddle,const CellCenter& middlePoint,float botYaw);


#endif // MAPPING_HPP