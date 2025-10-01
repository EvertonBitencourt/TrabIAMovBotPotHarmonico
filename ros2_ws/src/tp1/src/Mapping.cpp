#include "Mapping.hpp"
#include "rclcpp/rclcpp.hpp"
#include "Globals.hpp"

#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sstream>

extern Position botPosition;
extern std::vector<float> sonares;
extern std::vector<std::vector<bool>> knownRegion;

std::vector<double> sensorAngles = {-90, -50, -30, -10, 10, 30, 50, 90, 90, 130, 150, 170, -170, -150, -130, -90};
std::vector<int> sensorIndices = {0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14};
std::vector<float> offset = {0.0, 0.0};
float scaleFactor = 0.03f;

std::vector<Position> path;

GridInfo grid = {-1.0f, 1.0f, 0.005f};
int size = (grid.fim - grid.inicio) / grid.passo;
std::vector<std::vector<float>> worldMatrix(size, std::vector<float>(size, 7.5f));
std::vector<std::vector<int>> matrizPath(size, std::vector<int>(size, 0));


float round2(float valor) {
    return std::round(valor * 100.0f) / 100.0f;
}

void salvaMatriz(const std::vector<std::vector<float>>& matrix, const std::string& fileName) {
    std::ofstream file(fileName);

    if (!file.is_open()) {
        std::cerr << "Erro ao acessar '" << fileName << "' para escrita." << std::endl;
    } else {
		for (const auto& line : matrix) {
			for (const auto& value : line) {
				file << value << " ";
			}
			file << "\n";
		}

		file.close();
		std::cout << "Matriz salva em '" << fileName << "'." << std::endl;
	}
}

std::vector<std::vector<float>> loadMatrix(const std::string& fileName) {
    std::ifstream file(fileName);
    std::vector<std::vector<float>> matrix;

    if (!file.is_open()) {
        std::cerr << "Erro ao acessar '" << fileName << "' para leitura." << std::endl;
    } else {
		std::string line;
		while (std::getline(file, line)) {
			std::istringstream stream(line);
			std::vector<float> matrixLine;
			float value;

			while (stream >> value) {
				matrixLine.push_back(value);
			}

			if (!matrixLine.empty()) {
				matrix.push_back(matrixLine);
			}
		}

		file.close();
		std::cout << "Matriz carregada de '" << fileName << "'." << std::endl;
	}
    return matrix;
}

MatrixPosition findCell(float x, float y, float begin, float step) {
    int column = static_cast<int>((x - begin) / step);
    int line  = static_cast<int>((y - begin) / step);

    return {line, column};
}

CellCenter getCellCenter(const MatrixPosition& pos, float begin, float step) {
    float line = begin + pos.coluna * step + step / 2.0f;
    float column = begin + pos.linha  * step + step / 2.0f;
    return {line, column};
}

CellRelativeInfo getRelativeInfo(const CellCenter& botMiddle,const CellCenter& middlePoint,float botYaw) {
    float xDistance = middlePoint.x - botMiddle.x;
    float yDistance = middlePoint.y - botMiddle.y;

    float angle = std::atan2(yDistance, xDistance);
    float distance = std::sqrt(xDistance * xDistance + yDistance * yDistance);
    float yawRad = botYaw;

    if(angle < 0) angle += 2 * M_PI;
    if (yawRad < 0) yawRad += 2 * M_PI;

    float relativeAngle = angle - yawRad;

    if (relativeAngle > M_PI) relativeAngle -= 2 * M_PI;
    if (relativeAngle < -M_PI) relativeAngle += 2 * M_PI;

    return {distance, relativeAngle * 180.0f / M_PI};
}

bool isValidPosition(const MatrixPosition& pos, int lines, int columns) {
    return (pos.linha >= 0 && pos.linha < lines &&
            pos.coluna >= 0 && pos.coluna < columns);
}

float bayes(float R, float r, float s, float beta, float alpha, float max, float pOcup) {

    float RANGE = 0.01f;
    float pOcupS = 0.5f;

    if ((r >= s - RANGE) && (r <= s + RANGE) && (s <= R)){
        float pSOcup = 0.5f * ( (R-r)/R + (beta-std::abs(alpha))/beta ) * max;
        float pSOcup_pOcup = pSOcup * pOcup;
        pOcupS = (pSOcup_pOcup / (pSOcup_pOcup + ((1.0f - pSOcup) * (1 - pOcup))));
    }
    else {
        float pSVaz = 0.5f * ( (R-r)/R + (beta-std::abs(alpha))/beta ) * max;
        float pSOcup_pOcup = (1.0f - pSVaz) * pOcup;
        pOcupS = (pSOcup_pOcup / (pSOcup_pOcup + (pSVaz * (1 - pOcup))));
    }

    return pOcupS;
}

void updateBayes(std::vector<std::vector<float>>& matriz, Robot robot, float sensorAngle) {
    for (int line = 0; line < matriz.size(); ++line) {
        for (int column = 0; column < matriz[0].size(); ++column) {
            
            MatrixPosition pos = {line, column};
            CellCenter center = getCellCenter(pos, grid.inicio, grid.passo);

            CellRelativeInfo relations = getRelativeInfo(
                robot.cellCenter, 
                center, 
                robot.pos.theta - sensorAngle
            );

            float r = relations.distancia,
				alpha = relations.anguloRelativo,
				beta = robot.beta,
				R = 2.0f;

            if (r <= R * scaleFactor && (r <= robot.s + 0.01f) && alpha >= -beta && alpha <= beta) {
                float pOcup = matriz[line][column];
                matriz[line][column] = bayes(R, r, robot.s, beta, alpha, 0.98f, pOcup);
            }
        }
    }
}

std::vector<MatrixPosition> bresenham(MatrixPosition start, MatrixPosition end) {
    std::vector<MatrixPosition> points;

    int beginColumn = start.coluna,
		beginLine = start.linha,
		endColumn = end.coluna,
		endLine = end.linha;

    int dx = abs(endColumn - beginColumn),
		dy = -abs(endLine - beginLine),
		err = dx + dy;

    while (true) {
        points.push_back({beginLine, beginColumn});

        if (beginColumn == endColumn && beginLine == endLine) break;

        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            beginColumn += (beginColumn < endColumn) ? 1 : -1;
        }
        if (e2 <= dx) {
            err += dx;
            beginLine += (beginLine < endLine) ? 1 : -1;
        }
    }

    return points;
}

void updateHIMM(
    std::vector<std::vector<float>>& matrix,
    const Robot& robot,
    float sensorAngle,
    float maxRange = 2.0f,
    float add = 3.0f,
    float reduct = 1.0f,
    float minValue = 0.0f,
    float maxValue = 15.0f
) {

    float globalAngle = robot.pos.theta - sensorAngle;
    bool noDetect = false;

    float distance = robot.s;
    if (distance > maxRange*scaleFactor) {
        distance = maxRange*scaleFactor;
        noDetect = true;
    }

    float xFinal = robot.cellCenter.x + cos(globalAngle) * distance;
    float yFinal = robot.cellCenter.y + sin(globalAngle) * distance;

    MatrixPosition celulaFinal = findCell(xFinal, yFinal, grid.inicio, grid.passo);
    if (!isValidPosition(celulaFinal, matrix.size(), matrix[0].size())) return;

    std::vector<MatrixPosition> path = bresenham(robot.gridPos, celulaFinal);

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        float& cell = matrix[path[i].linha][path[i].coluna];
        cell = std::max(minValue, cell - reduct);
        knownRegion[path[i].linha][path[i].coluna] = true;
    }

    if (!path.empty()) {
        MatrixPosition occupied = path.back();
        float& cellCentral = matrix[occupied.linha][occupied.coluna];

        if (!noDetect) {
            float sum = 0.0f;
            float IMPORTANCE[3][3] = {
                {0.5f, 0.5f, 0.5f},
                {0.5f, 1.0f, 0.5f},
                {0.5f, 0.5f, 0.5f}
            };

            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    int nextLine = occupied.linha + i;
                    int nextColumn = occupied.coluna + j;

                    if (isValidPosition({nextLine, nextColumn}, matrix.size(), matrix[0].size())) {
                        float weight = IMPORTANCE[i + 1][j + 1];

                        if (i == 0 && j == 0) {
                            sum += add * weight;
                        } else {
                            sum += matrix[nextLine][nextColumn] * weight;
                        }
                    }
                }
            }

            cellCentral = std::min(maxValue, sum);
        } else {
            cellCentral = std::max(minValue, cellCentral - reduct);
        }

        knownRegion[occupied.linha][occupied.coluna] = true;
    }
}


void* mappingThreadFunction(void* arg) {
    while (rclcpp::ok()) {
        MatrixPosition botMatrix = findCell(botPosition.x * scaleFactor - offset[0], 
                                              botPosition.y * scaleFactor - offset[1], 
                                              grid.inicio, grid.passo);
        
        int lines = worldMatrix.size(),
			columns = worldMatrix[0].size();

        if (isValidPosition(botMatrix, lines, columns) && !sonares.empty()) {

            CellCenter botCenterCell = getCellCenter(botMatrix, grid.inicio, grid.passo);
            Robot robotInfo = {botMatrix, botPosition, botCenterCell, 0.0f};

            // Bayes
            if(false){
                for (int idx : sensorIndices) {
                    float sensorAngle = sensorAngles[idx] * M_PI / 180.0f;
                    robotInfo.s = sonares[idx] * scaleFactor;
                    updateBayes(worldMatrix, robotInfo, sensorAngle); // Bayes
                }
            }else{
            // HIMM
                for (int idx : sensorIndices) {
                    robotInfo.s = sonares[idx] * scaleFactor;
                    float sensorAngle = sensorAngles[idx] * M_PI / 180.0f;
                    updateHIMM(worldMatrix, robotInfo, sensorAngle);
                }
            }
        }

        usleep(10000);
    }
    return NULL;
}