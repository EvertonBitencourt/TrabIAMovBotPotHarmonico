#include "Mapping.hpp"
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <vector>

extern std::vector<std::vector<float>> worldMatrix;
extern std::vector<float> offset;
extern float scaleFactor;
extern Position botPosition;
extern GridInfo grid;

std::vector<std::vector<float>> potentialField;
std::vector<std::vector<bool>> knownRegion;

void initMatrixes() {
    size_t lines = worldMatrix.size();
    size_t columns = lines > 0 ? worldMatrix[0].size() : 0;

    knownRegion.resize(lines);
    potentialField.resize(lines);

    for (size_t i = 0; i < lines; ++i) {
        knownRegion[i].resize(columns, false);
        potentialField[i].resize(columns, 0.0f);
    }
} 

void updatePotentialField() {
	for (size_t y = 0; y < knownRegion.size(); ++y) {
		for (size_t x = 0; x < knownRegion[y].size(); ++x) {
			if (knownRegion[y][x] && worldMatrix[y][x] > 10.0f) {
				potentialField[y][x] = 1.0f;
			}
		}
	}
}

void updatePotentialField(int xStart, int xEnd, int yStart, int yEnd, float epsilon) {
	std::vector<std::vector<float>> updatedField = potentialField;
	float error;
	do {
		error = 0.0f;

        for (int y = yStart; y <= yEnd; ++y) {
            for (int x = xStart; x <= xEnd; ++x) {
                if (potentialField[y][x] != 1.0f) {
                    float newValue = 0.25f * (
                        potentialField[y - 1][x] +
                        potentialField[y + 1][x] +
                        potentialField[y][x - 1] +
                        potentialField[y][x + 1]
                    );

                    error += std::pow(potentialField[y][x] - newValue, 2);
                    updatedField[y][x] = newValue;
                }
            }
        }
        potentialField = updatedField;
	} while (error > epsilon);
}

void convertField(float epsilon) {
    if (!potentialField.empty()) {
		size_t lines = potentialField.size(),
			columns = potentialField[0].size();

		float xPosition = botPosition.x * scaleFactor + offset[0];
		float yPosition = botPosition.y * scaleFactor + offset[1];
		MatrixPosition matPos = findCell(xPosition, yPosition, grid.inicio, grid.passo);

		int RADIUS = 20;
		int xStart = std::max(1, matPos.coluna - RADIUS);
		int xEnd = std::min((int)columns - 2, matPos.coluna + RADIUS);
		int yStart = std::max(1, matPos.linha - RADIUS);
		int yEnd = std::min((int)lines - 2, matPos.linha + RADIUS);

		updatePotentialField(xStart, xEnd, yStart, yEnd, epsilon);
	}
}

void* potentialFieldThreadFunction(void* arg) {

    initMatrixes();

    while (rclcpp::ok()) {
		updatePotentialField();
        convertField(0.2f);
        
        usleep(200000);
    }
    return NULL;
}