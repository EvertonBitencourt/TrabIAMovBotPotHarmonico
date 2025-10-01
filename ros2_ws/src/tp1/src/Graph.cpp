#include "graphics.hpp"
#include "Mapping.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

// Variável global ou extern para compartilhar posição do robô
extern Position botPosition;
extern float scaleFactor;
extern std::vector<float> sonares;
extern std::vector<float> offset;
extern std::vector<double> sensorAngles;


// Histórico de posições
extern std::vector<Position> path;

// Cria Grade e Matrizes
extern GridInfo grid;
extern int size;  
extern std::vector<std::vector<float>> worldMatrix;
extern std::vector<std::vector<int>> matrizPath;
extern std::vector<std::vector<bool>> knownRegion;
extern std::vector<std::vector<float>> potentialField;


void desenhaGrade(float inicio, float fim, float passo) {
    glColor3f(0.85f, 0.0f, 0.0f);  // Vermelho para a grade
    glLineWidth(0.5f);
    glBegin(GL_LINES);

    int numLinhas = static_cast<int>((fim - inicio) / passo) + 1;

    // Linhas verticais
    for (int i = 0; i < numLinhas; ++i) {
        float x = inicio + i * passo;
        glVertex2f(x, inicio);
        glVertex2f(x, fim);
    }

    // Linhas horizontais
    for (int i = 0; i < numLinhas; ++i) {
        float y = inicio + i * passo;
        glVertex2f(inicio, y);
        glVertex2f(fim, y);
    }

    glEnd();
}

void pintaCelulas(const std::vector<std::vector<float>>& matriz, float inicio, float passo) {
    int linhas = matriz.size();
    int colunas = matriz[0].size();

    for (int i = 0; i < linhas; ++i) {
        for (int j = 0; j < colunas; ++j) {
            float x = inicio + j * passo;
            float y = inicio + i * passo;

            float matVal = matriz[i][j];
            if(matVal >= 1){
                matVal = matVal / 15.0f;
            }
            float cellColor = 1.0f * (1 - matVal);

            glColor3f(cellColor, cellColor, cellColor);  // Cores em escala de cinza para as células
            glBegin(GL_QUADS);
                glVertex2f(x, y);
                glVertex2f(x + passo, y);
                glVertex2f(x + passo, y + passo);
                glVertex2f(x, y + passo);
            glEnd();
        
        }
    }
}

void desenhaRobo(const Position& posRobo) {
    glColor3f(0.0f, 0.0f, 0.8f);  // Azul escuro para o robô
    float tamanho = 0.01f;
    int numSegmentos = 30;

    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(posRobo.x, posRobo.y);
    for (int i = 0; i <= numSegmentos; i++) {
        float angulo = 2.0f * M_PI * i / numSegmentos;
        float x = posRobo.x + cos(angulo) * tamanho;
        float y = posRobo.y + sin(angulo) * tamanho;
        glVertex2f(x, y);
    }
    glEnd();
}

void desenhaDirecao(const Position& posRobo) {
    float comprimentoLinha = 0.05f;
    float xFinal = posRobo.x + cos(posRobo.theta) * comprimentoLinha;
    float yFinal = posRobo.y + sin(posRobo.theta) * comprimentoLinha;

    glColor3f(0.1f, 0.6f, 0.2f);  // Verde para a direção
    glLineWidth(0.5f);
    glBegin(GL_LINES);
    glVertex2f(posRobo.x, posRobo.y);
    glVertex2f(xFinal, yFinal);
    glEnd();
}

void desenhaKnownRegion(GLFWwindow* windowKnown) {
    glfwMakeContextCurrent(windowKnown);
    glViewport(0, 0, 200, 200);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(grid.inicio, grid.fim, grid.inicio, grid.fim, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClearColor(1.0, 1.0, 1.0, 1.0);  // Fundo branco para a janela HIMM
    glClear(GL_COLOR_BUFFER_BIT);

    for (size_t i = 0; i < knownRegion.size(); ++i) {
        for (size_t j = 0; j < knownRegion[i].size(); ++j) {
            if (knownRegion[i][j]) {
                float x = grid.inicio + j * grid.passo;
                float y = grid.inicio + i * grid.passo;

                glColor3f(0.0f, 0.0f, 0.0f);  // Preto para a região conhecida
                glBegin(GL_QUADS);
                glVertex2f(x, y);
                glVertex2f(x + grid.passo, y);
                glVertex2f(x + grid.passo, y + grid.passo);
                glVertex2f(x, y + grid.passo);
                glEnd();
            }
        }
    }

    glfwSwapBuffers(windowKnown);
}

void desenhaCampoPotencial(GLFWwindow* window) {
    glfwMakeContextCurrent(window);
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glOrtho(grid.inicio, grid.fim, grid.inicio, grid.fim, -1.0, 1.0); // Projeção 2D

    for (size_t y = 0; y < potentialField.size(); ++y) {
        for (size_t x = 0; x < potentialField[y].size(); ++x) {
            float valor = std::clamp(potentialField[y][x], 0.0f, 1.0f);

            // Interpolação entre azul e vermelho
            float r = valor;
            float g = valor * 0.3f;
            float b = 1.0f - valor;

            float cx = grid.inicio + x * grid.passo;
            float cy = grid.inicio + y * grid.passo;

            glColor3f(r, g, b);  // Azul para roxo para vermelho
            glBegin(GL_QUADS);
                glVertex2f(cx, cy);
                glVertex2f(cx + grid.passo, cy);
                glVertex2f(cx + grid.passo, cy + grid.passo);
                glVertex2f(cx, cy + grid.passo);
            glEnd();
        }
    }

    glfwSwapBuffers(window);
}

void* graphicsThreadFunction(void* arg) {
    if (!glfwInit()) return NULL;

    int width = 600, height = 600;

    GLFWwindow* window = glfwCreateWindow(width, height, "Mapping", NULL, NULL);
    GLFWwindow* windowKnown = glfwCreateWindow(200, 200, "HIMM", NULL, NULL);
    GLFWwindow* windowCampo = glfwCreateWindow(300, 300, "Potential Field", NULL, NULL);

    if (!window || !windowKnown || !windowCampo) {
        glfwTerminate();
        return NULL;
    }

    glfwMakeContextCurrent(window);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(grid.inicio, grid.fim, grid.inicio, grid.fim, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    // Fundo preto para a janela Mapping
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);  // Fundo preto para a janela Mapping

    while (!glfwWindowShouldClose(window) && !glfwWindowShouldClose(windowKnown) && !glfwWindowShouldClose(windowCampo)) {
        Position posRobo = {
            botPosition.x * scaleFactor - offset[0], 
            botPosition.y * scaleFactor - offset[1],
            botPosition.theta
        };
        path.push_back(posRobo);

        // Janela de criação de mapa
        glfwMakeContextCurrent(window);
        glClear(GL_COLOR_BUFFER_BIT);
        pintaCelulas(worldMatrix, grid.inicio, grid.passo);
        desenhaRobo(posRobo);
        desenhaDirecao(posRobo);
        glfwSwapBuffers(window);

        // Janela de região explorada
        desenhaKnownRegion(windowKnown);

        // Janela do campo potencial
        desenhaCampoPotencial(windowCampo);

        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwDestroyWindow(windowKnown);
    glfwTerminate();
    return NULL;
}
