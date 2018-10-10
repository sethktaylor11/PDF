#ifndef _SPRING_SYSTEM_H_
#define _SPRING_SYSTEM_H_

#include <vector>
#include <glm/glm.hpp>
#include "tictoc.h"

class SpringSystem {
    public:
        SpringSystem(
                std::vector<glm::vec4> particles,
                std::vector<bool> fixed
                );
        SpringSystem() {};
        std::vector<glm::vec4> getNewPositions();
        std::vector<glm::vec4> particles;
        std::vector<bool> fixed;
    private:
        void calculateForces();
        glm::vec4 calculateForceAOnB(int a, int b, float weight);
        float calculateDilatation(int i);
        std::vector<std::vector<int>> neighborhoods;
        std::vector<glm::vec4> init_particles;
        std::vector<std::vector<float>> weights;
        std::vector<glm::vec4> velocities;
        std::vector<glm::vec4> forces;
        TicTocTimer t = tic();
        float time = 0.015f;
        float delta = 0.6f;
        float a = 0.1f;
        float b = 0.1f;
        float volume = 0.015625f;
};

#endif
