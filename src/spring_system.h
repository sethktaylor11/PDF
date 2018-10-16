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
        std::vector<std::vector<int>> neighborhoods;
        std::vector<std::vector<glm::vec4>> init_vecs;
        std::vector<std::vector<float>> init_lengths;
        std::vector<std::vector<glm::vec4>> init_dirs;
        std::vector<std::vector<float>> weights;
        std::vector<glm::vec4> velocities;
        std::vector<glm::vec4> forces;
        TicTocTimer t = tic();
        float time = 0.015f;
        float delta = 0.6f;
        float a = 1000.0f;
        float b = 1000.0f;
        float volume = 0.015625f;
};

#endif
