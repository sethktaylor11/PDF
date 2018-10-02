#ifndef _SPRING_SYSTEM_H_
#define _SPRING_SYSTEM_H_

#include <vector>
#include <glm/glm.hpp>
#include "tictoc.h"

class SpringSystem {
    public:
        SpringSystem(
                std::vector<glm::vec4> particles,
                std::vector<glm::uvec2> connections,
                std::vector<glm::uvec2> shear,
                std::vector<glm::uvec2> bending,
                std::vector<bool> fixed,
                std::vector<glm::uvec3> faces,
                std::vector<glm::vec4> block_vertices,
                std::vector<glm::uvec3> block_faces,
                bool col
                );
        SpringSystem() {};
        std::vector<glm::vec4> getNewPositions();
        std::vector<glm::vec4> particles;
        std::vector<glm::vec4> unconstrained_positions;
        std::vector<bool> fixed;
    private:
        void satisfyConstraints();
        std::vector<glm::vec4> calculateK1();
        std::vector<glm::vec4> calculateK2();
        std::vector<glm::vec4> calculateK3();
        std::vector<glm::vec4> calculateK4();
        std::vector<glm::uvec2> connections;
        std::vector<glm::uvec2> shear;
        std::vector<glm::uvec2> bending;
        std::vector<glm::uvec3> faces;
        std::vector<glm::vec4> block_vertices;
        std::vector<glm::uvec3> block_faces;
        std::vector<glm::vec4> velocities;
        std::vector<glm::vec4> k1;
        std::vector<glm::vec4> k2;
        std::vector<glm::vec4> k3;
        std::vector<glm::vec4> k4;
        float spring_length;
        float shear_length;
        float bending_length;
        TicTocTimer t = tic();
        float spring_constant = 1600;
        float shear_constant = 1200;
        float bending_constant = 800;
        float damping = .5;
        float air = .1;
        float time = 0.015f;
        bool col;
};

#endif
