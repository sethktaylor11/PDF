#ifndef _PERIDYNAMIC_SYSTEM_H_
#define _PERIDYNAMIC_SYSTEM_H_

#include <vector>
#include <glm/glm.hpp>
#include "tictoc.h"

using namespace std;

class PeridynamicSystem {
    public:
        PeridynamicSystem(
                vector<glm::vec4> nodes,
                vector<bool> fixedNodes,
                vector<vector<int>> tets,
                vector<glm::uvec3> faces,
                vector<int> boundary,
                vector<int> neighbors
                );
        PeridynamicSystem() {};
        void calculateNewPositions();
        vector<glm::vec4> nodes;
        vector<glm::vec4> particles;
        vector<glm::uvec3> faces;
        vector<int> boundary;
        vector<int> neighbors;
    private:
        void calculateForces();
        vector<bool> fixed;
        vector<vector<int>> tetNeighborhoods;
        vector<vector<int>> neighborhoods;
        vector<vector<bool>> broken;
        vector<vector<glm::vec4>> init_vecs;
        vector<vector<float>> init_lengths;
        vector<vector<glm::vec4>> init_dirs;
        vector<vector<float>> weights;
        vector<float> volumes;
        vector<float> moments;
        vector<glm::vec4> velocities;
        vector<glm::vec4> forces;
        vector<glm::vec3> orientations;
        vector<glm::vec3> angular_velocities;
        vector<glm::vec3> torques;
        TicTocTimer t = tic();
        float time = 0.015f;
        float delta = 0.6f;
        float a = 10.0f;
        float b = 10.0f;
        float volume = 0.015625f;
        float damping = 0.001f;
};

#endif
