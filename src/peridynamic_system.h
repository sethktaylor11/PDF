#ifndef _PERIDYNAMIC_SYSTEM_H_
#define _PERIDYNAMIC_SYSTEM_H_

#include <vector>
#include <glm/glm.hpp>
#include "tictoc.h"

using namespace std;

class Point {
    public:
        Point() {};
        vector<int> neighbors;
    private:
};

class Triangle {
    public:
        Triangle() {};
        int boundary;
        vector<int> neighbors;
    private:
};

class Tet {
    public:
        Tet(vector<int> roommates, bool fixed, glm::vec4 pos, float vol);
        Tet() {};
	vector<int> roommates;
	bool fixed;
        vector<int> neighbors; 
        vector<bool> broken;
        vector<glm::vec4> init_vecs;
        vector<float> init_lengths;
        vector<glm::vec4> init_dirs;
        vector<float> weights;
        float volume;
	glm::vec4 position;
        glm::vec4 velocity;
	glm::vec4 force;
    private:
};

class PeridynamicSystem {
    public:
        PeridynamicSystem(
                vector<glm::vec4> nodes,
                vector<bool> fixedNodes,
                vector<vector<int>> eles,
                vector<glm::uvec3> faces,
                vector<int> boundary,
                vector<int> neighbors,
                vector<vector<int>> roommates
                );
        PeridynamicSystem() {};
        void calculateNewPositions();
        vector<glm::vec4> nodes;
        vector<glm::uvec3> faces;
    private:
        void calculateForces();
	vector<Tet> tets;
	vector<Triangle> triangles;
	vector<Point> points;
        TicTocTimer t = tic();
        float time = 0.015f;
        float delta = 0.6f;
        float a = 20.0f;
        float b = 50.0f;
        float volume = 0.015625f;
        float damping = 0.001f;
};

#endif
