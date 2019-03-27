#ifndef _PERIDYNAMIC_SYSTEM_H_
#define _PERIDYNAMIC_SYSTEM_H_

#include <vector>
#include <glm/glm.hpp>
#include "tictoc.h"

using namespace std;

class Point {
    public:
        // Constructor
        Point() {};

	// Neighbors
	void removeNeighbor(int tet);
        vector<int> neighbors;
    private:
};

class Triangle {
    public:
        // Constructors
        Triangle(
                int boundary,
                int face,
		vector<int> points,
		vector<int> neighbors
		);
        Triangle() {};

	// Triangle
        int boundary;
	int face;
	
	// Points
	void replacePoint(int point, int newPoint);
	vector<int> points;

	// Neighbors
	bool hasNeighbor(int tet);
	void removeNeighbor(int tet);
        vector<int> neighbors;
    private:
};

class Tet {
    public:
        // Constructors
        Tet(
                glm::vec4 pos,
		float vol,
		bool fixed,
		vector<int> roommates
		);
        Tet() {};

	// Tet
	glm::vec4 position;
        glm::vec4 velocity;
	glm::vec4 force;
        float volume;
	bool fixed;

	// Triangles
	vector<int> triangles;

	// Neighbors
        vector<int> neighbors; 
        vector<bool> broken;
        vector<glm::vec4> init_vecs;
        vector<float> init_lengths;
        vector<glm::vec4> init_dirs;
        vector<float> weights;

	// Roommates
	bool hasRoommate(int tet);
	void removeRoommate(int tet);
	vector<int> roommates;
    private:
};

class PeridynamicSystem {
    public:
        // Constructors
        PeridynamicSystem(
                vector<glm::vec4> nodes,
                vector<bool> fixedNodes,
                vector<vector<int>> points,
                vector<int> boundary,
                vector<vector<int>> tris,
                vector<vector<int>> neighbors,
                vector<vector<int>> roommates
                );
        PeridynamicSystem() {};

        void calculateNewPositions();
        vector<glm::vec4> nodes;
        vector<glm::uvec3> faces;
    private:
        void calculateForces();

	// Tets
	void split(int tet1, int tet2);
	int findSharedTriangle(int tet1, int tet2);
	vector<Tet> tets;

	// Triangles
	vector<Triangle> triangles;

	// Points
	vector<Point> points;

        TicTocTimer t = tic();

	// Constants
        float time = 0.015f;
        float delta = 0.6f;
        float a = 20.0f;
        float b = 50.0f;
        float damping = 0.001f;
};

#endif
