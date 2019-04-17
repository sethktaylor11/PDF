#ifndef _PERIDYNAMIC_SYSTEM_H_
#define _PERIDYNAMIC_SYSTEM_H_

#include <vector>
#include <glm/glm.hpp>
#include "tictoc.h"

using namespace std;

// Rendered

// Node

class Node {
    public:
        // Constructor
        Node() {};

	// Neighbors
	void removeNeighbor(int p);
	void removeNeighbors(vector<int> points);
	vector<int> neighbors;

    private:
};

// Tets

// Point

class Point {
    public:
        // Constructor
        Point() {};

	// Node
	int node;

	// Neighbors
        vector<int> neighbors;

	// Tet
	int tet;
    private:
};

// Triangle

class Triangle {
    public:
        // Constructors
        Triangle(
                int boundary,
                int face,
		vector<int> points,
		int neighbor,
		int tet
		);
        Triangle() {};

	// Triangle
        int boundary;
	int face;
	
	// Points
	vector<int> points;

	// Neighbor
        int neighbor;

	// Tet
	int tet;
    private:
};

// Tet

class Tet {
    public:
        // Constructors
        Tet(
                glm::vec4 pos,
		float vol,
		bool fixed,
		int index,
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

	// Points
        vector<int> points;

	// Neighbors
        vector<int> neighbors; 
        vector<bool> broken;
        vector<glm::vec4> init_vecs;
        vector<float> init_lengths;
        vector<glm::vec4> init_dirs;
        vector<float> weights;

	// Next-door Neighbors
	bool hasNextDoorNeighbor(int tet);
	void removeNextDoorNeighbor(int tet);
	vector<int> nextDoorNeighbors;

	// Roommates
	bool hasRoommate(int tet);
	void removeRoommate(int tet);
	vector<int> roommates;
    private:
};

// PeridynamicSystem

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

        // Tets
	int mapPoint(int tet, int p);
        vector<int> mapTriangle(int tet, vector<int> tri);
	void tetRemovePointNeighbor(int tet1, int tet2);
	void duplicateTetNodes(int tet);
	void updateTetFaces(int tet);
	void split(int tet1, int tet2);
	void splitNextDoorNeighbors(int tet1, int tet2);
	void createTetFaces(int tet1, int tet2);
	void splitRoommates(int tet1, int tet2);
	vector<Tet> tets;

	// Triangles
	void updateTriangleFace(int t);
	void createTriangleFace(int t);
	vector<Triangle> triangles;

	// Points
        void pointRemoveNeighbor(int p, int t);
	void findReachable(vector<int> &reachable, int p);
	void duplicatePointNode(int p);
	vector<Point> points;

        // Rendered
	
	// Nodes
        bool hasNeighbor(int node, int tet);
        vector<Node> Nodes;

        TicTocTimer t = tic();

	// Constants
        float time = 0.015f;
        float delta = 0.6f;
        float a = 1.0f;
        float b = 1.0f;
        float damping = 0.001f;
};

#endif
