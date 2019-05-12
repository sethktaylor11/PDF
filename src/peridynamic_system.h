#ifndef _PERIDYNAMIC_SYSTEM_H_
#define _PERIDYNAMIC_SYSTEM_H_

#include "config.h"
#include <vector>
#include <glm/glm.hpp>
#include <Eigen/Dense>

using namespace std;

// Rendered

// Node

class Node {
	public:
		// Constructor
		Node() {};

		// Node
		Eigen::Vector3d position;

		// Neighbors
		void removeNeighbor(int p);
		void removeNeighbors(vector<int> points);
		vector<int> neighbors;

	private:
};

// Face

class Face {
	public:
		// Constructor
		Face(int b, int t) : boundary(b), triangle(t) {};
		Face() {};

		// Face
		int boundary;

		// Triangle
		int triangle;

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
				int face,
				vector<int> points,
				int neighbor,
				int tet
			);
		Triangle() {};

		// Triangle
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
				Eigen::Vector3d pos,
				double vol,
				bool fixed,
				int index,
				vector<int> roommates
		   );
		Tet() {};

		// Tet
		Eigen::Vector3d position;
		Eigen::Vector3d velocity;
		double volume;
		bool fixed;

		// Triangles
		vector<int> triangles;

		// Points
		vector<int> points;

		// Neighbors
		vector<int> neighbors; 
		vector<bool> broken;
		vector<Eigen::Vector3d> init_vecs;
		vector<double> init_lengths;
		vector<Eigen::Vector3d> init_dirs;
		vector<double> weights;

		// Next-door Neighbors
		bool hasNextDoorNeighbor(int tet);
		void removeNextDoorNeighbor(int tet);
		vector<int> nextDoorNeighbors;

		// Roommates
		vector<int> getCurrentRoommates();
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

		vector<glm::vec4> calculateNewPositions();
		vector<glm::uvec3> faces;
	private:
		vector<Eigen::Vector3d> calculateForces();

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

		// Faces
		vector<Face> Faces;

		// Nodes
		vector<glm::vec4> getNodes();
		vector<Node> Nodes;

		// Pressure
		double pressure = pressureStart;
};

#endif
