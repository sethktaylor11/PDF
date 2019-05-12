#include "peridynamic_system.h"
#include "config.h"
#include <iostream>
#include <algorithm>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <math.h>

using namespace std;

// Rendered

// Node

void Node::removeNeighbor(int p) {
    for (uint i = 0; i < neighbors.size(); i++) {
        if (neighbors[i] == p) {
            neighbors.erase(neighbors.begin() + i);
            break;
	}
    }
}

void Node::removeNeighbors(vector<int> points) {
    for (uint i = 0; i < points.size(); i++) {
        removeNeighbor(points[i]);
    }
}

// Tets

// Triangle

Triangle::Triangle(
        int boundary,
        int face,
	vector<int> points,
	int neighbor,
	int tet
	) : boundary(boundary), face(face),
    points(points), neighbor(neighbor), tet(tet)
{
}

// Tet

Tet::Tet(
        Eigen::Vector3d pos,
        double vol,
        bool fixed,
        int point_index,
        vector<int> roommates
        ) : position(pos), volume(vol), fixed(fixed), points(4),
    roommates(roommates), velocity(Eigen::Vector3d(0.0,0.0,0.0)),
    force(Eigen::Vector3d(0.0,0.0,0.0))
{
    points[0] = point_index;
    points[1] = point_index+1;
    points[2] = point_index+2;
    points[3] = point_index+3;
}

bool Tet::hasNextDoorNeighbor(int tet) {
    return std::find(nextDoorNeighbors.begin(), nextDoorNeighbors.end(), tet) != nextDoorNeighbors.end();
}

void Tet::removeNextDoorNeighbor(int tet) {
    for (uint i = 0; i < nextDoorNeighbors.size(); i++) {
        if (tet == nextDoorNeighbors[i]) {
            nextDoorNeighbors.erase(nextDoorNeighbors.begin() + i);
	    break;
	}
    }
}

vector<int> Tet::getCurrentRoommates() {
    vector<int> roommies;
    if (roommates[0] != -1) roommies.push_back(roommates[0]);
    if (roommates[1] != -1) roommies.push_back(roommates[1]);
    if (roommates[2] != -1) roommies.push_back(roommates[2]);
    if (roommates[3] != -1) roommies.push_back(roommates[3]);
    return roommies;
}

bool Tet::hasRoommate(int tet) {
    return tet == roommates[0] || tet == roommates[1]
        || tet == roommates[2] || tet == roommates[3];
}

void Tet::removeRoommate(int tet) {
    if (tet == roommates[0]) roommates[0] = -1;
    else if (tet == roommates[1]) roommates[1] = -1;
    else if (tet == roommates[2]) roommates[2] = -1;
    else roommates[3] = -1;
}

// PeridynamicSystem

PeridynamicSystem::PeridynamicSystem(
        vector<glm::vec4> nodes,
        vector<bool> fixedNodes,
        vector<vector<int>> eles,
        vector<int> boundary,
        vector<vector<int>> tris,
        vector<vector<int>> neighbors,
	vector<vector<int>> roommates
        ) : tets(eles.size(), Tet()), triangles(eles.size()*3, Triangle()),
    points(eles.size()*4, Point()), Nodes(nodes.size(), Node())
{
    for (uint i = 0; i < nodes.size(); i++) {
        glm::vec4 pos = nodes[i];
        Nodes[i].position = Eigen::Vector3d(pos[0],pos[1],pos[2]);
    }

    for (uint i = 0; i < eles.size(); i++) {
        vector<int> ele = eles[i];
        int A = ele[0];
        int B = ele[1];
        int C = ele[2];
        int D = ele[3];
	int point_index = i * 4;
	Nodes[A].neighbors.push_back(point_index);
	Nodes[B].neighbors.push_back(point_index+1);
	Nodes[C].neighbors.push_back(point_index+2);
	Nodes[D].neighbors.push_back(point_index+3);
	points[point_index].tet = i;
	points[point_index+1].tet = i;
	points[point_index+2].tet = i;
	points[point_index+3].tet = i;
	Eigen::Vector3d a = Nodes[A].position;
	Eigen::Vector3d b = Nodes[B].position;
	Eigen::Vector3d c = Nodes[C].position;
	Eigen::Vector3d d = Nodes[D].position;
	Eigen::Vector3d position = (a + b + c + d) / 4.0;
        double volume = (b-a).dot((c-a).cross(d-a))/6.0;
	bool fixed = false;
        if (fixedNodes[A] || fixedNodes[B] || fixedNodes[C] || fixedNodes[D]) fixed = true;
	tets[i] = Tet(position, volume, fixed, point_index, roommates[i]);
    }

    for (uint i = 0; i < Nodes.size(); i++) {
        for (uint j = 0; j < Nodes[i].neighbors.size(); j++) {
            int p = Nodes[i].neighbors[j];
            points[p].node = i;
            points[p].neighbors = Nodes[i].neighbors;
            points[p].neighbors.erase(points[p].neighbors.begin()+j);
        }
    }

    for (uint i = 0; i < Nodes.size(); i++) {
        for (uint j = 0; j < Nodes[i].neighbors.size()-1; j++) {
            int t1 = points[Nodes[i].neighbors[j]].tet;
            for (uint k = j + 1; k < Nodes[i].neighbors.size(); k++) {
                int t2 = points[Nodes[i].neighbors[k]].tet;
		if (tets[t1].hasRoommate(t2)) continue;
		if (tets[t1].hasNextDoorNeighbor(t2)) continue;
                tets[t1].nextDoorNeighbors.push_back(t2);
		tets[t2].nextDoorNeighbors.push_back(t1);
	    }
	}
    }

    for (uint i = 0; i < tris.size(); i++) {
        int tet1 = neighbors[i][0];
	vector<int> points1 = mapTriangle(tet1, tris[i]);
	int tri1 = triangles.size();

        int tet2 = neighbors[i][1];
	int tri2 = -1;
	if (tet2 != -1) tri2 = tri1 + 1;

        int face = -1;
	int b = boundary[i];
	if (b != 0) {
            face = faces.size();
	    faces.push_back(glm::uvec3(tris[i][0],tris[i][2],tris[i][1]));
        }

        tets[tet1].triangles.push_back(tri1);
        triangles.push_back(Triangle(b, face, points1, tri2, tet1));

	if (tet2 == -1) continue;

	vector<int> points2 = mapTriangle(tet2, tris[i]);
	int temp = points2[1];
	points2[1] = points2[2];
	points2[2] = temp;

        tets[tet2].triangles.push_back(tri2);
        triangles.push_back(Triangle(0, -1, points2, tri1, tet2));
    }

    for (uint i = 0; i < tets.size()-1; i++) {
        Eigen::Vector3d pos = tets[i].position;
        for (uint j = i+1; j < tets.size(); j++) {
	    Eigen::Vector3d vec = tets[j].position - pos;
	    double length = vec.norm();
            if (length < delta) {
                tets[i].neighbors.push_back(j);
                tets[j].neighbors.push_back(i);
                tets[i].init_vecs.push_back(vec);
                tets[j].init_vecs.push_back(-vec);
                tets[i].init_lengths.push_back(length);
                tets[j].init_lengths.push_back(length);
		Eigen::Vector3d dir = vec.normalized();
                tets[i].init_dirs.push_back(dir);
                tets[j].init_dirs.push_back(-dir);
                double weight = delta / length;
                tets[i].weights.push_back(weight);
                tets[j].weights.push_back(weight);
            }
        }
        tets[i].broken.resize(tets[i].neighbors.size(),false);
    }
    tets[tets.size()-1].broken.resize(tets[tets.size()-1].neighbors.size(),false);

    for (uint i = 0; i < tets.size(); i++) {
        Eigen::Vector3d pos = tets[i].position;
	for (uint j = 0; j < 4; j++) {
            int tet2 = tets[i].roommates[j];
            if (tet2 == -1) continue;
	    Eigen::Vector3d vec = tets[tet2].position - pos;
	    double length = vec.norm();
	    if (length >= delta) splitRoommates(i,tet2);
	}
	for (uint j = 0; j < tets[i].nextDoorNeighbors.size(); j++) {
            int tet2 = tets[i].nextDoorNeighbors[j];
            if (tet2 == -1) continue;
	    Eigen::Vector3d vec = tets[tet2].position - pos;
	    double length = vec.norm();
	    if (length >= delta) {
		    splitNextDoorNeighbors(i,tet2);
		    j--;
	    }
	}
    }
}

vector<glm::vec4> PeridynamicSystem::calculateNewPositions() {
    // new particle positions
    for (uint i = 0; i < tets.size(); i++) {
        if (tets[i].fixed) continue;
        tets[i].position += tets[i].velocity*timeStep;
    }
    // new node positions
    for (uint i = 0; i < Nodes.size(); i++) {
        Eigen::Vector3d velocity(0.0,0.0,0.0);
	double weight = 0;
        for (uint j = 0; j < Nodes[i].neighbors.size(); j++) {
            int tet = points[Nodes[i].neighbors[j]].tet;
	    double w = tets[tet].volume;
            velocity += w * tets[tet].velocity;
	    weight += w;
        }
        velocity /= weight;
        Nodes[i].position += velocity*timeStep;
    }
    // calculate forces
    calculateForces();
    // new velocities
    for (uint i = 0; i < tets.size(); i++) {
        if (tets[i].fixed) continue;
        // damping
        tets[i].velocity *= 1-damping;
        tets[i].velocity += tets[i].force*timeStep/tets[i].volume;
    }
    return getNodes();
}

void PeridynamicSystem::calculateForces() {
    // reset forces
    for (uint i = 0; i < tets.size(); i++) tets[i].force = Eigen::Vector3d(0.0,0.0,0.0);

    // compute relevant values from deformed positions
    vector<vector<Eigen::Vector3d>> vecs(tets.size());
    vector<vector<double>> lengths(tets.size());
    vector<vector<Eigen::Vector3d>> dirs(tets.size());
    vector<vector<double>> extensions(tets.size());
    vector<vector<double>> stretches(tets.size());

    for (uint i = 0; i < tets.size(); i++) {
        vecs[i].resize(tets[i].neighbors.size());
        lengths[i].resize(tets[i].neighbors.size());
        dirs[i].resize(tets[i].neighbors.size());
        extensions[i].resize(tets[i].neighbors.size());
        stretches[i].resize(tets[i].neighbors.size());
        for (uint j = 0; j < tets[i].neighbors.size(); j++) {
            if (tets[i].broken[j]) continue;
	    Eigen::Vector3d vec = tets[tets[i].neighbors[j]].position - tets[i].position;
            double length = vec.norm();
	    Eigen::Vector3d dir = vec.normalized();
            double init_length = tets[i].init_lengths[j];
            double extension = length - init_length;
            double stretch = extension / init_length;
            if (stretch >= criticalTension || -stretch >= criticalCompression) {
		if (tets[i].hasRoommate(tets[i].neighbors[j]))
                    splitRoommates(i,tets[i].neighbors[j]);
	        if (tets[i].hasNextDoorNeighbor(tets[i].neighbors[j]))
                    splitNextDoorNeighbors(i,tets[i].neighbors[j]);
                tets[i].broken[j] = true;
                continue;
            }
            vecs[i][j] = vec;
            lengths[i][j] = length;
            dirs[i][j] = dir;
            extensions[i][j] = extension;
            stretches[i][j] = stretch;
        }
    }

    // compute dilatations
    vector<double> thetas(tets.size());

    for (uint i = 0; i < tets.size(); i++) {
        double sum = 0.0;
        for (uint j = 0; j < tets[i].neighbors.size(); j++) {
            if (tets[i].broken[j]) continue;
            int k = tets[i].neighbors[j];
            double weight = tets[i].weights[j];
	    Eigen::Vector3d init_vec = tets[i].init_vecs[j];
            Eigen::Vector3d dir = dirs[i][j];
            double stretch = stretches[i][j];
            sum += weight * stretch * dir.dot(init_vec) * tets[k].volume;
        }
        double theta = 9.0 / (4.0 * glm::pi<double>() * pow(delta, 4)) * sum;
        thetas[i] = theta;
    }

    // compute forces
    for (uint i = 0; i < tets.size(); i++) { 
        int p1 = i;
        double theta1 = thetas[p1];
        for (uint j = 0; j < tets[p1].neighbors.size(); j++) {
            if (tets[p1].broken[j]) continue;
            int p2 = tets[p1].neighbors[j];
            if (p2 < p1) continue;
            double weight = tets[p1].weights[j];
	    Eigen::Vector3d dir = dirs[i][j];
	    Eigen::Vector3d init_dir = tets[p1].init_dirs[j];
            double dot = dir.dot(init_dir);
            // Tp2p1
            double A_dil = 4.0 * weight * a * dot * theta1;
            double extension = extensions[i][j];
            double A_dev = 4.0 * weight * b * (extension - delta / 4.0 * dot * theta1);
            double A = A_dil + A_dev;
	    Eigen::Vector3d Tp2p1 = 0.5 * A * dir;
            // Tp1p2
            double theta2 = thetas[p2];
            A_dil = 4.0 * weight * a * dot * theta2;
            A_dev = 4.0 * weight * b * (extension - delta / 4.0 * dot * theta2);
            A = A_dil + A_dev;
	    Eigen::Vector3d Tp1p2 = 0.5 * A * -dir;
            tets[p1].force += Tp2p1 * tets[p2].volume;
            tets[p1].force -= Tp1p2 * tets[p2].volume;
            tets[p2].force += Tp1p2 * tets[p1].volume;
            tets[p2].force -= Tp2p1 * tets[p1].volume;
        }
        tets[p1].force += Eigen::Vector3d(0.0,-gravity,0.0) * tets[p1].volume;
	double y = tets[p1].position(1) + (double) height;
	// collision with floor
	if (y < 0) {
            // floor force
            tets[p1].force+= floorStiffness * Eigen::Vector3d(0.0,-y,0.0);

	    double rel_vel = tets[p1].velocity(1);
	    if (rel_vel < 0) {
	        // floor impulse
		tets[p1].velocity(1) += -(1.0 + CoR) * rel_vel;
	    }
	}
    }

    // compute pressure
    for (uint i = 0; i < triangles.size(); i++) {
        // determine if the triangle is an interior face
        if (triangles[i].boundary != 2) continue;

        glm::uvec3 face = faces[triangles[i].face];
	int tet = triangles[i].tet;
	vector<int> roommates = tets[tet].getCurrentRoommates();

	// determine if the tet has 3 roommates
	if (roommates.size() < 3) continue;

	int tet2 = roommates[0];
	int tet3 = roommates[1];
	int tet4 = roommates[2];

	Eigen::Vector3d P1 = tets[tet].position;
	Eigen::Vector3d P2 = tets[tet2].position;
	Eigen::Vector3d P3 = tets[tet3].position;
	Eigen::Vector3d P4 = tets[tet4].position;

	Eigen::Vector3d V1 = tets[tet].velocity;
	Eigen::Vector3d V2 = tets[tet2].velocity;
	Eigen::Vector3d V3 = tets[tet3].velocity;
	Eigen::Vector3d V4 = tets[tet4].velocity;
	Eigen::MatrixXd V(3,4);
	V << V1, V2, V3, V4;

	Eigen::Vector3d A = Nodes[face[0]].position;
	Eigen::Vector3d B = Nodes[face[1]].position;
	Eigen::Vector3d C = Nodes[face[2]].position;

	// calculate face normal
	Eigen::Vector3d N = (B-A).cross(C-A);
	Eigen::Vector3d n = N.normalized();

	// calculate force due to pressure
        double area = N.norm() / 2.0;
	Eigen::Vector3d force = -pressure * n * area;

	Eigen::Vector3d F = (A + B + C) / 3.0;

	Eigen::Matrix4d E;
	E << 1, 1, 1, 1,
		(P1-F).cross(force), (P2-F).cross(force),
		(P3-F).cross(force), (P4-F).cross(force);

	Eigen::Vector4d d(1.0, 0.0, 0.0, 0.0);

	Eigen::ColPivHouseholderQR<Eigen::Matrix4d> solver(E);
	Eigen::Vector4d x = solver.solve(d);

	double aa = x(0);
	double bb = x(1);
	double cc = x(2);
	double dd = x(3);
	tets[tet].force += aa * force;
	tets[tet2].force += bb * force;
	tets[tet3].force += cc * force;
	tets[tet4].force += dd * force;
    }
}

// Tets

// Tets

int PeridynamicSystem::mapPoint(int tet, int p) {
    if (p == points[tets[tet].points[0]].node) return tets[tet].points[0];
    if (p == points[tets[tet].points[1]].node) return tets[tet].points[1];
    if (p == points[tets[tet].points[2]].node) return tets[tet].points[2];
    if (p == points[tets[tet].points[3]].node) return tets[tet].points[3];
    return -1;
}

vector<int> PeridynamicSystem::mapTriangle(int tet, vector<int> tri) {
    tri[0] = mapPoint(tet, tri[0]);
    tri[1] = mapPoint(tet, tri[1]);
    tri[2] = mapPoint(tet, tri[2]);
    return tri;
}

void PeridynamicSystem::tetRemovePointNeighbor(int tet1, int tet2) {
    // remove tet2 from tet1 point neighbors
    pointRemoveNeighbor(tets[tet1].points[0], tet2);
    pointRemoveNeighbor(tets[tet1].points[1], tet2);
    pointRemoveNeighbor(tets[tet1].points[2], tet2);
    pointRemoveNeighbor(tets[tet1].points[3], tet2);
}

void PeridynamicSystem::duplicateTetNodes(int tet) {
    // duplicate tet point nodes if necessary
    duplicatePointNode(tets[tet].points[0]);
    duplicatePointNode(tets[tet].points[1]);
    duplicatePointNode(tets[tet].points[2]);
    duplicatePointNode(tets[tet].points[3]);
}

void PeridynamicSystem::updateTetFaces(int tet) {
    // update faces in tet
    updateTriangleFace(tets[tet].triangles[0]);
    updateTriangleFace(tets[tet].triangles[1]);
    updateTriangleFace(tets[tet].triangles[2]);
    updateTriangleFace(tets[tet].triangles[3]);
}

void PeridynamicSystem::split(int tet1, int tet2) {
    // remove from point neighbors
    tetRemovePointNeighbor(tet1, tet2);
    tetRemovePointNeighbor(tet2, tet1);

    // duplicate nodes in tet1 if necessary
    duplicateTetNodes(tet1);
}

void PeridynamicSystem::splitNextDoorNeighbors(int tet1, int tet2) {
    // remove from next-door neighbors
    tets[tet1].removeNextDoorNeighbor(tet2);
    tets[tet2].removeNextDoorNeighbor(tet1);

    // shared splitting behavior
    split(tet1, tet2);
}

void PeridynamicSystem::createTetFaces(int tet1, int tet2) {
    // create new faces from neighboring triangles
    int t1 = tets[tet1].triangles[0];
    int n1 = triangles[t1].neighbor;
    int t2 = tets[tet1].triangles[1];
    int n2 = triangles[t2].neighbor;
    int t3 = tets[tet1].triangles[2];
    int n3 = triangles[t3].neighbor;
    int t4 = tets[tet1].triangles[3];
    int n4 = triangles[t4].neighbor;
    if (n1 != -1 && tet2 == triangles[n1].tet) {
        createTriangleFace(t1);
	createTriangleFace(n1);
    } else if (n2 != -1 && tet2 == triangles[n2].tet) {
        createTriangleFace(t2);
	createTriangleFace(n2);
    } else if (n3 != -1 && tet2 == triangles[n3].tet) {
        createTriangleFace(t3);
	createTriangleFace(n3);
    } else {
        createTriangleFace(t4);
	createTriangleFace(n4);
    }
}

void PeridynamicSystem::splitRoommates(int tet1, int tet2) {
    // create faces from splitting tet1 and tet2
    createTetFaces(tet1, tet2);

    // remove from roommates
    tets[tet1].removeRoommate(tet2);
    tets[tet2].removeRoommate(tet1);

    // shared splitting behavior
    split(tet1, tet2);
}

// Triangles

void PeridynamicSystem::updateTriangleFace(int t) {
    // update triangle t face
    if (triangles[t].face != -1) 
        faces[triangles[t].face] = glm::uvec3(
                points[triangles[t].points[0]].node,
                points[triangles[t].points[2]].node,
		points[triangles[t].points[1]].node);
}

void PeridynamicSystem::createTriangleFace(int t) {
    // create a face from triangle t
    triangles[t].face = faces.size();
    faces.push_back(glm::uvec3(points[triangles[t].points[0]].node,
                points[triangles[t].points[2]].node,
		points[triangles[t].points[1]].node));
}

// Points

void PeridynamicSystem::pointRemoveNeighbor(int p, int t) {
    for (uint i = 0; i < points[p].neighbors.size(); i++) {
	if (points[points[p].neighbors[i]].tet == t) {
            points[p].neighbors.erase(points[p].neighbors.begin() + i);
	    break;
	}
    }
}

void PeridynamicSystem::findReachable(vector<int> &reachable, int p) {
    // recursive function to find all points reachable from point p
    for (uint i = 0; i < points[p].neighbors.size(); i++) {
        if (std::find(reachable.begin(), reachable.end(), points[p].neighbors[i]) != reachable.end())
            continue;
	reachable.push_back(points[p].neighbors[i]);
	findReachable(reachable, points[p].neighbors[i]);
    }
}

void PeridynamicSystem::duplicatePointNode(int p) {
    // find all points reachable from point p
    vector<int> reachable = points[p].neighbors;
    reachable.push_back(p);
    for (uint i = 0; i < points[p].neighbors.size(); i++) {
        findReachable(reachable, points[p].neighbors[i]);
    }

    // if the reachable set is the same as the set of point p's node's
    // neighbors then do nothing
    if (reachable.size() == Nodes[points[p].node].neighbors.size())
        return;

    // create duplicate node
    Node n = Nodes[points[p].node];

    // remove the reachable set from the duplicate node
    n.removeNeighbors(reachable);

    // update the node in the duplicate nodes neighbors
    for (uint i = 0; i < n.neighbors.size(); i++) {
        points[n.neighbors[i]].node = Nodes.size();
    }

    // update the faces in the duplicate nodes neighbors' tets
    for (uint i = 0; i < n.neighbors.size(); i++) {
        updateTetFaces(points[n.neighbors[i]].tet);
    }

    // add in duplicate node
    Nodes.push_back(n);

    // update the neighbors in the original node
    Nodes[points[p].node].neighbors = reachable;
}

// Rendered

// Nodes

vector<glm::vec4> PeridynamicSystem::getNodes() {
    vector<glm::vec4> nodes(Nodes.size());
    for (uint i = 0; i < Nodes.size(); i++) {
        Eigen::Vector3d pos = Nodes[i].position;
	nodes[i] = glm::vec4(float(pos[0]),float(pos[1]),float(pos[2]),1);
    }
    return nodes;
}
