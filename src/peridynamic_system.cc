#include "peridynamic_system.h"
#include <iostream>
#include <algorithm>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <Eigen/Dense>

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
        glm::vec4 pos,
        float vol,
        bool fixed,
        int point_index,
        vector<int> roommates
        ) : position(pos), volume(vol), fixed(fixed), points(4),
    roommates(roommates), velocity(glm::vec4(0)), force(glm::vec4(0))
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
        ) : nodes(nodes), tets(eles.size(), Tet()),
    triangles(eles.size()*3, Triangle()), points(eles.size()*4, Point()),
    Nodes(nodes.size(), Node())
{
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
        glm::vec4 a = nodes[A];
        glm::vec4 b = nodes[B];
        glm::vec4 c = nodes[C];
        glm::vec4 d = nodes[D];
	glm::vec4 position = (a + b + c + d) / 4.0f;
        float volume = glm::dot(glm::vec3(b-a),glm::cross(glm::vec3(c-a),glm::vec3(d-a)))/6;
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
        glm::vec4 pos = tets[i].position;
        for (uint j = i+1; j < tets.size(); j++) {
            glm::vec4 vec = tets[j].position - pos;
            float length = glm::length(vec);
            if (length < delta) {
                tets[i].neighbors.push_back(j);
                tets[j].neighbors.push_back(i);
                tets[i].init_vecs.push_back(vec);
                tets[j].init_vecs.push_back(-vec);
                tets[i].init_lengths.push_back(length);
                tets[j].init_lengths.push_back(length);
                glm::vec4 dir = vec / length;
                tets[i].init_dirs.push_back(dir);
                tets[j].init_dirs.push_back(-dir);
                float weight = delta / length;
                tets[i].weights.push_back(weight);
                tets[j].weights.push_back(weight);
            }
        }
        tets[i].broken.resize(tets[i].neighbors.size(),false);
    }
    tets[tets.size()-1].broken.resize(tets[tets.size()-1].neighbors.size(),false);

    for (uint i = 0; i < tets.size(); i++) {
        glm::vec4 pos = tets[i].position;
	for (uint j = 0; j < 4; j++) {
            int tet2 = tets[i].roommates[j];
            if (tet2 == -1) continue;
	    glm::vec4 vec = tets[tet2].position - pos;
	    float length = glm::length(vec);
	    if (length >= delta) splitRoommates(i,tet2);
	}
	for (uint j = 0; j < tets[i].nextDoorNeighbors.size(); j++) {
            int tet2 = tets[i].nextDoorNeighbors[j];
            if (tet2 == -1) continue;
	    glm::vec4 vec = tets[tet2].position - pos;
	    float length = glm::length(vec);
	    if (length >= delta) {
		    splitNextDoorNeighbors(i,tet2);
		    j--;
	    }
	}
    }
}

void PeridynamicSystem::calculateNewPositions() {
    // new particle positions
    for (uint i = 0; i < tets.size(); i++) {
        if (tets[i].fixed) continue;
        tets[i].position += tets[i].velocity*time;
    }
    // new node positions
    for (uint i = 0; i < nodes.size(); i++) {
        // TODO needs weights and masses
        glm::vec4 velocity = glm::vec4(0);
	float weight = 0;
        for (uint j = 0; j < Nodes[i].neighbors.size(); j++) {
            int tet = points[Nodes[i].neighbors[j]].tet;
	    float w = tets[tet].volume;
            velocity += w * tets[tet].velocity;
	    weight += w;
        }
        velocity /= weight;
        nodes[i] += velocity*time;
    }
    // calculate forces
    calculateForces();
    // new velocities
    for (uint i = 0; i < tets.size(); i++) {
        if (tets[i].fixed) continue;
        // damping
        tets[i].velocity *= 1-damping;
        tets[i].velocity += tets[i].force*time/tets[i].volume;
    }
}

void PeridynamicSystem::calculateForces() {
    // reset forces
    for (uint i = 0; i < tets.size(); i++) tets[i].force = glm::vec4(0);

    // compute relevant values from deformed positions
    vector<vector<glm::vec4>> vecs(tets.size());
    vector<vector<float>> lengths(tets.size());
    vector<vector<glm::vec4>> dirs(tets.size());
    vector<vector<float>> extensions(tets.size());
    vector<vector<float>> stretches(tets.size());

    for (uint i = 0; i < tets.size(); i++) {
        vecs[i].resize(tets[i].neighbors.size());
        lengths[i].resize(tets[i].neighbors.size());
        dirs[i].resize(tets[i].neighbors.size());
        extensions[i].resize(tets[i].neighbors.size());
        stretches[i].resize(tets[i].neighbors.size());
        for (uint j = 0; j < tets[i].neighbors.size(); j++) {
            if (tets[i].broken[j]) continue;
            glm::vec4 vec = tets[tets[i].neighbors[j]].position - tets[i].position;
            float length = glm::length(vec);
            glm::vec4 dir = vec / length;
            float init_length = tets[i].init_lengths[j];
            float extension = length - init_length;
            float stretch = extension / init_length;
	    /*
            if (stretch >= .1) {
		if (tets[i].hasRoommate(tets[i].neighbors[j]))
                    splitRoommates(i,tets[i].neighbors[j]);
	        if (tets[i].hasNextDoorNeighbor(tets[i].neighbors[j]))
                    splitNextDoorNeighbors(i,tets[i].neighbors[j]);
                tets[i].broken[j] = true;
                continue;
            }
	    */
            vecs[i][j] = vec;
            lengths[i][j] = length;
            dirs[i][j] = dir;
            extensions[i][j] = extension;
            stretches[i][j] = stretch;
        }
    }

    // compute dilatations
    vector<float> thetas(tets.size());

    for (uint i = 0; i < tets.size(); i++) {
        float sum = 0.0f;
        for (uint j = 0; j < tets[i].neighbors.size(); j++) {
            if (tets[i].broken[j]) continue;
            int k = tets[i].neighbors[j];
            float weight = tets[i].weights[j];
            glm::vec4 init_vec = tets[i].init_vecs[j];
            glm::vec4 dir = dirs[i][j];
            float stretch = stretches[i][j];
            sum += weight * stretch * glm::dot(dir,init_vec) * tets[k].volume;
        }
        float theta = 9.0f / (4.0f * glm::pi<float>() * glm::pow(delta, 4)) * sum;
        thetas[i] = theta;
    }

    // compute forces
    for (uint i = 0; i < tets.size(); i++) { 
        int p1 = i;
        float theta1 = thetas[p1];
        for (uint j = 0; j < tets[p1].neighbors.size(); j++) {
            if (tets[p1].broken[j]) continue;
            int p2 = tets[p1].neighbors[j];
            if (p2 < p1) continue;
            float weight = tets[p1].weights[j];
            glm::vec4 dir = dirs[i][j];
            glm::vec4 init_dir = tets[p1].init_dirs[j];
            float dot = glm::dot(dir,init_dir);
            // Tp2p1
            float A_dil = 4.0f * weight * a * dot * theta1;
            float extension = extensions[i][j];
            float A_dev = 4.0f * weight * b * (extension - delta / 4.0f * dot * theta1);
            float A = A_dil + A_dev;
            glm::vec4 Tp2p1 = 0.5f * A * dir;
            // Tp1p2
            float theta2 = thetas[p2];
            A_dil = 4.0f * weight * a * dot * theta2;
            A_dev = 4.0f * weight * b * (extension - delta / 4.0f * dot * theta2);
            A = A_dil + A_dev;
            glm::vec4 Tp1p2 = 0.5f * A * -dir;
            tets[p1].force += Tp2p1 * tets[p2].volume;
            tets[p1].force -= Tp1p2 * tets[p2].volume;
            tets[p2].force += Tp1p2 * tets[p1].volume;
            tets[p2].force -= Tp2p1 * tets[p1].volume;
        }
        //tets[p1].force += glm::vec4(0,-1,0,0) * tets[p1].volume;
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

	glm::vec3 p1 = glm::vec3(tets[tet].position);
	glm::vec3 p2 = glm::vec3(tets[tet2].position);
	glm::vec3 p3 = glm::vec3(tets[tet3].position);
	glm::vec3 p4 = glm::vec3(tets[tet4].position);
	Eigen::Vector3f P1(p1[0],p1[1],p1[2]);
	Eigen::Vector3f P2(p2[0],p2[1],p2[2]);
	Eigen::Vector3f P3(p3[0],p3[1],p3[2]);
	Eigen::Vector3f P4(p4[0],p4[1],p4[2]);

	Eigen::Vector3f zero(0,0,0);
	glm::vec3 v1 = glm::vec3(tets[tet].velocity);
	glm::vec3 v2 = glm::vec3(tets[tet2].velocity);
	glm::vec3 v3 = glm::vec3(tets[tet3].velocity);
	glm::vec3 v4 = glm::vec3(tets[tet4].velocity);
	Eigen::Vector3f V1(v1[0],v1[1],v1[2]);
	Eigen::Vector3f V2(v2[0],v2[1],v2[2]);
	Eigen::Vector3f V3(v3[0],v3[1],v3[2]);
	Eigen::Vector3f V4(v4[0],v4[1],v4[2]);
	Eigen::MatrixXf V(3,5);
	V << V1, V2, V3, V4, zero;

	Eigen::MatrixXf Minv(5,5);
	Minv.setZero();
	Minv(0,0) = 1 / tets[tet].volume;
	Minv(1,1) = 1 / tets[tet2].volume;
	Minv(2,2) = 1 / tets[tet3].volume;
	Minv(3,3) = 1 / tets[tet4].volume;

        glm::vec4 AA = nodes[face[0]];
        glm::vec4 B = nodes[face[1]];
        glm::vec4 C = nodes[face[2]];

	// calculate face normal
        glm::vec3 N = glm::cross(glm::vec3(B-AA),glm::vec3(C-AA));
        glm::vec3 n = glm::normalize(N);

	// calculate force due to pressure
        float area = glm::length(N)/2;
	glm::vec3 force = -1.0f * n * area;

	glm::vec4 F = (AA + B + C)/3.0f;
	Eigen::Vector3f P(F[0],F[1],F[2]);

        Eigen::Vector3f Force(force[0],force[1],force[2]);
	Eigen::VectorXf c = V.transpose() * Force;
	Eigen::MatrixXf Q = time * Force.dot(Force) * Minv;

	Eigen::MatrixXf E(4,5);
	E << 1, 1, 1, 1, 0,
		P1 - P4, P2 - P4, P3 - P4, zero, Force;
	Eigen::Vector4f d;
        d << 1, P - P4;

	Eigen::MatrixXf Zero(4,4);
	Zero.setZero();
	Eigen::MatrixXf A(9,9);
        A << Q, E.transpose(), E, Zero;

	Eigen::VectorXf b(9);
	b << -c, d;

	Eigen::VectorXf x = A.inverse() * b;

	float a = x(0);
	float bb = x(1);
	float cc = x(2);
	float dd = x(3);
	tets[tet].force += a * glm::vec4(force, 0);
	tets[tet2].force += bb * glm::vec4(force, 0);
	tets[tet3].force += cc * glm::vec4(force, 0);
	tets[tet4].force += dd * glm::vec4(force, 0);
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
        points[n.neighbors[i]].node = nodes.size();
    }

    // update the faces in the duplicate nodes neighbors' tets
    for (uint i = 0; i < n.neighbors.size(); i++) {
        updateTetFaces(points[n.neighbors[i]].tet);
    }

    // add in duplicate node
    Nodes.push_back(n);
    nodes.push_back(nodes[points[p].node]);

    // update the neighbors in the original node
    Nodes[points[p].node].neighbors = reachable;
}

// Rendered

// Nodes

bool PeridynamicSystem::hasNeighbor(int node, int tet) {
    for (uint i = 0; i < Nodes[node].neighbors.size(); i++) {
        if (points[Nodes[node].neighbors[i]].tet == tet) return true;
    }
    return false;
}
