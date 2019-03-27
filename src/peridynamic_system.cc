#include "peridynamic_system.h"
#include <iostream>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

using namespace std;

void Point::removeNeighbor(int tet) {
    for (uint i = 0; i < neighbors.size(); i++) {
        if (tet == neighbors[i]) {
            neighbors.erase(neighbors.begin() + i);
	    break;
	}
    }
}

Triangle::Triangle(
        int boundary,
        int face,
	vector<int> points,
	vector<int> neighbors
	) : boundary(boundary), face(face),
    points(points), neighbors(neighbors)
{
}

void Triangle::replacePoint(int point, int newPoint) {
    if (points[0] == point) points[0] = newPoint;
    else if (points[1] == point) points[1] = newPoint;
    else if (points[2] == point) points[2] = newPoint;
}

bool Triangle::hasNeighbor(int tet) {
    return tet == neighbors[0] || tet == neighbors[1];
}

void Triangle::removeNeighbor(int tet) {
    if (neighbors[0] == tet) neighbors[0] = neighbors[1];
    neighbors[1] = -1;
}

Tet::Tet(
        glm::vec4 pos,
        float vol,
        bool fixed,
        vector<int> roommates
        ) : position(pos), volume(vol), fixed(fixed),
    roommates(roommates), velocity(glm::vec4(0)), force(glm::vec4(0))
{
}

bool Tet::hasRoommate(int tet) {
    return tet == roommates[0] || tet == roommates[1]
        || tet == roommates[2] || tet == roommates[3];
}

void Tet::removeRoommate(int tet) {
    if (roommates[0] == tet) roommates[0] = -1;
    else if (roommates[1] == tet) roommates[1] = -1;
    else if (roommates[2] == tet) roommates[2] = -1;
    else roommates[3] = -1;
}

PeridynamicSystem::PeridynamicSystem(
        vector<glm::vec4> nodes,
        vector<bool> fixedNodes,
        vector<vector<int>> eles,
        vector<int> boundary,
        vector<vector<int>> tris,
        vector<vector<int>> neighbors,
	vector<vector<int>> roommates
        ) : nodes(nodes), tets(eles.size(), Tet()),
    triangles(tris.size(), Triangle()), points(nodes.size(), Point())
{
    for (uint i = 0; i < eles.size(); i++) {
        vector<int> ele = eles[i];
        int A = ele[0];
        int B = ele[1];
        int C = ele[2];
        int D = ele[3];
	points[A].neighbors.push_back(i);
	points[B].neighbors.push_back(i);
	points[C].neighbors.push_back(i);
	points[D].neighbors.push_back(i);
        glm::vec4 a = nodes[A];
        glm::vec4 b = nodes[B];
        glm::vec4 c = nodes[C];
        glm::vec4 d = nodes[D];
	glm::vec4 position = (a + b + c + d) / 4.0f;
        float volume = glm::dot(glm::vec3(b-a),glm::cross(glm::vec3(c-a),glm::vec3(d-a)))/6;
	bool fixed = false;
        if (fixedNodes[A] || fixedNodes[B] || fixedNodes[C] || fixedNodes[D]) fixed = true;
	tets[i] = Tet(position, volume, fixed, roommates[i]);
    }

    for (uint i = 0; i < tris.size(); i++) {
        int face = -1;
	if (boundary[i] != 0) {
            face = faces.size();
	    faces.push_back(glm::uvec3(tris[i][0],tris[i][2],tris[i][1]));
        }
        triangles[i] = Triangle(boundary[i], face, tris[i], neighbors[i]);
        tets[neighbors[i][0]].triangles.push_back(i);
	if (neighbors[i][1] != -1)
            tets[neighbors[i][1]].triangles.push_back(i);
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
}

void PeridynamicSystem::calculateNewPositions() {
    // midway velocity
    for (uint i = 0; i < tets.size(); i++) {
        if (tets[i].fixed) continue;
        tets[i].velocity += tets[i].force*time/(2*tets[i].volume);
    }
    // new particle positions
    for (uint i = 0; i < tets.size(); i++) {
        // damping
        tets[i].velocity *= 1-damping;
        if (tets[i].fixed) continue;
        tets[i].position += tets[i].velocity*time;
    }
    // new node positions
    for (uint i = 0; i < nodes.size(); i++) {
        // TODO needs weights and masses
        glm::vec4 velocity = glm::vec4(0);
        for (uint j = 0; j < points[i].neighbors.size(); j++) {
            velocity += tets[points[i].neighbors[j]].velocity;
        }
        velocity /= points[i].neighbors.size();
        nodes[i] += velocity*time;
    }
    // calculate forces
    calculateForces();
    // new velocities
    for (uint i = 0; i < tets.size(); i++) {
        tets[i].velocity *= 1-damping;
        if (tets[i].fixed) continue;
        tets[i].velocity += tets[i].force*time/(2*tets[i].volume);
    }
    /*
    // remove broken faces
    for (uint i = 0; i < faces.size(); i++) {
        int tet = triangles[i].neighbors[0];
	for (uint j = 0; j < tets[tet].broken.size(); j++) {
            if (tets[tet].broken[j]) {
                faces.erase(faces.begin() + i);
		triangles.erase(triangles.begin() + i);
		i--;
		continue;
	    }
	}
	i++;
    }
    */
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
            if (stretch >= .1) {
	        if (tets[i].hasRoommate(tets[i].neighbors[j])) {
                    split(i,tets[i].neighbors[j]);
		}
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
        tets[p1].force += glm::vec4(0,-1,0,0) * tets[p1].volume;
        //tets[p1].force += glm::vec4(1,0,0,0) * tets[p1].volume;
    }

    /*
    glm::vec4 pressure = glm::vec4(0);
    // compute pressure
    for (uint i = 0; i < faces.size(); i++) {
        if (boundary[i] != 2) continue;
        glm::uvec3 face = faces[i];
        glm::vec4 A = nodes[face[0]];
        glm::vec4 B = nodes[face[1]];
        glm::vec4 C = nodes[face[2]];
        glm::vec3 N = glm::cross(glm::vec3(B-A),glm::vec3(C-A));
        glm::vec3 n = glm::normalize(N);
        float area = glm::length(N)/2;
        forces[neighbors[i]] += -40.0f * glm::vec4(n,0) * area;
        pressure += -40.0f * glm::vec4(n,0) * area;
    }
    cout << "pressure " << glm::to_string(pressure) << endl;
    glm::vec3 angular = glm::vec3(0);
    for (uint i = 0; i < tets.size(); i++) {
	angular += glm::cross(glm::vec3(particles[i])-glm::vec3(0,3,0),glm::vec3(forces[i]));
    }
    cout << "angular " << glm::to_string(angular) << endl;
    assert(glm::length(pressure) == 0);
    */
}

void PeridynamicSystem::split(int tet1, int tet2) {
    // remove from roommates
    tets[tet1].removeRoommate(tet2);
    tets[tet2].removeRoommate(tet1);

    // find shared triangle
    int tri = findSharedTriangle(tet1, tet2);

    // create duplicate points
    int p1 = triangles[tri].points[0];
    int p2 = triangles[tri].points[1];
    int p3 = triangles[tri].points[2];
    glm::vec4 n1 = nodes[p1];
    glm::vec4 n2 = nodes[p2];
    glm::vec4 n3 = nodes[p3];
    Point P1 = points[p1];
    Point P2 = points[p2];
    Point P3 = points[p3];

    // remove tet1 from duplicate points
    P1.removeNeighbor(tet1);
    P2.removeNeighbor(tet1);
    P3.removeNeighbor(tet1);

    // create duplicate triangle
    Triangle t = triangles[tri];

    // correct duplicate triangle point indices
    vector<int> p(3);
    p[0] = points.size();
    p[1] = points.size() + 1;
    p[2] = points.size() + 2;
    t.points = p;

    // remove tet1 from duplicate triangle
    t.removeNeighbor(tet1);

    // update triangle in tet2 and points and faces in tet2 triangles
    int t1 = tets[tet2].triangles[0];
    int t2 = tets[tet2].triangles[1];
    int t3 = tets[tet2].triangles[2];
    int t4 = tets[tet2].triangles[3];
    if (tri == t1) {
        tets[tet2].triangles[0] = triangles.size();
	
	triangles[t2].replacePoint(p1, p[0]);
	triangles[t2].replacePoint(p2, p[1]);
	triangles[t2].replacePoint(p3, p[2]);

	if (triangles[t2].face != -1) 
            faces[triangles[t2].face] = glm::uvec3(triangles[t2].points[0],
                    triangles[t2].points[2], triangles[t2].points[1]);
                                          
	triangles[t3].replacePoint(p1, p[0]);
	triangles[t3].replacePoint(p2, p[1]);
	triangles[t3].replacePoint(p3, p[2]);

	if (triangles[t3].face != -1) 
            faces[triangles[t3].face] = glm::uvec3(triangles[t3].points[0],
                    triangles[t3].points[2], triangles[t3].points[1]);
                                          
	triangles[t4].replacePoint(p1, p[0]);
	triangles[t4].replacePoint(p2, p[1]);
	triangles[t4].replacePoint(p3, p[2]);

	if (triangles[t4].face != -1) 
            faces[triangles[t4].face] = glm::uvec3(triangles[t4].points[0],
                    triangles[t4].points[2], triangles[t4].points[1]);
    } else if (tri == t2) {
        tets[tet2].triangles[1] = triangles.size();
	
	triangles[t1].replacePoint(p1, p[0]);
	triangles[t1].replacePoint(p2, p[1]);
	triangles[t1].replacePoint(p3, p[2]);

	if (triangles[t1].face != -1) 
            faces[triangles[t1].face] = glm::uvec3(triangles[t1].points[0],
                    triangles[t1].points[2], triangles[t1].points[1]);
                                          
	triangles[t3].replacePoint(p1, p[0]);
	triangles[t3].replacePoint(p2, p[1]);
	triangles[t3].replacePoint(p3, p[2]);

	if (triangles[t3].face != -1) 
            faces[triangles[t3].face] = glm::uvec3(triangles[t3].points[0],
                    triangles[t3].points[2], triangles[t3].points[1]);
                                          
	triangles[t4].replacePoint(p1, p[0]);
	triangles[t4].replacePoint(p2, p[1]);
	triangles[t4].replacePoint(p3, p[2]);

	if (triangles[t4].face != -1) 
            faces[triangles[t4].face] = glm::uvec3(triangles[t4].points[0],
                    triangles[t4].points[2], triangles[t4].points[1]);
    } else if (tri == t3) {
        tets[tet2].triangles[2] = triangles.size();
	
	triangles[t1].replacePoint(p1, p[0]);
	triangles[t1].replacePoint(p2, p[1]);
	triangles[t1].replacePoint(p3, p[2]);

	if (triangles[t1].face != -1) 
            faces[triangles[t1].face] = glm::uvec3(triangles[t1].points[0],
                    triangles[t1].points[2], triangles[t1].points[1]);
                                          
	triangles[t2].replacePoint(p1, p[0]);
	triangles[t2].replacePoint(p2, p[1]);
	triangles[t2].replacePoint(p3, p[2]);

	if (triangles[t2].face != -1) 
            faces[triangles[t2].face] = glm::uvec3(triangles[t2].points[0],
                    triangles[t2].points[2], triangles[t2].points[1]);
                                          
	triangles[t4].replacePoint(p1, p[0]);
	triangles[t4].replacePoint(p2, p[1]);
	triangles[t4].replacePoint(p3, p[2]);

	if (triangles[t4].face != -1) 
            faces[triangles[t4].face] = glm::uvec3(triangles[t4].points[0],
                    triangles[t4].points[2], triangles[t4].points[1]);
    } else if (tri == t4) {
        tets[tet2].triangles[3] = triangles.size();
	
	triangles[t1].replacePoint(p1, p[0]);
	triangles[t1].replacePoint(p2, p[1]);
	triangles[t1].replacePoint(p3, p[2]);

	if (triangles[t1].face != -1) 
            faces[triangles[t1].face] = glm::uvec3(triangles[t1].points[0],
                    triangles[t1].points[2], triangles[t1].points[1]);
                                          
	triangles[t2].replacePoint(p1, p[0]);
	triangles[t2].replacePoint(p2, p[1]);
	triangles[t2].replacePoint(p3, p[2]);

	if (triangles[t2].face != -1) 
            faces[triangles[t2].face] = glm::uvec3(triangles[t2].points[0],
                    triangles[t2].points[2], triangles[t2].points[1]);
                                          
	triangles[t3].replacePoint(p1, p[0]);
	triangles[t3].replacePoint(p2, p[1]);
	triangles[t3].replacePoint(p3, p[2]);

	if (triangles[t3].face != -1) 
            faces[triangles[t3].face] = glm::uvec3(triangles[t3].points[0],
                    triangles[t3].points[2], triangles[t3].points[1]);
    }

    // add in duplicate points
    nodes.push_back(n1);
    nodes.push_back(n2);
    nodes.push_back(n3);
    points.push_back(P1);
    points.push_back(P2);
    points.push_back(P3);

    // add in duplicate triangle
    triangles.push_back(t);

    // remove tet2 from original points
    points[p1].removeNeighbor(tet2);
    points[p2].removeNeighbor(tet2);
    points[p3].removeNeighbor(tet2);

    // remove tet2 from original triangle
    triangles[tri].removeNeighbor(tet2);
}

int PeridynamicSystem::findSharedTriangle(int tet1, int tet2) {
    if (triangles[tets[tet1].triangles[0]].hasNeighbor(tet2))
        return tets[tet1].triangles[0];
    if (triangles[tets[tet1].triangles[1]].hasNeighbor(tet2))
        return tets[tet1].triangles[1];
    if (triangles[tets[tet1].triangles[2]].hasNeighbor(tet2))
        return tets[tet1].triangles[2];
    if (triangles[tets[tet1].triangles[3]].hasNeighbor(tet2))
        return tets[tet1].triangles[3];
    return -1;
}
