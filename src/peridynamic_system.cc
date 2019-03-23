#include "peridynamic_system.h"
#include <iostream>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

using namespace std;
        
Tet::Tet(
        vector<int> roommates,
        bool fixed,
        glm::vec4 pos,
        float vol
        ) : roommates(roommates), fixed(fixed), position(pos),
    volume(vol), velocity(glm::vec4(0)), force(glm::vec4(0))
{
}

PeridynamicSystem::PeridynamicSystem(
        vector<glm::vec4> nodes,
        vector<bool> fixedNodes,
        vector<vector<int>> eles,
        vector<glm::uvec3> faces,
        vector<int> boundary,
        vector<int> neighbors,
	vector<vector<int>> roommates
        ) : nodes(nodes), faces(faces), tets(eles.size(), Tet()),
    triangles(faces.size(), Triangle()), points(nodes.size(), Point())
{
    for (uint i = 0; i < faces.size(); i++) {
        triangles[i].boundary = boundary[i];
	triangles[i].neighbors.push_back(neighbors[i]);
    }

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
	tets[i] = Tet(roommates[i], fixed, position, volume);
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
		/*
		if (neighbors(i,j)) {
                    deleteFaces(i);
		    deleteFaces(j);
		}
		*/
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

    //compute forces
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
