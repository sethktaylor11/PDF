#include "peridynamic_system.h"
#include <iostream>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

using namespace std;

PeridynamicSystem::PeridynamicSystem(
        vector<glm::vec4> nodes,
        vector<bool> fixedNodes,
        vector<vector<int>> tets,
        vector<glm::uvec3> faces,
        vector<int> boundary,
        vector<int> neighbors
        ) : nodes(nodes), faces(faces), boundary(boundary),
    neighbors(neighbors), fixed(tets.size(), false),
    tetNeighborhoods(nodes.size(), vector<int>()),
    particles(tets.size(), glm::vec4(0)),
    init_vecs(tets.size(), vector<glm::vec4>()),
    init_lengths(tets.size(), vector<float>()),
    init_dirs(tets.size(), vector<glm::vec4>()),
    neighborhoods(tets.size(), vector<int>()),
    broken(tets.size(), vector<bool>()),
    weights(tets.size(), vector<float>()),
    Kinv(tets.size(), glm::mat3()),
    volumes(tets.size(), 0),
    moments(tets.size(), 0),
    velocities(tets.size(), glm::vec4(0)),
    forces(tets.size(), glm::vec4(0)),
    orientations(tets.size(), glm::vec3(0)),
    angular_velocities(tets.size(), glm::vec3(0)),
    torques(tets.size(), glm::vec3(0))
{
    lambda = 2 * v * G / (1 - 2 * v);
    mu = G - eta / 2;
    std::cout << "lambda " << lambda << std::endl;
    std::cout << "mu " << mu << std::endl;
    for (uint i = 0; i < tets.size(); i++) {
        vector<int> tet = tets[i];
        int A = tet[0];
        int B = tet[1];
        int C = tet[2];
        int D = tet[3];
        tetNeighborhoods[A].push_back(i);
        tetNeighborhoods[B].push_back(i);
        tetNeighborhoods[C].push_back(i);
        tetNeighborhoods[D].push_back(i);
        glm::vec4 a = nodes[A];
        glm::vec4 b = nodes[B];
        glm::vec4 c = nodes[C];
        glm::vec4 d = nodes[D];
	glm::vec4 p = (a + b + c + d) / 4.0f;
        particles[i] = p;
        float V = glm::dot(glm::vec3(b-a),glm::cross(glm::vec3(c-a),glm::vec3(d-a)))/6;
        volumes[i] = V;
	float r1 = glm::length(a-p);
	float r2 = glm::length(b-p);
	float r3 = glm::length(c-p);
	float r4 = glm::length(d-p);
	float r = (r1 + r2 + r3 + r4) / 4;
	moments[i] = 2 * V * glm::pow(r,2) / 5;
        if (fixedNodes[A] || fixedNodes[B] || fixedNodes[C] || fixedNodes[D]) fixed[i] = true;
    }

    for (uint i = 0; i < particles.size()-1; i++) {
        glm::vec4 pos = particles[i];
        for (uint j = i+1; j < particles.size(); j++) {
            glm::vec4 vec = particles[j] - pos;
            float length = glm::length(vec);
            if (length < delta) {
                neighborhoods[i].push_back(j);
                neighborhoods[j].push_back(i);
                init_vecs[i].push_back(vec);
                init_vecs[j].push_back(-vec);
                init_lengths[i].push_back(length);
                init_lengths[j].push_back(length);
                glm::vec4 dir = vec / length;
                init_dirs[i].push_back(dir);
                init_dirs[j].push_back(-dir);
                float weight = delta / length;
                weights[i].push_back(weight);
                weights[j].push_back(weight);
            }
        }
        broken[i].resize(neighborhoods[i].size(),false);
    }

    for (uint i = 0; i < particles.size()-1; i++) {
        glm::mat3 K = glm::mat3(0);
	for (uint j = 0; j < neighborhoods[i].size(); j++ ) {
            glm::vec3 init_vec = glm::vec3(init_vecs[i][j]);
            K += weights[i][j] * glm::outerProduct(init_vec,init_vec) * volumes[neighborhoods[i][j]];
	}
	Kinv[i] = glm::inverse(K);
    }
}

void PeridynamicSystem::calculateNewPositions() {
    // midway velocity
    for (uint i = 0; i < particles.size(); i++) {
        if (fixed[i]) continue;
        velocities[i] += (1-damping)*forces[i]*time/(2*volumes[i]);
	angular_velocities[i] += (1-damping)*torques[i]*time/(2*moments[i]);
    }
    // new particle positions
    for (uint i = 0; i < particles.size(); i++) {
        // damping
        if (fixed[i]) continue;
        particles[i] += velocities[i]*time;
	orientations[i] += angular_velocities[i]*time;
    }
    // new node positions
    for (uint i = 0; i < nodes.size(); i++) {
        glm::vec4 velocity = glm::vec4(0);
	float weight = 0;
        for (uint j = 0; j < tetNeighborhoods[i].size(); j++) {
            int p = tetNeighborhoods[i][j];
	    float w = volumes[p];
	    weight += w;
            velocity += w * velocities[p];
	    velocity += w * glm::vec4(glm::cross(angular_velocities[p],glm::vec3(particles[p]-nodes[i])),0);
        }
        velocity /= weight;
	/*
	glm::vec3 angular = glm::vec3(0);
	float moment = 0;
        for (uint j = 0; j < tetNeighborhoods[i].size(); j++) {
            int p = tetNeighborhoods[i][j];
	    float m = moments[p];
	    moment += m;
	    angular += m * glm::cross(angular_velocities[p],glm::vec3(particles[p]-nodes[i]));
        }
	angular /= moment;
        nodes[i] += (velocity+glm::vec4(angular,0))*time;
	*/
        nodes[i] += velocity*time;
    }
    // calculate forces
    calculateForces();
    // new velocities
    for (uint i = 0; i < particles.size(); i++) {
        if (fixed[i]) continue;
        velocities[i] += (1-damping)*forces[i]*time/(2*volumes[i]);
	angular_velocities[i] += (1-damping)*torques[i]*time/(2*moments[i]);
    }
}

void PeridynamicSystem::calculateForces() {
    // reset forces
    forces = vector<glm::vec4>(particles.size(), glm::vec4(0));
    torques = vector<glm::vec3>(particles.size(), glm::vec3(0));

    // compute relevant values from deformed positions
    vector<vector<glm::vec3>> vecs(particles.size());
    //vector<vector<float>> lengths(particles.size());
    //vector<vector<glm::vec4>> dirs(particles.size());
    //vector<vector<float>> extensions(particles.size());
    //vector<vector<float>> stretches(particles.size());
    vector<vector<glm::vec3>> u_theta_vecs(particles.size());
    vector<vector<glm::vec3>> orientation_vecs(particles.size());

    for (uint i = 0; i < particles.size(); i++) {
        vecs[i].resize(neighborhoods[i].size());
        /*
        lengths[i].resize(neighborhoods[i].size());
        dirs[i].resize(neighborhoods[i].size());
        extensions[i].resize(neighborhoods[i].size());
        stretches[i].resize(neighborhoods[i].size());
	*/
        u_theta_vecs[i].resize(neighborhoods[i].size());
        orientation_vecs[i].resize(neighborhoods[i].size());
        glm::vec4 pos = particles[i];
        glm::vec3 orientation = orientations[i];
        for (uint j = 0; j < neighborhoods[i].size(); j++) {
            if (broken[i][j]) continue;
            int p2 = neighborhoods[i][j];
            glm::vec4 vec = particles[p2] - pos;
	    /*
            float length = glm::length(vec);
            glm::vec4 dir = vec / length;
            float init_length = init_lengths[i][j];
            float extension = length - init_length;
            float stretch = extension / init_length;
	    */
	    glm::vec4 init_vec = init_vecs[i][j];
	    glm::vec3 u_theta_vec = glm::vec3(vec - init_vec) - glm::cross((orientation + orientations[p2])/2.0f,glm::vec3(init_vec));
	    glm::vec3 orientation_vec = orientations[p2] - orientation;
            /*
            if (stretch >= .5) {
                broken[i][j] = true;
                continue;
            }
            */
            vecs[i][j] = glm::vec3(vec);
            //lengths[i][j] = length;
            //dirs[i][j] = dir;
            //extensions[i][j] = extension;
            //stretches[i][j] = stretch;
	    u_theta_vecs[i][j] = u_theta_vec;
	    orientation_vecs[i][j] = orientation_vec;
        }
    }

    /*
    // compute dilatations
    vector<float> thetas(particles.size());

    for (uint i = 0; i < particles.size(); i++) {
        float sum = 0.0f;
        for (uint j = 0; j < neighborhoods[i].size(); j++) {
            if (broken[i][j]) continue;
            int k = neighborhoods[i][j];
            float weight = weights[i][j];
            glm::vec4 init_vec = init_vecs[i][j];
            glm::vec4 dir = dirs[i][j];
            float stretch = stretches[i][j];
            sum += weight * stretch * glm::dot(dir,init_vec) * volumes[k];
        }
        float theta = 9.0f / (4.0f * glm::pi<float>() * glm::pow(delta, 4)) * sum;
        thetas[i] = theta;
    }
    */

    // compute gammas and kappas
    vector<glm::mat3> gammas(particles.size());
    vector<glm::mat3> kappas(particles.size());

    for (uint i = 0; i < particles.size(); i++) {
        glm::mat3 gamma = glm::mat3(0.0f);
        glm::mat3 kappa = glm::mat3(0.0f);
        for (uint j = 0; j < neighborhoods[i].size(); j++) {
            if (broken[i][j]) continue;
            int k = neighborhoods[i][j];
            float weight = weights[i][j];
            glm::vec3 init_vec = glm::vec3(init_vecs[i][j]);
            //glm::vec3 vec = glm::vec3(vecs[i][j]);
            glm::vec3 vec = glm::vec3(u_theta_vecs[i][j]);
            gamma += weight * glm::outerProduct(vec,init_vec) * volumes[k];
            glm::vec3 orientation_vec = orientation_vecs[i][j];
            kappa += weight * glm::outerProduct(orientation_vec,init_vec) * volumes[k];
        }
	gammas[i] = gamma * Kinv[i];
	kappas[i] = kappa * Kinv[i];
    }

    // compute sigmas and mus
    vector<glm::mat3> sigmasKinv(particles.size());
    vector<glm::mat3> musKinv(particles.size());

    glm::mat3 I = glm::mat3(1.0f);
    for (uint i = 0; i < particles.size(); i++) {
	    glm::mat3 g = gammas[i];
	    sigmasKinv[i] = (lambda * (g[0][0] + g[1][1] + g[2][2]) * I + (mu + eta) * glm::transpose(g) + mu * g) * Kinv[i];
	    glm::mat3 k = kappas[i];
	    musKinv[i] = (alpha * (k[0][0] + k[1][1] + k[2][2]) * I + beta * k + gamma * glm::transpose(k)) * Kinv[i];
    }

    // compute forces
    for (uint i = 0; i < particles.size(); i++) { 
        int p1 = i;
        //float theta1 = thetas[p1];
        for (uint j = 0; j < neighborhoods[i].size(); j++) {
            if (broken[i][j]) continue;
            int p2 = neighborhoods[i][j];
            if (p2 < p1) continue;
            float weight = weights[i][j];
	    glm::vec3 vec = vecs[i][j];
	    /*
            glm::vec4 dir = dirs[i][j];
            glm::vec4 init_dir = init_dirs[i][j];
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
	    */
	    glm::vec3 init_vec = glm::vec3(init_vecs[i][j]);
	    glm::vec3 Tp1p2 = glm::vec4(weight * sigmasKinv[p1] * init_vec, 0.0f);
	    glm::vec3 Tp2p1 = glm::vec4(weight * sigmasKinv[p2] * -init_vec, 0.0f);
            forces[p1] += glm::vec4(Tp2p1 * volumes[p2],0);
            forces[p1] -= glm::vec4(Tp1p2 * volumes[p2],0);
            forces[p2] += glm::vec4(Tp1p2 * volumes[p1],0);
            forces[p2] -= glm::vec4(Tp2p1 * volumes[p1],0);
	    glm::vec3 Mp1p2 = weight * musKinv[p1] * init_vec;
	    glm::vec3 Mp2p1 = weight * musKinv[p2] * init_vec;
	    torques[p1] += Mp2p1 * volumes[p2] + glm::cross(vec,Tp2p1) / 2.0f * volumes[p2];
	    torques[p1] -= Mp1p2 * volumes[p2] + glm::cross(vec,Tp1p2) / 2.0f * volumes[p2];
	    torques[p2] += Mp1p2 * volumes[p1] + glm::cross(-vec,Tp1p2) / 2.0f * volumes[p1];
	    torques[p2] -= Mp2p1 * volumes[p1] + glm::cross(-vec,Tp2p1) / 2.0f * volumes[p1];
        }
        // forces[p1] += glm::vec4(0,-1,0,0) * volumes[i];
    }

    //glm::vec4 pressure = glm::vec4(0);
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
        int p = neighbors[i];
	glm::vec4 P = particles[p];
	glm::vec3 force = -1.0f * n * area; // volume[i]
        forces[p] += glm::vec4(force,0);
        //pressure += glm::vec4(force,0);
        //glm::vec4 F = (A+B+C)/3.0f;
        //glm::vec3 torque = glm::cross(glm::vec3(F-P),force);
        //cout << "torque " << glm::to_string(torque) << endl;
	//torques[p] += torque;
    }
    /*
    cout << "pressure " << glm::to_string(pressure) << endl;
    glm::vec3 angular = glm::vec3(0);
    for (uint i = 0; i < particles.size(); i++) {
	angular += glm::cross(glm::vec3(particles[i])-glm::vec3(0,3,0),glm::vec3(forces[i]));
    }
    cout << "angular " << glm::to_string(angular) << endl;
    assert(glm::length(pressure) == 0);
    */
}
