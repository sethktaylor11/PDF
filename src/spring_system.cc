#include "spring_system.h"
#include <iostream>
#include <glm/gtx/string_cast.hpp>

SpringSystem::SpringSystem(
        std::vector<glm::vec4> particles,
        std::vector<bool> fixed
        ) : particles(particles), fixed(fixed), init_particles(particles),
    neighborhoods(particles.size(), std::vector<int>()),
    weights(particles.size(), std::vector<float>()),
    velocities(particles.size(), glm::vec4(0)), forces(particles.size(), glm::vec4(0))
{
    for (uint i = 0; i < particles.size()-1; i++) {
        for (uint j = i+1; j < particles.size(); j++) {
            float length = glm::length(particles[j] - particles[i]);
            if (length < delta) {
                neighborhoods[i].push_back(j);
                neighborhoods[j].push_back(i);
                float weight = delta/length;
                weights[i].push_back(weight);
                weights[j].push_back(weight);
            }
        }
    }
}

std::vector<glm::vec4> SpringSystem::getNewPositions() {
    // midway velocity
    for (uint i = 0; i < particles.size(); i++) {
        //std::cout << glm::to_string(forces[i]) << std::endl;
        if (fixed[i]) continue;
        velocities[i] += forces[i]*time/2.0f;
    }
    // new positions
    for (uint i = 0; i < particles.size(); i++) {
        if (fixed[i]) continue;
        particles[i] += velocities[i]*time;
    }
    // calculate forces
    calculateForces();
    // new velocities
    for (uint i = 0; i < particles.size(); i++) {
        if (fixed[i]) continue;
        velocities[i] += forces[i]*time/2.0f;
    }
    return particles;
}

void SpringSystem::calculateForces() {
    forces = std::vector<glm::vec4>(particles.size(), glm::vec4(0));

    for (uint i = 0; i < particles.size(); i++) { 
        int p1 = i;
        for (uint j = 0; j < neighborhoods[i].size(); j++) {
            int p2 = neighborhoods[i][j];
            float weight = weights[i][j];
            glm::vec4 Tp2p1 = calculateForceAOnB(p2,p1,weight);
            glm::vec4 Tp1p2 = calculateForceAOnB(p1,p2,weight);
            forces[p1] += Tp2p1 * volume; // volume[p2];
            forces[p2] += Tp1p2 * volume; // volume[p1];
        }
        forces[p1] += glm::vec4(0,-1,0,0);
    }
}

glm::vec4 SpringSystem::calculateForceAOnB(int j, int i, float weight) {
    glm::vec4 vec = particles[j] - particles[i];
    glm::vec4 init_vec = init_particles[j] - init_particles[i];
    float length = glm::length(vec);
    float init_length = glm::length(init_vec);
    glm::vec4 dir = vec / length;
    glm::vec4 init_dir = init_vec / init_length;
    float theta = 9.0f / (4.0f * glm::pi<float>() * glm::pow(delta, 4));
    float A_dil = 4.0f * weight * a * glm::dot(dir,init_dir) * theta;
    float extension = length - init_length;
    float A_dev = 4.0f * weight * b * (extension - delta / 4.0f * glm::dot(dir,init_dir) * theta);
    float A = A_dil + A_dev;
    return 0.5f * A * dir;
}

float SpringSystem::calculateDilatation(int i) {
    float sum = 0.0f;
    for (uint j = 0; j < neighborhoods[i].size(); j++) {
        int k = neighborhoods[i][j];
        float weight = weights[i][j];
        glm::vec4 vec = particles[k] - particles[i];
        glm::vec4 init_vec = init_particles[k] - init_particles[i];
        float length = glm::length(vec);
        float init_length = glm::length(init_vec);
        glm::vec4 dir = vec / length;
        float stretch = length / init_length - 1;
        sum += weight * stretch * glm::dot(dir,init_vec) * volume; //volume[k];
    }
    return 9.0f / (4.0f * glm::pi<float>() * glm::pow(delta, 4)) * sum;
}
