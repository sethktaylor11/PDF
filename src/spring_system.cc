#include "spring_system.h"
#include <iostream>
#include <glm/gtx/string_cast.hpp>

SpringSystem::SpringSystem(
        std::vector<glm::vec4> particles,
        std::vector<bool> fixed
        ) : particles(particles), fixed(fixed),
    init_vecs(particles.size(), std::vector<glm::vec4>()),
    init_lengths(particles.size(), std::vector<float>()),
    init_dirs(particles.size(), std::vector<glm::vec4>()),
    neighborhoods(particles.size(), std::vector<int>()),
    broken(particles.size(), std::vector<bool>()),
    weights(particles.size(), std::vector<float>()),
    velocities(particles.size(), glm::vec4(0)), forces(particles.size(), glm::vec4(0))
{
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
}

std::vector<glm::vec4> SpringSystem::getNewPositions() {
    // midway velocity
    for (uint i = 0; i < particles.size(); i++) {
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
    // reset forces
    forces = std::vector<glm::vec4>(particles.size(), glm::vec4(0));

    // compute relevant values from deformed positions
    std::vector<std::vector<glm::vec4>> vecs(particles.size());
    std::vector<std::vector<float>> lengths(particles.size());
    std::vector<std::vector<glm::vec4>> dirs(particles.size());
    std::vector<std::vector<float>> extensions(particles.size());
    std::vector<std::vector<float>> stretches(particles.size());

    for (uint i = 0; i < particles.size(); i++) {
        vecs[i].resize(neighborhoods[i].size());
        lengths[i].resize(neighborhoods[i].size());
        dirs[i].resize(neighborhoods[i].size());
        extensions[i].resize(neighborhoods[i].size());
        stretches[i].resize(neighborhoods[i].size());
        glm::vec4 pos = particles[i];
        for (uint j = 0; j < neighborhoods[i].size(); j++) {
            if (broken[i][j]) continue;
            int p2 = neighborhoods[i][j];
            glm::vec4 vec = particles[p2] - pos;
            float length = glm::length(vec);
            glm::vec4 dir = vec / length;
            float init_length = init_lengths[i][j];
            float extension = length - init_length;
            float stretch = extension / init_length;
            if (stretch >= .5) {
                broken[i][j] = true;
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
    std::vector<float> thetas(particles.size());

    for (uint i = 0; i < particles.size(); i++) {
        float sum = 0.0f;
        for (uint j = 0; j < neighborhoods[i].size(); j++) {
            if (broken[i][j]) continue;
            int k = neighborhoods[i][j];
            float weight = weights[i][j];
            glm::vec4 init_vec = init_vecs[i][j];
            glm::vec4 dir = dirs[i][j];
            float stretch = stretches[i][j];
            sum += weight * stretch * glm::dot(dir,init_vec) * volume; //volume[k];
        }
        float theta = 9.0f / (4.0f * glm::pi<float>() * glm::pow(delta, 4)) * sum;
        thetas.push_back(theta);
    }

    //compute forces
    for (uint i = 0; i < particles.size(); i++) { 
        int p1 = i;
        float theta1 = thetas[p1];
        for (uint j = 0; j < neighborhoods[i].size(); j++) {
            if (broken[i][j]) continue;
            int p2 = neighborhoods[i][j];
            if (p2 < p1) continue;
            float weight = weights[i][j];
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
            forces[p1] += Tp2p1 * volume; // volume[p2];
            forces[p1] -= Tp1p2 * volume; // volume[p2];
            forces[p2] += Tp1p2 * volume; // volume[p1];
            forces[p2] -= Tp2p1 * volume; // volume[p1];
        }
        forces[p1] += glm::vec4(0,-1,0,0);
    }
}