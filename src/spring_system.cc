#include "spring_system.h"
#include <iostream>
#include <glm/gtx/string_cast.hpp>

SpringSystem::SpringSystem(
        std::vector<glm::vec4> particles,
        std::vector<glm::uvec2> connections,
        std::vector<glm::uvec2> shear,
        std::vector<glm::uvec2> bending,
        std::vector<bool> fixed,
        std::vector<glm::uvec3> faces,
        std::vector<glm::vec4> block_vertices,
        std::vector<glm::uvec3> block_faces,
        bool col
        ) : particles(particles), connections(connections), shear(shear), bending(bending),
    fixed(fixed), faces(faces), block_vertices(block_vertices), block_faces(block_faces),
    col(col),
    velocities(particles.size(), glm::vec4(0))
{
    int p1 = connections[0][0];
    int p2 = connections[0][1];
    glm::vec4 vec = particles[p1]-particles[p2];
    float length = glm::length(vec);
    spring_length = length;
    p1 = shear[0][0];
    p2 = shear[0][1];
    vec = particles[p1]-particles[p2];
    length = glm::length(vec);
    shear_length = length;
    p1 = bending[0][0];
    p2 = bending[0][1];
    vec = particles[p1]-particles[p2];
    length = glm::length(vec);
    bending_length = length;
    unconstrained_positions.resize(particles.size());
}

void SpringSystem::satisfyConstraints() {
    int num_iter = 1;
    for(int i = 0; i < num_iter; ++i) {
        // go through every particle
        for(int p_num = 0; p_num < particles.size(); ++p_num) {
            float best_time = time;
            int face = -1;
            glm::vec4 pos = particles[p_num];
            glm::vec4 vel = velocities[p_num];
            if(unconstrained_positions[p_num][1] < -0.75f) {
                unconstrained_positions[p_num][1] = -0.75f;
                vel[1] = 0;
                velocities[p_num] = vel;
            }
            bool in_block = true;
            float x = unconstrained_positions[p_num][0];
            float x_correction = 0;
            bool x_plus = true;
            if (x > -.5 && x < .5) {
                float minus = x + .5;
                float plus = .5 - x;
                if (minus < plus) {
                    x_correction = minus;
                    x_plus = false;
                } else {
                    x_correction = plus;
                }
            } else {
                in_block = false;
            }
            float y = unconstrained_positions[p_num][1];
            float y_correction = 0;
            bool y_plus = true;
            if (y > -.5 && y < .5) {
                float minus = y + .5;
                float plus = .5 - y;
                if (minus < plus) {
                    y_correction = minus;
                    y_plus = false;
                } else {
                    y_correction = plus;
                }
            } else {
                in_block = false;
            }
            float z = unconstrained_positions[p_num][2];
            float z_correction = 0;
            bool z_plus = true;
            if (z > -.5 && z < .5) {
                float minus = z + .5;
                float plus = .5 - z;
                if (minus < plus) {
                    z_correction = minus;
                    z_plus = false;
                } else {
                    z_correction = plus;
                }
            } else {
                in_block = false;
            }
            if (in_block) {
                int side = 0;
                if (y_correction < x_correction) side = 1;
                if (z_correction < y_correction) side = 2;
                float pos = .55;
                if (side == 0 && !x_plus) pos = -.55;
                if (side == 1 && !y_plus) pos = -.55;
                if (side == 2 && !z_plus) pos = -.55;
                unconstrained_positions[p_num][side] = pos;
                vel[side] = 0;
                velocities[p_num] = vel;
            }
            if(col) continue;
            if(glm::length(vel) == 0) continue;
            glm::vec3 pos3 = glm::vec3(pos);
            glm::vec3 vel3 = glm::vec3(vel);
            // go through every face, see which is the first one interesected
            for(int f_num = 0; f_num < faces.size(); ++f_num) {
                // if there is one, dont change position
                int f_a = faces[f_num][0];
                int f_b = faces[f_num][1];
                int f_c = faces[f_num][2];
                if (p_num == f_a
                        || p_num == f_b
                        || p_num == f_c)
                    continue;
                glm::vec3 a = glm::vec3(unconstrained_positions[f_a]);
                glm::vec3 b = glm::vec3(unconstrained_positions[f_b]);
                glm::vec3 c = glm::vec3(unconstrained_positions[f_c]);
                glm::vec3 ab = b-a;
                glm::vec3 bc = c-b;
                glm::vec3 ca = a-c;
                glm::vec3 n = glm::normalize(glm::cross(ab,-ca)); // check
                if(glm::dot(n,vel3) >= -0.0001) continue;
                float t = glm::dot((a-pos3),n)/glm::dot(vel3,n);

                if (t <=0 || t >= best_time) continue;

                glm::vec3 p = pos3 + t*vel3;

                glm::vec3 ap = p - a;
                glm::vec3 bp = p - b;
                glm::vec3 cp = p - c;

                if(glm::dot(glm::cross(ab, ap), n) < 0) continue;
                if(glm::dot(glm::cross(bc, bp), n) < 0) continue;
                if(glm::dot(glm::cross(ca, cp), n) < 0) continue;

                best_time = t;
                face = f_num;
                // later wiggle that big particle
            }
            if(face == -1) continue;

            glm::vec3 a;
            glm::vec3 b;
            glm::vec3 c;
            a = glm::vec3(unconstrained_positions[faces[face][0]]);
            b = glm::vec3(unconstrained_positions[faces[face][1]]);
            c = glm::vec3(unconstrained_positions[faces[face][2]]);
            glm::vec3 ab = b-a;
            glm::vec3 bc = c-b;
            glm::vec3 ca = a-c;
            glm::vec3 n = glm::normalize(glm::cross(ab,-ca));

            float t = glm::dot((a-pos3),n)/glm::dot(vel3,n);

            glm::vec3 new_vel3 = vel3-glm::dot(vel3,n)*n;

            vel = glm::vec4(new_vel3,0.0);
            unconstrained_positions[p_num] = particles[p_num]+vel*time;
            velocities[p_num] = vel;

        }
    }
}

std::vector<glm::vec4> SpringSystem::calculateK1() {
    std::vector<glm::vec4> forces(particles.size(), glm::vec4(0.0, -1.0, 0.0, 0.0));
    for(int i = 0; i < connections.size(); ++i) {
        int p1 = connections[i][0];
        int p2 = connections[i][1];
        glm::vec4 vec = particles[p1]-particles[p2];
        float length = glm::length(vec);
        float stretch = spring_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*spring_constant;
        forces[p2] += stretch*(-dir)*spring_constant;
        glm::vec4 vec2 = velocities[p1]-velocities[p2];
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for(int i = 0; i < shear.size(); ++i) {
        int p1 = shear[i][0];
        int p2 = shear[i][1];
        glm::vec4 vec = particles[p1]-particles[p2];
        float length = glm::length(vec);
        float stretch = shear_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*shear_constant;
        forces[p2] += stretch*(-dir)*shear_constant;
        glm::vec4 vec2 = velocities[p1]-velocities[p2];
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for(int i = 0; i < bending.size(); ++i) {
        int p1 = bending[i][0];
        int p2 = bending[i][1];
        glm::vec4 vec = particles[p1]-particles[p2];
        float length = glm::length(vec);
        float stretch = bending_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*bending_constant;
        forces[p2] += stretch*(-dir)*bending_constant;
        glm::vec4 vec2 = velocities[p1]-velocities[p2];
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for (int i = 0; i < particles.size(); ++i) {
        forces[i] += -air*velocities[i];
    }
    for (int i = 0; i < forces.size(); ++i) {
        forces[i] *= time;
    }
    return forces;
}

std::vector<glm::vec4> SpringSystem::calculateK2() {
    std::vector<glm::vec4> forces(particles.size(), glm::vec4(0.0, -1.0, 0.0, 0.0));
    for(int i = 0; i < connections.size(); ++i) {
        int p1 = connections[i][0];
        int p2 = connections[i][1];
        glm::vec4 vec = particles[p1]+time/(2.0f)*(velocities[p1]+k1[p1]/(2.0f))-particles[p2]-time/(2.0f)*(velocities[p2]+k1[p2]/(2.0f));
        float length = glm::length(vec);
        float stretch = spring_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*spring_constant;
        forces[p2] += stretch*(-dir)*spring_constant;
        glm::vec4 vec2 = velocities[p1]+k1[p1]/(2.0f)-velocities[p2]-k1[2]/(2.0f);
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for(int i = 0; i < shear.size(); ++i) {
        int p1 = shear[i][0];
        int p2 = shear[i][1];
        glm::vec4 vec = particles[p1]+time/(2.0f)*(velocities[p1]+k1[p1]/(2.0f))-particles[p2]-time/(2.0f)*(velocities[p2]+k1[p2]/(2.0f));
        float length = glm::length(vec);
        float stretch = shear_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*shear_constant;
        forces[p2] += stretch*(-dir)*shear_constant;
        glm::vec4 vec2 = velocities[p1]+k1[p1]/(2.0f)-velocities[p2]-k1[2]/(2.0f);
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for(int i = 0; i < bending.size(); ++i) {
        int p1 = bending[i][0];
        int p2 = bending[i][1];
        glm::vec4 vec = particles[p1]+time/(2.0f)*(velocities[p1]+k1[p1]/(2.0f))-particles[p2]-time/(2.0f)*(velocities[p2]+k1[p2]/(2.0f));
        float length = glm::length(vec);
        float stretch = bending_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*bending_constant;
        forces[p2] += stretch*(-dir)*bending_constant;
        glm::vec4 vec2 = velocities[p1]+k1[p1]/(2.0f)-velocities[p2]-k1[2]/(2.0f);
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for (int i = 0; i < particles.size(); ++i) {
        forces[i] += -air*(velocities[i]+k1[i]/(2.0f));
    }
    for (int i = 0; i < forces.size(); ++i) {
        forces[i] *= time;
    }
    return forces;
}

std::vector<glm::vec4> SpringSystem::calculateK3() {
    std::vector<glm::vec4> forces(particles.size(), glm::vec4(0.0, -1.0, 0.0, 0.0));
    for(int i = 0; i < connections.size(); ++i) {
        int p1 = connections[i][0];
        int p2 = connections[i][1];
        glm::vec4 vec = particles[p1]+time/(2.0f)*(velocities[p1]+k2[p1]/(2.0f))-particles[p2]-time/(2.0f)*(velocities[p2]+k2[p2]/(2.0f));
        float length = glm::length(vec);
        float stretch = spring_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*spring_constant;
        forces[p2] += stretch*(-dir)*spring_constant;
        glm::vec4 vec2 = velocities[p1]+k2[p1]/(2.0f)-velocities[p2]-k2[p2]/(2.0f);
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for(int i = 0; i < shear.size(); ++i) {
        int p1 = shear[i][0];
        int p2 = shear[i][1];
        glm::vec4 vec = particles[p1]+time/(2.0f)*(velocities[p1]+k2[p1]/(2.0f))-particles[p2]-time/(2.0f)*(velocities[p2]+k2[p2]/(2.0f));
        float length = glm::length(vec);
        float stretch = shear_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*shear_constant;
        forces[p2] += stretch*(-dir)*shear_constant;
        glm::vec4 vec2 = velocities[p1]+k2[p1]/(2.0f)-velocities[p2]-k2[p2]/(2.0f);
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for(int i = 0; i < bending.size(); ++i) {
        int p1 = bending[i][0];
        int p2 = bending[i][1];
        glm::vec4 vec = particles[p1]+time/(2.0f)*(velocities[p1]+k2[p1]/(2.0f))-particles[p2]-time/(2.0f)*(velocities[p2]+k2[p2]/(2.0f));
        float length = glm::length(vec);
        float stretch = bending_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*bending_constant;
        forces[p2] += stretch*(-dir)*bending_constant;
        glm::vec4 vec2 = velocities[p1]+k2[p1]/(2.0f)-velocities[p2]-k2[p2]/(2.0f);
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for (int i = 0; i < particles.size(); ++i) {
        forces[i] += -air*(velocities[i]+k2[i]/(2.0f));
    }
    for (int i = 0; i < forces.size(); ++i) {
        forces[i] *= time;
    }
    return forces;
}

std::vector<glm::vec4> SpringSystem::calculateK4() {
    std::vector<glm::vec4> forces(particles.size(), glm::vec4(0.0, -1.0, 0.0, 0.0));
    for(int i = 0; i < connections.size(); ++i) {
        int p1 = connections[i][0];
        int p2 = connections[i][1];
        glm::vec4 vec = particles[p1]+time*(velocities[p1]+k3[p1])-particles[p2]-time*(velocities[p2]+k3[p2]);
        float length = glm::length(vec);
        float stretch = spring_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*spring_constant;
        forces[p2] += stretch*(-dir)*spring_constant;
        glm::vec4 vec2 = velocities[p1]+k3[p1]-velocities[p2]-k3[p2];
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for(int i = 0; i < shear.size(); ++i) {
        int p1 = shear[i][0];
        int p2 = shear[i][1];
        glm::vec4 vec = particles[p1]+time*(velocities[p1]+k3[p1])-particles[p2]-time*(velocities[p2]+k3[p2]);
        float length = glm::length(vec);
        float stretch = shear_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*shear_constant;
        forces[p2] += stretch*(-dir)*shear_constant;
        glm::vec4 vec2 = velocities[p1]+k3[p1]-velocities[p2]-k3[p2];
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for(int i = 0; i < bending.size(); ++i) {
        int p1 = bending[i][0];
        int p2 = bending[i][1];
        glm::vec4 vec = particles[p1]+time*(velocities[p1]+k3[p1])-particles[p2]-time*(velocities[p2]+k3[p2]);
        float length = glm::length(vec);
        float stretch = bending_length - length;
        glm::vec4 dir = glm::normalize(vec);
        forces[p1] += stretch*dir*bending_constant;
        forces[p2] += stretch*(-dir)*bending_constant;
        glm::vec4 vec2 = velocities[p1]+k3[p1]-velocities[p2]-k3[p2];
        forces[p1] += -damping*glm::dot(vec2,vec)*vec/(length*length);
        forces[p2] += damping*glm::dot(vec2,vec)*vec/(length*length);
    }
    for (int i = 0; i < particles.size(); ++i) {
        forces[i] += -air*(velocities[i]+k3[i]);
    }
    for (int i = 0; i < forces.size(); ++i) {
        forces[i] *= time;
    }
    return forces;
}

std::vector<glm::vec4> SpringSystem::getNewPositions() {
    // new positions
    for(int i = 0; i < particles.size(); ++i) {
        if(fixed[i]) continue;
        unconstrained_positions[i] = particles[i] + velocities[i]*time;
    }
    // satisfy constraints
    satisfyConstraints();
    // copy new to particles
    for(int i = 0; i < particles.size(); ++i) {
        if(fixed[i]) continue;
        particles[i] = unconstrained_positions[i];
    }
    k1 = calculateK1();
    k2 = calculateK2();
    k3 = calculateK3();
    k4 = calculateK4();
    for(int i = 0; i < particles.size(); ++i) {
        if(fixed[i]) {
            velocities[i] = glm::vec4(0);
            continue;
        }
        velocities[i] += (k1[i]+(2.0f)*k2[i]+(2.0f)*k3[i]+k4[i])/(6.0f);
    }
    return particles;
}
