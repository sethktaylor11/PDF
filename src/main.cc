#include <GL/glew.h>
#include <dirent.h>

#include "procedure_geometry.h"
#include "render_pass.h"
#include "config.h"
#include "gui.h"
//#include "spring_system.h"
#include "peridynamic_system.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <string.h>

#include <glm/gtx/component_wise.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/io.hpp>
#include <debuggl.h>

using namespace std;

int window_width = 800, window_height = 600;
const string window_title = "PDF";

const char* vertex_shader =
#include "shaders/default.vert"
;

const char* geometry_shader =
#include "shaders/default.geom"
;

const char* floor_fragment_shader =
#include "shaders/floor.frag"
;

const char* block_vertex_shader =
#include "shaders/block.vert"
;

const char* block_geometry_shader =
#include "shaders/block.geom"
;

const char* block_fragment_shader =
#include "shaders/block.frag"
;

const char* box_vertex_shader =
#include "shaders/box.vert"
;

const char* box_geometry_shader =
#include "shaders/box.geom"
;

const char* box_fragment_shader =
#include "shaders/box.frag"
;

// FIXME: Add more shaders here.

void ErrorCallback(int error, const char* description) {
    cerr << "GLFW Error: " << description << "\n";
}

GLFWwindow* init_glefw()
{
    if (!glfwInit())
        exit(EXIT_FAILURE);
    glfwSetErrorCallback(ErrorCallback);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    auto ret = glfwCreateWindow(window_width, window_height, window_title.data(), nullptr, nullptr);
    CHECK_SUCCESS(ret != nullptr);
    glfwMakeContextCurrent(ret);
    glewExperimental = GL_TRUE;
    CHECK_SUCCESS(glewInit() == GLEW_OK);
    glGetError();  // clear GLEW's error for it
    glfwSwapInterval(1);
    const GLubyte* renderer = glGetString(GL_RENDERER);  // get renderer string
    const GLubyte* version = glGetString(GL_VERSION);    // version as a string
    cout << "Renderer: " << renderer << "\n";
    cout << "OpenGL version supported:" << version << "\n";

    return ret;
}

void CreateBox(vector<glm::vec4>& particle_pos,
        vector<bool>& fixed_points)
{
    particle_pos.clear();
    fixed_points.clear();

    float length = 8.0;
    float width = 1.0;
    float height = 1.0;
    float inc = 0.25;

    for (float i = -length/2; i <= length/2+inc/2; i+=inc) {
        for (float j = 8; j <= 8+(height+inc/2); j+=inc) {
            for (float k = -width/2; k <= width/2+inc/2; k+=inc) {
                particle_pos.push_back(glm::vec4(i,j,k,1.0));
                if(i < -length/2 + inc/2) {
                    fixed_points.push_back(true);
                } else {
                    fixed_points.push_back(false);
                }
            }
        }
    }
}

void CreateBlock(vector<glm::vec4>& obj_vertices,
        vector<glm::uvec3>& obj_faces)
{
    obj_vertices.clear();
    obj_faces.clear();
    float l = .25;
    for(float i = -l/2; i <= l; i+=l) {
        for(float j = -l/2; j <= l; j+=l) {
            for(float k = -l/2; k <= l; k+=l) {
                obj_vertices.push_back(glm::vec4(i,j,k, 1.0f));
            }
        }
    }

    // Face 1
    obj_faces.push_back(glm::uvec3(0, 1, 3)); 
    obj_faces.push_back(glm::uvec3(0, 3, 2)); 
    // Face 2                             
    obj_faces.push_back(glm::uvec3(0, 4, 5)); 
    obj_faces.push_back(glm::uvec3(0, 5, 1)); 
    // Face 3                             
    obj_faces.push_back(glm::uvec3(0, 2, 6)); 
    obj_faces.push_back(glm::uvec3(0, 6, 4)); 
    // Face 4                           
    obj_faces.push_back(glm::uvec3(1, 5, 7));
    obj_faces.push_back(glm::uvec3(1, 7, 3)); 
    // Face 5                           
    obj_faces.push_back(glm::uvec3(2, 3, 7)); 
    obj_faces.push_back(glm::uvec3(2, 7, 6)); 
    // Face 6                           
    obj_faces.push_back(glm::uvec3(4, 6, 7)); 
    obj_faces.push_back(glm::uvec3(4, 7, 5)); 
}

void readNodes(vector<glm::vec4>& nodes, vector<bool>& fixedNodes) {
    ifstream nodesFile;
    nodesFile.open("tank.1.node");
    int nP, d, nAN, nBN;
    nodesFile >> nP >> d >> nAN >> nBN;
    nodes.resize(nP);
    fixedNodes.resize(nP);
    int p;
    float x, y, z;
    for (int i = 0; i < nP; i++) {
        nodesFile >> p >> x >> y >> z;
        nodes[p] = glm::vec4(x,y,z,1);
        //if (x == -4.0f) fixedNodes[p] = true;
    }
}

void readFaces(vector<glm::uvec3>& faces, vector<int>& boundary, vector<int>& neighbors) {
    ifstream facesFile;
    facesFile.open("tank.1.face");
    int nF, nBF;
    facesFile >> nF >> nBF;
    faces.resize(nF);
    boundary.resize(nF);
    neighbors.resize(nF);
    for (int i = 0; i < nF; i++) {
        int f, A, B, C, b, n1, n2;
        facesFile >> f >> A >> B >> C >> b >> n1 >> n2;
        faces[f] = glm::uvec3(A,C,B);
        boundary[f] = b;
        neighbors[f] = n1 + n2 + 1;
    }
}

void readTets(vector<vector<int>>& tets) {
    ifstream tetsFile;
    tetsFile.open("tank.1.ele");
    int nT, nN, nAT;
    tetsFile >> nT >> nN >> nAT;
    tets.resize(nT);
    for (int i = 0; i < nT; i++) {
        int t, A, B, C, D;
        tetsFile >> t >> A >> B >> C >> D;
        vector<int> tet = vector<int>(4);
        tet[0] = A;
        tet[1] = B;
        tet[2] = C;
        tet[3] = D;
        tets[t] = tet;
    }
}

int main(int argc, char* argv[])
{
    bool collisionDetection = false;
    GLFWwindow *window = init_glefw();
    GUI gui(window, collisionDetection);

    // Floor

    vector<glm::vec4> floor_vertices;
    vector<glm::uvec3> floor_faces;
    create_floor(floor_vertices, floor_faces);

    // Read Nodes

    vector<glm::vec4> nodes;
    vector<bool> fixedNodes;
    readNodes(nodes, fixedNodes);

    // Read Faces

    vector<glm::uvec3> faces;
    vector<int> boundary;
    vector<int> neighbors;
    readFaces(faces, boundary, neighbors);

    // Read Tets

    vector<vector<int>> tets;
    readTets(tets);

    PeridynamicSystem* ps = new PeridynamicSystem(nodes,fixedNodes,tets,faces,boundary,neighbors);

    /*
    // Box

    vector<glm::vec4> particles;
    vector<bool> fixed;
    CreateBox(particles,fixed);
    SpringSystem* ss = new SpringSystem(particles,fixed);

    // Block

    vector<glm::vec4> block_vertices;
    vector<glm::uvec3> block_faces;
    CreateBlock(block_vertices, block_faces);
    */

    glm::vec4 light_position = glm::vec4(0.0f, 100.0f, 0.0f, 1.0f);
    MatrixPointers mats; // Define MatrixPointers here for lambda to capture
    /*
     * In the following we are going to define several lambda functions to bind Uniforms.
     *
     * Introduction about lambda functions:
     *      http://en.cppreference.com/w/cpp/language/lambda
     *      http://www.stroustrup.com/C++11FAQ.html#lambda
     */
    /*
     * The following lambda functions are defined to bind uniforms
     */
    auto matrix_binder = [](int loc, const void* data) {
        glUniformMatrix4fv(loc, 1, GL_FALSE, (const GLfloat*)data);
    };
    auto vector_binder = [](int loc, const void* data) {
        glUniform4fv(loc, 1, (const GLfloat*)data);
    };
    auto vector3_binder = [](int loc, const void* data) {
        glUniform3fv(loc, 1, (const GLfloat*)data);
    };
    auto float_binder = [](int loc, const void* data) {
        glUniform1fv(loc, 1, (const GLfloat*)data);
    };
    auto int_binder = [](int loc, const void* data) {
        glUniform1iv(loc, 1, (const GLint*)data);
    };
    auto block_delta_binder = [&ps](int loc, const void* data) {
        glUniform4fv(loc, ps->particles.size(), (const GLfloat*)data);
    };
    auto vertex_positions_binder = [&ps](int loc, const void* data) {
        glUniform4fv(loc, ps->nodes.size(), (const GLfloat*)data);
    };

    /*
     * The lambda functions below are used to retrieve data
     */
    auto std_model_data = [&mats]() -> const void* {
        return mats.model;
    }; // This returns point to model matrix
    glm::mat4 floor_model_matrix = glm::mat4(1.0f);
    auto floor_model_data = [&floor_model_matrix]() -> const void* {
        return &floor_model_matrix[0][0];
    }; // This return model matrix for the floor.
    auto std_view_data = [&mats]() -> const void* {
        return mats.view;
    };
    auto std_camera_data  = [&gui]() -> const void* {
        return &gui.getCamera()[0];
    };
    auto std_proj_data = [&mats]() -> const void* {
        return mats.projection;
    };
    auto std_light_data = [&light_position]() -> const void* {
        return &light_position[0];
    };
    auto alpha_data  = [&gui]() -> const void* {
        static const float transparet = 0.5; // Alpha constant goes here
        static const float non_transparet = 1.0;
        if (gui.isTransparent())
            return &transparet;
        else
            return &non_transparet;
    };
    auto block_delta_data = [&ps]() -> const void* {
        return ps->particles.data();
    };
    auto vertex_positions_data = [&ps]() -> const void* {
        return ps->nodes.data();
    };
    auto box_delta_data = []() -> const void* {
        static const float box_delta = 3.0f;
        return &box_delta;
    };

    // FIXME: add more lambdas for data_source if you want to use RenderPass.
    //        Otherwise, do whatever you like here

    ShaderUniform std_model = { "model", matrix_binder, std_model_data };
    ShaderUniform floor_model = { "model", matrix_binder, floor_model_data };
    ShaderUniform std_view = { "view", matrix_binder, std_view_data };
    ShaderUniform std_camera = { "camera_position", vector3_binder, std_camera_data };
    ShaderUniform std_proj = { "projection", matrix_binder, std_proj_data };
    ShaderUniform std_light = { "light_position", vector_binder, std_light_data };
    ShaderUniform object_alpha = { "alpha", float_binder, alpha_data };
    ShaderUniform block_delta = { "block_delta", block_delta_binder, block_delta_data };
    ShaderUniform vertex_positions = { "vertex_positions", vertex_positions_binder, vertex_positions_data };
    ShaderUniform box_delta = { "box_delta", float_binder, box_delta_data };
    // FIXME: define more ShaderUniforms for RenderPass if you want to use it.
    //        Otherwise, do whatever you like here

    // Floor render pass
    RenderDataInput floor_pass_input;
    floor_pass_input.assign(0, "vertex_position", floor_vertices.data(), floor_vertices.size(), 4, GL_FLOAT);
    floor_pass_input.assignIndex(floor_faces.data(), floor_faces.size(), 3);
    RenderPass floor_pass(-1,
            floor_pass_input,
            { vertex_shader, geometry_shader, floor_fragment_shader},
            { floor_model, std_view, std_proj, std_light },
            { "fragment_color" }
            );

    /*
    RenderDataInput block_pass_input;
    block_pass_input.assign(0, "vertex_position", block_vertices.data(), block_vertices.size(), 4, GL_FLOAT);
    block_pass_input.assignIndex(block_faces.data(), block_faces.size(), 3);
    RenderPass block_pass(-1, block_pass_input,
            { block_vertex_shader, block_geometry_shader, block_fragment_shader},
            { std_model, std_view, std_proj,
            std_light, std_camera,
            block_delta,
            box_delta },
            { "fragment_color" }
            );
    */

    vector<int> vec;
    for(uint i = 0; i < nodes.size(); i++) {
        vec.push_back(i);
    }

    RenderDataInput box_pass_input;
    box_pass_input.assign(1, "vertex", vec.data(), vec.size(), 1, GL_INT);
    box_pass_input.assignIndex(faces.data(), faces.size(), 3);
    RenderPass box_pass(-1, box_pass_input,
            { box_vertex_shader, box_geometry_shader, box_fragment_shader},
            { std_model, std_view, std_proj,
            std_light, std_camera,
            vertex_positions,
            box_delta },
            { "fragment_color" }
            );

    float aspect = 0.0f;

    while (!glfwWindowShouldClose(window)) {
        // Setup some basic window stuff.
        glfwGetFramebufferSize(window, &window_width, &window_height);
        glViewport(0, 0, window_width, window_height);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_MULTISAMPLE);
        glEnable(GL_BLEND);
        glEnable(GL_CULL_FACE);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDepthFunc(GL_LESS);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glCullFace(GL_BACK);

        gui.updateMatrices();
        mats = gui.getMatrixPointers();

        floor_pass.setup();
        // Draw our triangles.
        CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES,
                    floor_faces.size() * 3,
                    GL_UNSIGNED_INT, 0));

        //ps->calculateNewPositions();

        /*
        vector<glm::vec4> particles2 = ss->getNewPositions();
        for(uint i = 0; i < particles2.size(); ++i) {
            particles[i] = particles2[i];
        }

        block_pass.setup();
        CHECK_GL_ERROR(glDrawElementsInstanced(GL_TRIANGLES,
                    block_faces.size() * 3,
                    GL_UNSIGNED_INT, 0, ps->particles.size()));
        */

        /**/
        box_pass.setup();
        CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES,
                    faces.size() * 3,
                    GL_UNSIGNED_INT, 0));
        /**/

        // Poll and swap.
        glfwPollEvents();
        glfwSwapBuffers(window);
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}
