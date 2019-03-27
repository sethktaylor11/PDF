#include <GL/glew.h>
#include <dirent.h>

#include "procedure_geometry.h"
#include "render_pass.h"
#include "config.h"
#include "gui.h"
#include "peridynamic_system.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <string.h>

#define GLM_ENABLE_EXPERIMENTAL
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

void readNodes(vector<glm::vec4>& nodes, vector<bool>& fixedNodes) {
    ifstream nodesFile;
    nodesFile.open("box.2.node");
    int nP, d, nAN, nBN;
    nodesFile >> nP >> d >> nAN >> nBN;
    nodes.resize(nP);
    fixedNodes.resize(nP);
    int p;
    float x, y, z;
    for (int i = 0; i < nP; i++) {
        nodesFile >> p >> x >> y >> z;
        nodes[p] = glm::vec4(x,y,z,1);
        if (x == -4.0f) fixedNodes[p] = true;
    }
}

void readFaces(vector<int>& boundary, vector<vector<int>>& triangles, vector<vector<int>>& neighbors) {
    ifstream facesFile;
    facesFile.open("box.2.face");
    int nF, nBF;
    facesFile >> nF >> nBF;
    boundary.resize(nF);
    triangles.resize(nF, vector<int>(3));
    neighbors.resize(nF, vector<int>(2));
    for (int i = 0; i < nF; i++) {
        int f, A, B, C, b, n1, n2;
        facesFile >> f >> A >> B >> C >> b >> n1 >> n2;
        boundary[f] = b;
	triangles[f][0] = A;
	triangles[f][1] = B;
	triangles[f][2] = C;
        neighbors[f][0] = n1;
	neighbors[f][1] = n2;
    }
}

void readTets(vector<vector<int>>& tets) {
    ifstream tetsFile;
    tetsFile.open("box.2.ele");
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

void readNeighbors(vector<vector<int>>& neighbors) {
    ifstream neighborsFile;
    neighborsFile.open("box.2.neigh");
    int nT, nN;
    neighborsFile >> nT >> nN;
    neighbors.resize(nT);
    for (int i = 0; i < nT; i++) {
        int t, A, B, C, D;
        neighborsFile >> t >> A >> B >> C >> D;
        vector<int> tets(4);
	tets[0] = A;
	tets[1] = B;
	tets[2] = C;
	tets[3] = D;
        neighbors[t] = tets;
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

    vector<int> boundary;
    vector<vector<int>> triangles;
    vector<vector<int>> faceNeighbors;
    readFaces(boundary, triangles, faceNeighbors);

    // Read Tets

    vector<vector<int>> tets;
    readTets(tets);

    // Read Neighbors
    
    vector<vector<int>> neighbors;
    readNeighbors(neighbors);

    PeridynamicSystem* ps = new PeridynamicSystem(nodes,fixedNodes,tets,boundary,triangles,faceNeighbors,neighbors);

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
    auto box_delta_data = []() -> const void* {
        static const float box_delta = 10.0f;
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

    RenderDataInput box_pass_input;
    box_pass_input.assign(0, "vertex_position", nodes.data(), nodes.size(), 4, GL_FLOAT);
    box_pass_input.assignIndex(ps->faces.data(), ps->faces.size(), 3);
    RenderPass box_pass(-1, box_pass_input,
            { box_vertex_shader, box_geometry_shader, box_fragment_shader},
            { std_model, std_view, std_proj,
            std_light, std_camera,
            box_delta },
            { "fragment_color" }
            );

    int i = 0;
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

        ps->calculateNewPositions();

	box_pass.updateVBO(0, ps->nodes.data(), ps->nodes.size());
	box_pass.updateIndex(ps->faces.data(), ps->faces.size());

        box_pass.setup();
        CHECK_GL_ERROR(glDrawElements(GL_TRIANGLES,
                    ps->faces.size() * 3,
                    GL_UNSIGNED_INT, 0));

        // Poll and swap.
        glfwPollEvents();
        glfwSwapBuffers(window);
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}
