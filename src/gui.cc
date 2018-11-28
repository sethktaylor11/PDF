#include "gui.h"
#include "config.h"
#include <jpegio.h>
#include <iostream>
#include <debuggl.h>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

/*
void CreateCloth(std::vector<glm::vec4>& particle_pos,
        std::vector<glm::uvec2>& connections,
        std::vector<glm::uvec2>& shear,
        std::vector<glm::uvec2>& bending,
        std::vector<bool>& fixed_points,
        std::vector<glm::uvec3>& faces,
        std::vector<glm::vec2>& uv_coordinates)
{
    particle_pos.clear();
    connections.clear();
    fixed_points.clear();
    faces.clear();

    float length = 4.0;
    int subdivisions = 20;
    float inc = length/subdivisions;
    int count = subdivisions+1;

    for (float i = -length/2; i <= length/2+inc/2; i+=inc) {
        for (float j = 2; j <= 2+(length+inc/2); j+=inc) {
            particle_pos.push_back(glm::vec4(i,j,0.0,1.0));
            uv_coordinates.push_back(glm::vec2((i+length/2)/length, (j-2)/length));
            if((i < (-length/2 + inc/2) && j > (2+length-inc/2)) || (i > (length/2 - inc/2) && j > (2+length-inc/2))) {
                fixed_points.push_back(true);
            } else {
                fixed_points.push_back(false);
            }
        }
    }

    for (int i = 0; i < subdivisions; i++) {
        for (int j = 0; j < subdivisions; j++) {
            int index = i*count+j;

            shear.push_back(glm::uvec2(index, index+(count+1)));
            shear.push_back(glm::uvec2(index+1, index+count));

            faces.push_back(glm::uvec3(index,index+count,index+(count+1)));
            faces.push_back(glm::uvec3(index,index+(count+1),index+1));
            faces.push_back(glm::uvec3(index,index+(count+1),index+count));
            faces.push_back(glm::uvec3(index,index+1,index+(count+1)));
        }
    }

    for (int i = 0; i < count; i++) {
        for (int j = 0; j < count; j++) {
            int index = i*count+j;
            if (j+2 < count) {
                bending.push_back(glm::uvec2(index,index+2));
            }
            if (i+2 < count) {
                bending.push_back(glm::uvec2(index,index+2*count));
            }
        }
    }

    for (int i = 0; i < count; i++) {
        for (int j = 0; j < count; j++) {
            int index = i*count+j;
            if (j+1 != count) {
                connections.push_back(glm::uvec2(index,index+1));
            }
            if (i+1 != count) {
                connections.push_back(glm::uvec2(index,index+count));
            }
        }
    }
}
*/

/*
void CreateBlock(std::vector<glm::vec4>& obj_vertices,
        std::vector<glm::uvec3>& obj_faces)
{
    obj_vertices.clear();
    obj_faces.clear();
    for(float i = -0.5; i <= 1; i++) {
        for(float j = -0.5; j <= 1; j++) {
            for(float k = -0.5; k <= 1; k++) {
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
*/

GUI::GUI(GLFWwindow* window, bool col)
    :window_(window)
{
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, KeyCallback);
    glfwSetCursorPosCallback(window_, MousePosCallback);
    glfwSetMouseButtonCallback(window_, MouseButtonCallback);

    glfwGetWindowSize(window_, &window_width_, &window_height_);
    float aspect_ = static_cast<float>(window_width_) / window_height_;
    projection_matrix_ = glm::perspective((float)(kFov * (M_PI / 180.0f)), aspect_, kNear, kFar);
}

GUI::~GUI()
{
}

void GUI::keyCallback(int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, GL_TRUE);
        return ;
    }
    if (captureWASDUPDOWN(key, action))
        return ;
}

void GUI::mousePosCallback(double mouse_x, double mouse_y)
{
    last_x_ = current_x_;
    last_y_ = current_y_;
    current_x_ = mouse_x;
    current_y_ = window_height_ - mouse_y;
    float delta_x = current_x_ - last_x_;
    float delta_y = current_y_ - last_y_;
    if (sqrt(delta_x * delta_x + delta_y * delta_y) < 1e-15)
        return;
    glm::vec3 mouse_direction = glm::normalize(glm::vec3(delta_x, delta_y, 0.0f));

    bool drag_camera = drag_state_ && current_button_ == GLFW_MOUSE_BUTTON_RIGHT;
    if (drag_camera) {
        glm::vec3 axis = glm::normalize(
                orientation_ *
                glm::vec3(mouse_direction.y, -mouse_direction.x, 0.0f)
                );
        orientation_ =
            glm::mat3(glm::rotate(rotation_speed_, axis) * glm::mat4(orientation_));
        tangent_ = glm::column(orientation_, 0);
        up_ = glm::column(orientation_, 1);
        look_ = glm::column(orientation_, 2);
    }
}

void GUI::mouseButtonCallback(int button, int action, int mods)
{
    drag_state_ = (action == GLFW_PRESS);
    current_button_ = button;
}

void GUI::updateMatrices()
{
    // Compute our view, and projection matrices.
    if (fps_mode_)
        center_ = eye_ + camera_distance_ * look_;
    else
        eye_ = center_ - camera_distance_ * look_;

    view_matrix_ = glm::lookAt(eye_, center_, up_);
    light_position_ = glm::vec4(eye_, 1.0f);

    aspect_ = static_cast<float>(window_width_) / window_height_;
    projection_matrix_ =
        glm::perspective((float)(kFov * (M_PI / 180.0f)), aspect_, kNear, kFar);
    model_matrix_ = glm::mat4(1.0f);
}

MatrixPointers GUI::getMatrixPointers() const
{
    MatrixPointers ret;
    ret.projection = &projection_matrix_[0][0];
    ret.model= &model_matrix_[0][0];
    ret.view = &view_matrix_[0][0];
    return ret;
}

bool GUI::captureWASDUPDOWN(int key, int action)
{
    if (key == GLFW_KEY_W) {
        if (fps_mode_)
            eye_ += zoom_speed_ * look_;
        else
            camera_distance_ -= zoom_speed_;
        return true;
    } else if (key == GLFW_KEY_S) {
        if (fps_mode_)
            eye_ -= zoom_speed_ * look_;
        else
            camera_distance_ += zoom_speed_;
        return true;
    } else if (key == GLFW_KEY_A) {
        if (fps_mode_)
            eye_ -= pan_speed_ * tangent_;
        else
            center_ -= pan_speed_ * tangent_;
        return true;
    } else if (key == GLFW_KEY_D) {
        if (fps_mode_)
            eye_ += pan_speed_ * tangent_;
        else
            center_ += pan_speed_ * tangent_;
        return true;
    } else if (key == GLFW_KEY_DOWN) {
        if (fps_mode_)
            eye_ -= pan_speed_ * up_;
        else
            center_ -= pan_speed_ * up_;
        return true;
    } else if (key == GLFW_KEY_UP) {
        if (fps_mode_)
            eye_ += pan_speed_ * up_;
        else
            center_ += pan_speed_ * up_;
        return true;
    }
    return false;
}


// Delegrate to the actual GUI object.
void GUI::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
    gui->keyCallback(key, scancode, action, mods);
}

void GUI::MousePosCallback(GLFWwindow* window, double mouse_x, double mouse_y)
{
    GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
    gui->mousePosCallback(mouse_x, mouse_y);
}

void GUI::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
    gui->mouseButtonCallback(button, action, mods);
}
