#ifndef CONFIG_H
#define CONFIG_H

#include <string>

/*
 * Global variables go here.
 */

const float kNear = 0.1f;
const float kFar = 1000.0f;
const float kFov = 45.0f;

// Floor info.
const float kFloorEps = 0.5 * (0.025 + 0.0175);
const float kFloorXMin = -100.0f;
const float kFloorXMax = 100.0f;
const float kFloorZMin = -100.0f;
const float kFloorZMax = 100.0f;
const float kFloorY = -0.75617 - kFloorEps;

// Simulation parameters
const std::string mesh = "box.2";
const float height = 5.0f;
const double floorStiffness = 1.0;
const double CoR = 0.5;
const double timeStep = 0.015;
const double delta = 0.6;
const double a = 10.0;
const double b = 10.0;
const double damping = 0.001;
const double criticalTension = 0.25;
const double criticalCompression = 0.25;
const double gravity = 1.0;
const double pressure = 0.15;

#endif
