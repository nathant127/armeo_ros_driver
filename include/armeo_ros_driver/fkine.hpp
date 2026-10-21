#pragma once

#include <vector>
#include <Eigen/Dense>

// Structure to represent a single row in the DH parameter table
struct DHParam {
    double a;     // Link length
    double d;     // Link offset - Variable for Prismatic joints
    double alpha; // Link twist (radians)
    double theta; // Joint angle (radians) - Variable for Revolute joints
};

constexpr std::vector<DHParam> robot_dh = {
    {-0.0000,  0.0000,  1.5708, -0.0000, false},
    { 0.0000,  0.0000, -1.5708,  3.1416, false},
    {-0.0000,  0.2490,  1.5708, -0.0000, false},
    {-0.0000, -0.0000, -1.5708, -0.0000, false},
    { 0.0000,  0.2520,  1.5708,  0.0000, false},
    { 0.0840, -0.0000,  0.0000,  1.5708, false}
};

using Transform = Eigen::Matrix<double, 4, 4>;

Transform fkine(const Eigen::Vector<double, 6>& joint_states) {
    Transform T = Transform::Identity();
    for (size_t i = 0; i < robot_dh.size(); ++i) {
        T = T * computeLinkTransform(robot_dh[i], joint_states[i]);
    }
    return T;
}

Transform computeLinkTransform(const DHParam& param, double joint_input) {
    // Determine active parameters depending on joint type
    double theta = param.theta + joint_input;
    double d     = param.d;
    double a     = param.a;
    double alpha = param.alpha;

    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);

    // Standard DH formulation matrix
    return {
        {ct, -st * ca,  st * sa, a * ct},
        {st,  ct * ca, -ct * sa, a * st},
        {0.0,    sa,       ca,      d  },
        {0.0,   0.0,      0.0,     1.0 }
    };
}
