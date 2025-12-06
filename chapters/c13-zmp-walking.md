# Chapter 13: ZMP Walking and Balance Control for Humanoid Robotics

## Learning Outcomes
By the end of this chapter, students will be able to:
- Understand the Zero Moment Point (ZMP) stability criterion for humanoid locomotion
- Implement ZMP-based walking pattern generation algorithms
- Design feedback controllers for dynamic balance maintenance
- Apply Capture Point theory for balance recovery strategies
- Evaluate the stability margins of humanoid walking gaits

## Overview
Zero Moment Point (ZMP) based control represents one of the most successful approaches to achieving stable bipedal locomotion in humanoid robots. This chapter explores the theoretical foundations of ZMP stability, practical implementation of walking pattern generation algorithms, and feedback control strategies for maintaining dynamic balance during locomotion. We'll examine the mathematical formulation of ZMP, the Linear Inverted Pendulum Model (LIPM), and advanced techniques for generating stable walking patterns that enable humanoid robots to walk dynamically while maintaining balance.

## Key Concepts
- **Zero Moment Point (ZMP)**: The point on the ground where the sum of all moments due to external forces equals zero, used as a stability criterion
- **Linear Inverted Pendulum Model (LIPM)**: Simplified model of humanoid dynamics for ZMP-based control
- **Capture Point**: Location where a robot must step to come to a complete stop, used for balance recovery
- **Walking Pattern Generation**: Algorithms for creating stable footstep sequences and center of mass trajectories
- **Feedback Control**: Real-time adjustments to maintain balance based on sensor measurements

## 13.1 Theoretical Foundations of ZMP

### 13.1.1 Zero Moment Point Definition

The Zero Moment Point (ZMP) is a fundamental concept in bipedal robotics that defines the point on the ground where the sum of all moments due to external forces acting on the robot equals zero. For a humanoid robot, the ZMP is calculated as:

```
ZMP_x = (Σ(m_i * (ẍ_i * z_i - ẍ_g * z_i)) + Σ(F_ix * z_i - F_iz * x_i)) / Σ(m_i * g + F_iz)
ZMP_y = (Σ(m_i * (ÿ_i * z_i - ÿ_g * z_i)) + Σ(F_iy * z_i - F_iz * y_i)) / Σ(m_i * g + F_iz)
```

Where:
- `m_i` is the mass of the i-th point mass
- `ẍ_i`, `ÿ_i` are the accelerations of the i-th point mass
- `x_i`, `y_i`, `z_i` are the positions of the i-th point mass
- `F_ix`, `F_iy`, `F_iz` are the external forces on the i-th point mass
- `g` is the gravitational acceleration

### 13.1.2 Stability Criterion

For a humanoid robot to maintain stable posture or locomotion, the ZMP must lie within the support polygon defined by the contact points with the ground. The support polygon is typically the convex hull of the feet when both are in contact, or the area of the single foot during single support phase.

The ZMP stability criterion can be expressed as:
```
ZMP_support_polygon = {(x, y) | ZMP_x ∈ [x_min, x_max], ZMP_y ∈ [y_min, y_max]}
```

## 13.2 Linear Inverted Pendulum Model (LIPM)

### 13.2.1 Mathematical Formulation

The Linear Inverted Pendulum Model (LIPM) simplifies the complex dynamics of a humanoid robot to a point mass supported by a massless leg. This model assumes:

1. The center of mass (CoM) height remains constant: `z = h = constant`
2. The angular momentum around the CoM is zero
3. The robot is simplified to a point mass at the CoM

Under these assumptions, the relationship between the CoM position `(x, y)` and the ZMP position `(px, py)` becomes:

```
ẍ = g/h * (x - px)
ÿ = g/h * (y - py)
```

### 13.2.2 C++ Implementation of LIPM

```cpp
// LIPMController.h
#pragma once
#include <Eigen/Dense>
#include <vector>

class LIPMController {
private:
    double com_height_;          // Center of mass height (m)
    double gravity_;             // Gravitational acceleration (m/s²)
    double omega_;               // Natural frequency (sqrt(g/h))

    Eigen::Vector2d com_position_;   // Current CoM position (x, y)
    Eigen::Vector2d com_velocity_;   // Current CoM velocity (vx, vy)
    Eigen::Vector2d zmp_position_;   // Current ZMP position (px, py)

    Eigen::Vector2d desired_com_position_;
    Eigen::Vector2d desired_com_velocity_;
    Eigen::Vector2d desired_zmp_position_;

public:
    LIPMController(double com_height = 0.8);

    // Update state
    void updateState(const Eigen::Vector2d& com_pos,
                    const Eigen::Vector2d& com_vel,
                    const Eigen::Vector2d& zmp_pos);

    // Compute desired ZMP from desired CoM trajectory
    Eigen::Vector2d computeZMPTrajectory(const Eigen::Vector2d& desired_com_pos,
                                        const Eigen::Vector2d& desired_com_vel);

    // Compute CoM trajectory from ZMP reference
    Eigen::Vector2d computeCoMTrajectory(const Eigen::Vector2d& zmp_ref,
                                        double dt);

    // Solve LIPM differential equation
    Eigen::Vector2d solveLIPM(const Eigen::Vector2d& zmp_ref,
                             const Eigen::Vector2d& current_com,
                             const Eigen::Vector2d& current_vel,
                             double dt);

    // Calculate stability margins
    double calculateStabilityMargin(const Eigen::Vector2d& zmp_pos,
                                  const std::vector<Eigen::Vector2d>& support_polygon);
};

// LIPMController.cpp
#include "LIPMController.h"
#include <cmath>
#include <algorithm>

LIPMController::LIPMController(double com_height)
    : com_height_(com_height), gravity_(9.81) {
    omega_ = std::sqrt(gravity_ / com_height_);
    com_position_.setZero();
    com_velocity_.setZero();
    zmp_position_.setZero();
    desired_com_position_.setZero();
    desired_com_velocity_.setZero();
    desired_zmp_position_.setZero();
}

void LIPMController::updateState(const Eigen::Vector2d& com_pos,
                                const Eigen::Vector2d& com_vel,
                                const Eigen::Vector2d& zmp_pos) {
    com_position_ = com_pos;
    com_velocity_ = com_vel;
    zmp_position_ = zmp_pos;
}

Eigen::Vector2d LIPMController::computeZMPTrajectory(
    const Eigen::Vector2d& desired_com_pos,
    const Eigen::Vector2d& desired_com_vel) {

    // From LIPM: px = x - h/g * ẍ and py = y - h/g * ÿ
    // For trajectory generation, we use: px = x - 1/ω² * ẍ
    Eigen::Vector2d zmp_ref;
    zmp_ref = desired_com_pos - (1.0 / (omega_ * omega_)) * desired_com_vel;

    return zmp_ref;
}

Eigen::Vector2d LIPMController::solveLIPM(const Eigen::Vector2d& zmp_ref,
                                         const Eigen::Vector2d& current_com,
                                         const Eigen::Vector2d& current_vel,
                                         double dt) {

    // Solve the LIPM differential equation using analytical solution
    // x(t) = A * exp(ω*t) + B * exp(-ω*t) + px
    // where px, py are ZMP positions

    double exp_pos = std::exp(omega_ * dt);
    double exp_neg = std::exp(-omega_ * dt);

    // Calculate coefficients A and B based on initial conditions
    double A_x = (current_vel[0] + omega_ * (current_com[0] - zmp_ref[0])) / (2.0 * omega_);
    double B_x = (current_vel[0] - omega_ * (current_com[0] - zmp_ref[0])) / (2.0 * omega_);

    double A_y = (current_vel[1] + omega_ * (current_com[1] - zmp_ref[1])) / (2.0 * omega_);
    double B_y = (current_vel[1] - omega_ * (current_com[1] - zmp_ref[1])) / (2.0 * omega_);

    Eigen::Vector2d new_com;
    new_com[0] = A_x * exp_pos + B_x * exp_neg + zmp_ref[0];
    new_com[1] = A_y * exp_pos + B_y * exp_neg + zmp_ref[1];

    return new_com;
}

double LIPMController::calculateStabilityMargin(
    const Eigen::Vector2d& zmp_pos,
    const std::vector<Eigen::Vector2d>& support_polygon) {

    if (support_polygon.size() < 3) {
        return 0.0; // Invalid polygon
    }

    // Find the closest point on the support polygon to the ZMP
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < support_polygon.size(); ++i) {
        size_t next_i = (i + 1) % support_polygon.size();

        // Calculate distance from ZMP to each edge of the polygon
        Eigen::Vector2d edge = support_polygon[next_i] - support_polygon[i];
        Eigen::Vector2d to_zmp = zmp_pos - support_polygon[i];

        double edge_length = edge.norm();
        if (edge_length < 1e-6) continue; // Skip degenerate edge

        Eigen::Vector2d edge_unit = edge / edge_length;
        double projection = to_zmp.dot(edge_unit);

        if (projection < 0) {
            // Closest point is the start vertex
            double dist = (zmp_pos - support_polygon[i]).norm();
            min_distance = std::min(min_distance, dist);
        } else if (projection > edge_length) {
            // Closest point is the end vertex
            double dist = (zmp_pos - support_polygon[next_i]).norm();
            min_distance = std::min(min_distance, dist);
        } else {
            // Closest point is on the edge
            Eigen::Vector2d closest_point = support_polygon[i] + projection * edge_unit;
            double dist = (zmp_pos - closest_point).norm();
            min_distance = std::min(min_distance, dist);
        }
    }

    return min_distance;
}
```

## 13.3 Walking Pattern Generation

### 13.3.1 Footstep Planning

Walking pattern generation involves creating a sequence of footstep positions and timing that enables stable locomotion. The process typically involves:

1. **Footstep placement**: Determining where to place the swing foot
2. **Timing**: Defining the duration of single and double support phases
3. **CoM trajectory**: Planning the center of mass path during walking

### 13.3.2 Preview Control for ZMP Tracking

Preview control uses future reference trajectory information to improve tracking performance. For ZMP control, this means using future ZMP references to compute current CoM positions.

```cpp
// PreviewController.h
#pragma once
#include <Eigen/Dense>
#include <vector>

class PreviewController {
private:
    int preview_horizon_;       // Number of future steps to consider
    double control_dt_;         // Control timestep
    double sampling_dt_;        // Sampling timestep for preview

    std::vector<double> k_matrix_;      // Feedback gains
    std::vector<double> kf_matrix_;     // Preview gains

    Eigen::VectorXd zmp_reference_;     // Future ZMP reference trajectory
    Eigen::VectorXd com_state_;         // Current CoM state [position, velocity]

public:
    PreviewController(int horizon, double dt);

    void setReferenceTrajectory(const std::vector<Eigen::Vector2d>& zmp_refs);
    Eigen::Vector2d computeControl(const Eigen::Vector2d& current_zmp,
                                  const Eigen::Vector2d& current_com,
                                  const Eigen::Vector2d& current_com_vel);

private:
    void computePreviewGains();
    Eigen::Vector2d solveRiccatiEquation();
};
```

## 13.4 Capture Point Theory

### 13.4.1 Capture Point Definition

The Capture Point (CP) is the location on the ground where a robot must step to come to a complete stop. It's defined as:

```
CP_x = CoM_x + CoM_vx / ω
CP_y = CoM_y + CoM_vy / ω
```

Where `ω = sqrt(g/h)` is the natural frequency of the inverted pendulum.

### 13.4.2 Balance Recovery Strategies

Capture Point theory provides a framework for balance recovery by determining where to step to regain stability.

```cpp
// CapturePointController.h
#pragma once
#include <Eigen/Dense>

class CapturePointController {
private:
    double com_height_;
    double gravity_;
    double omega_;

    Eigen::Vector2d com_position_;
    Eigen::Vector2d com_velocity_;

public:
    CapturePointController(double height);

    Eigen::Vector2d calculateCapturePoint();
    bool isStable(const Eigen::Vector2d& support_polygon_center,
                  double support_polygon_width);
    void computeBalanceRecoveryStep(Eigen::Vector2d& step_position,
                                   double& step_timing);
};
```

## 13.5 Feedback Control for Dynamic Balance

### 13.5.1 ZMP Feedback Control

ZMP feedback control adjusts the desired CoM trajectory based on the measured ZMP error to maintain stability.

```cpp
// ZMPFeedbackController.h
#pragma once
#include <Eigen/Dense>

class ZMPFeedbackController {
private:
    double kp_zmp_;        // Proportional gain for ZMP error
    double ki_zmp_;        // Integral gain for ZMP error
    double kd_zmp_;        // Derivative gain for ZMP error

    Eigen::Vector2d zmp_error_integral_;
    Eigen::Vector2d prev_zmp_error_;

    Eigen::Vector2d feedback_offset_;

public:
    ZMPFeedbackController(double kp = 10.0, double ki = 1.0, double kd = 1.0);

    Eigen::Vector2d computeFeedback(const Eigen::Vector2d& measured_zmp,
                                   const Eigen::Vector2d& desired_zmp,
                                   double dt);

    void reset();
};
```

## 13.6 Implementation Considerations

### 13.6.1 Real-time Performance

ZMP-based walking controllers must operate in real-time to maintain stability:

- Control frequency: Typically 200-1000 Hz for humanoid robots
- Computational complexity: LIPM solutions should be computed in < 5ms
- Sensor integration: IMU, force/torque sensors, joint encoders

### 13.6.2 Robustness and Adaptation

Real-world implementations must handle:
- Model uncertainties (mass, inertia, CoM position)
- External disturbances (pushes, uneven terrain)
- Actuator limitations (torque, velocity, position limits)

## 13.7 Integration with ROS 2

### 13.7.1 ROS 2 Walking Controller Interface

```cpp
// zmp_walking_controller.hpp
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "LIPMController.h"

namespace zmp_walking_controller {

class ZMPWalkingController : public controller_interface::ControllerInterface {
public:
    ZMPWalkingController();

    controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    // ZMP controller instance
    std::unique_ptr<LIPMController> lipm_controller_;

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr zmp_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr com_trajectory_publisher_;

    // Walking parameters
    double step_length_;
    double step_width_;
    double step_height_;
    double walking_frequency_;

    // State estimation
    Eigen::Vector2d estimated_com_position_;
    Eigen::Vector2d estimated_com_velocity_;
    Eigen::Vector2d estimated_zmp_position_;

    void initializeWalkingPattern();
    void updateStateEstimation();
    void computeWalkingControl();
};

} // namespace zmp_walking_controller
```

## Summary

ZMP-based walking and balance control provides a mathematically sound approach to achieving stable bipedal locomotion in humanoid robots. The Linear Inverted Pendulum Model simplifies the complex dynamics while maintaining the essential stability characteristics. Walking pattern generation algorithms create stable footstep sequences, while feedback control systems maintain balance in the presence of disturbances.

The success of ZMP-based control depends on accurate state estimation, real-time computation of control commands, and proper integration with the robot's hardware. Capture Point theory provides additional tools for balance recovery, enabling robots to respond to unexpected disturbances.

Modern implementations leverage preview control and optimization techniques to improve tracking performance and stability margins, making ZMP-based walking a cornerstone of humanoid robotics.

## Review Questions

1. Explain the Zero Moment Point (ZMP) stability criterion. Why is it important for humanoid locomotion?

2. Describe the Linear Inverted Pendulum Model (LIPM) assumptions and how they simplify humanoid dynamics.

3. How is the Capture Point used for balance recovery? Calculate the Capture Point for a given CoM position and velocity.

4. What are the key components of a ZMP-based walking pattern generation system? Explain the role of each component.

5. Compare the advantages and limitations of ZMP-based control versus other balance control approaches.

6. Design a simple ZMP feedback controller. Specify the control law and tuning parameters.

7. How does preview control improve ZMP tracking performance? What are the computational requirements?

8. What are the real-time performance requirements for ZMP-based walking controllers? How do these impact system design?