---
id: c11-sensor-fusion
title: "Chapter 11: Sensor Fusion for Humanoid Robotics"
sidebar_label: "C11: Sensor Fusion"
sidebar_position: 11
---

# Chapter 11: Sensor Fusion for Humanoid Robotics

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Implement** Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF) for humanoid state estimation
2. **Design** multi-sensor fusion architectures that integrate IMU, vision, and proprioceptive sensors
3. **Deploy** visual-inertial odometry (VIO) systems for humanoid localization and mapping
4. **Evaluate** sensor fusion performance in terms of accuracy, latency, and robustness for real-time applications

## Overview

Chapters 9 and 10 established real-time control systems and algorithms for humanoid robotics. However, effective control requires accurate state estimation from multiple sensors. **Sensor fusion** combines data from various sensors (IMUs, cameras, encoders, force/torque sensors) to provide robust, accurate state estimates for humanoid control systems.

Humanoid robots operate in complex, dynamic environments where individual sensors have limitations: IMUs drift over time, cameras fail in poor lighting, and encoders don't provide absolute position. Sensor fusion algorithms address these limitations by combining complementary sensor information, providing robust state estimates for balance control, navigation, and manipulation tasks.

**Visual-Inertial Odometry (VIO)** represents a key fusion approach, combining camera visual features with IMU inertial measurements for accurate pose estimation. **Kalman filtering** provides the mathematical foundation for optimal state estimation under uncertainty. **Multi-sensor architectures** integrate diverse sensor types into unified estimation frameworks.

This chapter explores sensor fusion techniques for humanoid robotics, focusing on real-time implementation, computational efficiency, and robust performance. You will learn to implement EKF/UKF estimators, design VIO systems, and optimize fusion algorithms for embedded deployment.

## Key Concepts

- **Sensor Fusion**: Process of combining data from multiple sensors to achieve more accurate and reliable state estimates than individual sensors can provide
- **Extended Kalman Filter (EKF)**: Nonlinear Kalman filter variant that linearizes system models around current state estimates
- **Unscented Kalman Filter (UKF)**: Nonlinear filter that uses deterministic sampling to capture state distribution more accurately than EKF
- **Visual-Inertial Odometry (VIO)**: Estimation technique combining visual features and inertial measurements for pose tracking
- **State Estimation**: Process of determining robot state (position, orientation, velocity, etc.) from sensor measurements
- **Multi-rate Fusion**: Technique for fusing sensors with different update rates (e.g., IMU at 1000Hz, cameras at 30Hz)
- **Observability**: Property of sensor systems indicating whether state variables can be uniquely determined from sensor measurements
- **Covariance**: Matrix representing uncertainty in state estimates, essential for optimal fusion

## Kalman Filtering for Humanoid State Estimation

Kalman filters provide optimal state estimation for linear systems with Gaussian noise, while EKF and UKF extend this to nonlinear systems:

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  Sensor Data    │───►│  Kalman Filter   │───►│  Estimated State │
│  (IMU, Vision,  │    │  (Prediction &   │    │  (Position,      │
│   Encoders)     │    │   Update)        │    │   Orientation,   │
│                 │    │                  │    │   Velocity)      │
└─────────────────┘    └──────────────────┘    └──────────────────┘
         │                       │                        │
         ▼                       ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  Multi-rate     │    │  Covariance      │    │  < 10ms          │
│  Data Input     │    │  Management      │    │  Update Time     │
│  (1000Hz IMU,   │    │  (Uncertainty    │    │                  │
│   100Hz Vision) │    │   Propagation)   │    │                  │
└─────────────────┘    └──────────────────┘    └──────────────────┘
```

Kalman filters optimally combine sensor measurements with motion models to minimize estimation uncertainty, making them ideal for humanoid state estimation where multiple sensors provide complementary information.

### Extended Kalman Filter Implementation

```cpp
// ekf_state_estimator.cpp - Extended Kalman Filter for humanoid state estimation
#include <Eigen/Dense>
#include <vector>
#include <chrono>

namespace humanoid_sensors {

class EKFStateEstimator {
private:
    // State vector: [px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz]
    // Position (3), Orientation quaternion (4), Linear velocity (3), Angular velocity (3)
    static constexpr int STATE_DIM = 13;
    static constexpr int IMU_MEAS_DIM = 6;  // Accelerometer (3) + Gyroscope (3)
    static constexpr int POS_MEAS_DIM = 3;  // Position measurement (3)

    // State vector
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;

    // Process noise and measurement noise
    Eigen::MatrixXd process_noise_;
    Eigen::MatrixXd imu_noise_;
    Eigen::MatrixXd pos_noise_;

    // Time tracking
    double last_update_time_;
    bool initialized_;

    // Gravity vector in world frame
    Eigen::Vector3d gravity_world_;

public:
    EKFStateEstimator() : initialized_(false) {
        state_ = Eigen::VectorXd::Zero(STATE_DIM);
        covariance_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.1;

        // Initialize with identity quaternion for orientation
        state_(6) = 1.0;  // qw component

        // Process noise matrix (tuned for humanoid dynamics)
        process_noise_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
        process_noise_.block<3,3>(0,0) *= 0.1;    // Position process noise
        process_noise_.block<4,4>(3,3) *= 0.01;   // Orientation process noise
        process_noise_.block<3,3>(7,7) *= 0.5;    // Velocity process noise
        process_noise_.block<3,3>(10,10) *= 0.2;  // Angular velocity process noise

        // Measurement noise matrices
        imu_noise_ = Eigen::MatrixXd::Identity(IMU_MEAS_DIM, IMU_MEAS_DIM) * 0.01;
        pos_noise_ = Eigen::MatrixXd::Identity(POS_MEAS_DIM, POS_MEAS_DIM) * 0.05;

        gravity_world_ << 0.0, 0.0, -9.81;
        last_update_time_ = 0.0;
    }

    struct IMUData {
        Eigen::Vector3d accelerometer;
        Eigen::Vector3d gyroscope;
        double timestamp;
    };

    struct PositionData {
        Eigen::Vector3d position;
        double timestamp;
    };

    void initialize_state(const Eigen::Vector3d& initial_pos,
                         const Eigen::Quaterniond& initial_orientation,
                         double timestamp) {
        state_.segment<3>(0) = initial_pos;  // Position
        state_.segment<4>(3) << initial_orientation.x(),
                                initial_orientation.y(),
                                initial_orientation.z(),
                                initial_orientation.w();  // Orientation quaternion
        state_.segment<3>(7) = Eigen::Vector3d::Zero();  // Velocity
        state_.segment<3>(10) = Eigen::Vector3d::Zero(); // Angular velocity

        last_update_time_ = timestamp;
        initialized_ = true;
    }

    void predict(const IMUData& imu_data) {
        if (!initialized_) return;

        double dt = imu_data.timestamp - last_update_time_;
        if (dt <= 0) return;  // Invalid time

        // Extract state components
        Eigen::Vector3d pos = state_.segment<3>(0);
        Eigen::Quaterniond orientation(state_(6), state_(3), state_(4), state_(5)); // w, x, y, z
        Eigen::Vector3d vel = state_.segment<3>(7);
        Eigen::Vector3d ang_vel = state_.segment<3>(10);

        // Normalize quaternion to prevent drift
        orientation.normalize();

        // Prediction step: integrate motion model
        // 1. Update orientation based on angular velocity
        Eigen::Quaterniond orientation_rate =
            compute_quaternion_derivative(orientation, imu_data.gyroscope);
        Eigen::Vector4d orientation_dot = Eigen::Vector4d(
            orientation_rate.w(),
            orientation_rate.x(),
            orientation_rate.y(),
            orientation_rate.z()
        );
        Eigen::Vector4d new_orientation = state_.segment<4>(3) + dt * orientation_dot.segment<4>(0);

        // Normalize quaternion
        Eigen::Quaterniond new_quat(new_orientation(0), new_orientation(1),
                                   new_orientation(2), new_orientation(3));
        new_quat.normalize();
        state_.segment<4>(3) << new_quat.x(), new_quat.y(), new_quat.z(), new_quat.w();

        // 2. Compute acceleration in world frame
        Eigen::Matrix3d rot_matrix = new_quat.toRotationMatrix();
        Eigen::Vector3d acceleration_world = rot_matrix * imu_data.accelerometer + gravity_world_;

        // 3. Update velocity and position
        Eigen::Vector3d new_vel = vel + dt * acceleration_world;
        Eigen::Vector3d new_pos = pos + dt * new_vel;

        // Update state vector
        state_.segment<3>(0) = new_pos;    // Position
        state_.segment<3>(7) = new_vel;    // Velocity
        state_.segment<3>(10) = imu_data.gyroscope;  // Angular velocity (measured)

        // Update covariance matrix (simplified - in practice, use Jacobians)
        Eigen::MatrixXd F = compute_state_transition_jacobian(imu_data, dt);
        covariance_ = F * covariance_ * F.transpose() + process_noise_ * dt;

        last_update_time_ = imu_data.timestamp;
    }

    void update_position(const PositionData& pos_data) {
        if (!initialized_) return;

        // Measurement model: extract position from state
        Eigen::VectorXd h = Eigen::VectorXd::Zero(POS_MEAS_DIM);
        h = state_.segment<3>(0);  // Position is first 3 elements

        // Measurement Jacobian (identity for position measurement)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(POS_MEAS_DIM, STATE_DIM);
        H.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();

        // Innovation and Kalman gain calculation
        Eigen::VectorXd innovation = pos_data.position - h;
        Eigen::MatrixXd S = H * covariance_ * H.transpose() + pos_noise_;
        Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

        // Update state and covariance
        state_ = state_ + K * innovation;
        covariance_ = (Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H) * covariance_;

        // Re-normalize quaternion after update
        Eigen::Quaterniond quat(state_(6), state_(3), state_(4), state_(5));
        quat.normalize();
        state_.segment<4>(3) << quat.x(), quat.y(), quat.z(), quat.w();
    }

    void update_imu(const IMUData& imu_data) {
        // For IMU data, we typically use it in the prediction step
        // This method could handle outlier detection or additional processing
        predict(imu_data);
    }

    // Real-time safe accessors
    Eigen::Vector3d get_position() const {
        return state_.segment<3>(0);
    }

    Eigen::Quaterniond get_orientation() const {
        Eigen::Quaterniond quat(state_(6), state_(3), state_(4), state_(5));
        return quat;
    }

    Eigen::Vector3d get_velocity() const {
        return state_.segment<3>(7);
    }

    Eigen::Vector3d get_angular_velocity() const {
        return state_.segment<3>(10);
    }

    Eigen::MatrixXd get_covariance() const {
        return covariance_;
    }

private:
    Eigen::Quaterniond compute_quaternion_derivative(
        const Eigen::Quaterniond& q,
        const Eigen::Vector3d& omega) const {

        // Quaternion derivative: q_dot = 0.5 * Omega * q
        // where Omega is the skew-symmetric matrix of angular velocity
        Eigen::Quaterniond omega_quat(0.0, omega.x(), omega.y(), omega.z());
        return 0.5 * (q * omega_quat);
    }

    Eigen::MatrixXd compute_state_transition_jacobian(const IMUData& imu_data, double dt) const {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

        // Linearize the motion model around current state
        // This is a simplified version - full implementation would include
        // proper Jacobian computation for the nonlinear motion model

        // Position-velocity relationship
        F.block<3,3>(0, 7) = dt * Eigen::Matrix3d::Identity();

        // Velocity-acceleration relationship (approximate)
        Eigen::Quaterniond current_quat(state_(6), state_(3), state_(4), state_(5));
        Eigen::Matrix3d rot_matrix = current_quat.toRotationMatrix();

        // The Jacobian of rotation matrix with respect to orientation is complex
        // This is a simplified approximation
        F.block<3,3>(7, 3) = dt * compute_rotation_jacobian(imu_data.accelerometer);

        return F;
    }

    Eigen::Matrix3d compute_rotation_jacobian(const Eigen::Vector3d& acc) const {
        // Simplified Jacobian computation
        // In practice, this would involve the full derivative of the rotation matrix
        return Eigen::Matrix3d::Zero();
    }
};

} // namespace humanoid_sensors
```

### Unscented Kalman Filter for Nonlinear State Estimation

```cpp
// ukf_state_estimator.cpp - Unscented Kalman Filter for humanoid state estimation
#include <Eigen/Dense>
#include <vector>

namespace humanoid_sensors {

class UKFStateEstimator {
private:
    static constexpr int STATE_DIM = 13;
    static constexpr int MEAS_DIM = 6;  // IMU measurement dimension

    // State vector and covariance
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;

    // Sigma points
    std::vector<Eigen::VectorXd> sigma_points_;
    std::vector<double> weights_mean_;
    std::vector<double> weights_covariance_;

    // UKF parameters
    double alpha_;
    double beta_;
    double kappa_;
    int lambda_;
    int num_sigma_points_;

    // Process and measurement noise
    Eigen::MatrixXd process_noise_;
    Eigen::MatrixXd measurement_noise_;

    // Time tracking
    double last_update_time_;
    bool initialized_;

    // Gravity vector
    Eigen::Vector3d gravity_world_;

public:
    UKFStateEstimator() : alpha_(0.001), beta_(2.0), kappa_(0.0), initialized_(false) {
        // Calculate UKF parameters
        lambda_ = alpha_ * alpha_ * (STATE_DIM + kappa_) - STATE_DIM;
        num_sigma_points_ = 2 * STATE_DIM + 1;

        // Initialize state and covariance
        state_ = Eigen::VectorXd::Zero(STATE_DIM);
        state_(6) = 1.0;  // Initialize quaternion w component
        covariance_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.1;

        // Initialize sigma points
        sigma_points_.resize(num_sigma_points_);
        for (auto& point : sigma_points_) {
            point = Eigen::VectorXd::Zero(STATE_DIM);
        }

        // Initialize weights
        weights_mean_.resize(num_sigma_points_);
        weights_covariance_.resize(num_sigma_points_);

        double weight_0 = lambda_ / (STATE_DIM + lambda_);
        weights_mean_[0] = weight_0;
        weights_covariance_[0] = weight_0 + (1 - alpha_ * alpha_ + beta_);

        for (int i = 1; i < num_sigma_points_; ++i) {
            weights_mean_[i] = 1.0 / (2 * (STATE_DIM + lambda_));
            weights_covariance_[i] = weights_mean_[i];
        }

        // Initialize noise matrices
        process_noise_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.01;
        measurement_noise_ = Eigen::MatrixXd::Identity(MEAS_DIM, MEAS_DIM) * 0.01;

        gravity_world_ << 0.0, 0.0, -9.81;
        last_update_time_ = 0.0;
    }

    void initialize_state(const Eigen::Vector3d& initial_pos,
                         const Eigen::Quaterniond& initial_orientation,
                         double timestamp) {
        state_.segment<3>(0) = initial_pos;  // Position
        state_.segment<4>(3) << initial_orientation.x(),
                                initial_orientation.y(),
                                initial_orientation.z(),
                                initial_orientation.w();  // Orientation quaternion
        state_.segment<3>(7) = Eigen::Vector3d::Zero();  // Velocity
        state_.segment<3>(10) = Eigen::Vector3d::Zero(); // Angular velocity

        last_update_time_ = timestamp;
        initialized_ = true;

        // Generate initial sigma points
        generate_sigma_points();
    }

    void predict(double dt) {
        if (!initialized_ || dt <= 0) return;

        // Generate sigma points based on current state and covariance
        generate_sigma_points();

        // Propagate each sigma point through the motion model
        std::vector<Eigen::VectorXd> propagated_points(num_sigma_points_);
        for (int i = 0; i < num_sigma_points_; ++i) {
            propagated_points[i] = propagate_state(sigma_points_[i], dt);
        }

        // Calculate predicted state and covariance
        state_ = Eigen::VectorXd::Zero(STATE_DIM);
        for (int i = 0; i < num_sigma_points_; ++i) {
            state_ += weights_mean_[i] * propagated_points[i];
        }

        // Re-normalize quaternion
        Eigen::Quaterniond quat(state_(6), state_(3), state_(4), state_(5));
        quat.normalize();
        state_.segment<4>(3) << quat.x(), quat.y(), quat.z(), quat.w();

        // Calculate predicted covariance
        covariance_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
        for (int i = 0; i < num_sigma_points_; ++i) {
            Eigen::VectorXd diff = propagated_points[i] - state_;
            covariance_ += weights_covariance_[i] * diff * diff.transpose();
        }
        covariance_ += process_noise_ * dt;
    }

    void update(const Eigen::VectorXd& measurement) {
        if (!initialized_) return;

        // Generate sigma points based on predicted state
        generate_sigma_points();

        // Propagate sigma points through measurement model
        std::vector<Eigen::VectorXd> measurement_points(num_sigma_points_);
        Eigen::VectorXd predicted_measurement = Eigen::VectorXd::Zero(MEAS_DIM);

        for (int i = 0; i < num_sigma_points_; ++i) {
            measurement_points[i] = measurement_model(sigma_points_[i]);
            predicted_measurement += weights_mean_[i] * measurement_points[i];
        }

        // Calculate innovation covariance
        Eigen::MatrixXd innovation_cov = Eigen::MatrixXd::Zero(MEAS_DIM, MEAS_DIM);
        for (int i = 0; i < num_sigma_points_; ++i) {
            Eigen::VectorXd diff = measurement_points[i] - predicted_measurement;
            innovation_cov += weights_covariance_[i] * diff * diff.transpose();
        }
        innovation_cov += measurement_noise_;

        // Calculate cross-covariance
        Eigen::MatrixXd cross_cov = Eigen::MatrixXd::Zero(STATE_DIM, MEAS_DIM);
        for (int i = 0; i < num_sigma_points_; ++i) {
            Eigen::VectorXd state_diff = sigma_points_[i] - state_;
            Eigen::VectorXd meas_diff = measurement_points[i] - predicted_measurement;
            cross_cov += weights_covariance_[i] * state_diff * meas_diff.transpose();
        }

        // Calculate Kalman gain
        Eigen::MatrixXd kalman_gain = cross_cov * innovation_cov.inverse();

        // Update state and covariance
        Eigen::VectorXd innovation = measurement - predicted_measurement;
        state_ = state_ + kalman_gain * innovation;

        // Re-normalize quaternion after update
        Eigen::Quaterniond quat(state_(6), state_(3), state_(4), state_(5));
        quat.normalize();
        state_.segment<4>(3) << quat.x(), quat.y(), quat.z(), quat.w();

        covariance_ = covariance_ - kalman_gain * innovation_cov * kalman_gain.transpose();
    }

    // Real-time safe accessors
    Eigen::Vector3d get_position() const {
        return state_.segment<3>(0);
    }

    Eigen::Quaterniond get_orientation() const {
        Eigen::Quaterniond quat(state_(6), state_(3), state_(4), state_(5));
        return quat;
    }

    Eigen::Vector3d get_velocity() const {
        return state_.segment<3>(7);
    }

    Eigen::Vector3d get_angular_velocity() const {
        return state_.segment<3>(10);
    }

    Eigen::MatrixXd get_covariance() const {
        return covariance_;
    }

private:
    void generate_sigma_points() {
        // Calculate square root of covariance matrix
        Eigen::MatrixXd sqrt_cov = covariance_.llt().matrixL();

        // First sigma point is the mean
        sigma_points_[0] = state_;

        // Generate remaining sigma points
        for (int i = 0; i < STATE_DIM; ++i) {
            Eigen::VectorXd offset = std::sqrt(STATE_DIM + lambda_) * sqrt_cov.col(i);

            sigma_points_[i + 1] = state_ + offset;
            sigma_points_[i + 1 + STATE_DIM] = state_ - offset;

            // Normalize quaternion for each sigma point
            Eigen::Quaterniond quat(sigma_points_[i + 1](6),
                                   sigma_points_[i + 1](3),
                                   sigma_points_[i + 1](4),
                                   sigma_points_[i + 1](5));
            quat.normalize();
            sigma_points_[i + 1].segment<4>(3) << quat.x(), quat.y(), quat.z(), quat.w();

            quat = Eigen::Quaterniond(sigma_points_[i + 1 + STATE_DIM](6),
                                     sigma_points_[i + 1 + STATE_DIM](3),
                                     sigma_points_[i + 1 + STATE_DIM](4),
                                     sigma_points_[i + 1 + STATE_DIM](5));
            quat.normalize();
            sigma_points_[i + 1 + STATE_DIM].segment<4>(3) << quat.x(), quat.y(), quat.z(), quat.w();
        }
    }

    Eigen::VectorXd propagate_state(const Eigen::VectorXd& state_in, double dt) const {
        Eigen::VectorXd state_out = state_in;

        // Extract components
        Eigen::Vector3d pos = state_in.segment<3>(0);
        Eigen::Quaterniond orientation(state_in(6), state_in(3), state_in(4), state_in(5));
        Eigen::Vector3d vel = state_in.segment<3>(7);
        Eigen::Vector3d ang_vel = state_in.segment<3>(10);

        // Normalize quaternion
        orientation.normalize();

        // Compute rotation matrix
        Eigen::Matrix3d rot_matrix = orientation.toRotationMatrix();

        // Compute acceleration in world frame (simplified - assumes IMU measurement is available)
        Eigen::Vector3d acceleration_world = rot_matrix * Eigen::Vector3d(0, 0, 0) + gravity_world_;

        // Integrate: position, velocity, orientation
        Eigen::Vector3d new_pos = pos + dt * vel;
        Eigen::Vector3d new_vel = vel + dt * acceleration_world;

        // Update orientation using angular velocity
        Eigen::Quaterniond orientation_rate = compute_quaternion_derivative(orientation, ang_vel);
        Eigen::Quaterniond new_orientation = orientation + dt * orientation_rate;
        new_orientation.normalize();

        // Update state
        state_out.segment<3>(0) = new_pos;
        state_out.segment<3>(7) = new_vel;
        state_out.segment<4>(3) << new_orientation.x(), new_orientation.y(),
                                  new_orientation.z(), new_orientation.w();

        return state_out;
    }

    Eigen::VectorXd measurement_model(const Eigen::VectorXd& state_in) const {
        // Simplified measurement model - returns IMU-like measurements
        // In practice, this would depend on the specific sensor configuration
        Eigen::VectorXd measurement = Eigen::VectorXd::Zero(MEAS_DIM);

        // Return accelerometer and gyroscope measurements
        // (In real implementation, this would use the actual sensor model)
        measurement.segment<3>(0) = Eigen::Vector3d(0, 0, 0);  // Accelerometer
        measurement.segment<3>(3) = state_in.segment<3>(10);   // Gyroscope (angular velocity)

        return measurement;
    }

    Eigen::Quaterniond compute_quaternion_derivative(
        const Eigen::Quaterniond& q,
        const Eigen::Vector3d& omega) const {

        Eigen::Quaterniond omega_quat(0.0, omega.x(), omega.y(), omega.z());
        return 0.5 * (q * omega_quat);
    }
};

} // namespace humanoid_sensors
```

## Visual-Inertial Odometry (VIO) for Humanoid Localization

Visual-Inertial Odometry combines camera visual features with IMU measurements for robust pose estimation:

### VIO Feature Tracking and State Estimation

```cpp
// vio_estimator.cpp - Visual-Inertial Odometry for humanoid localization
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <deque>

namespace humanoid_sensors {

class VIOEstimator {
private:
    // State: [position, velocity, orientation, bias_gyro, bias_accel]
    static constexpr int VIO_STATE_DIM = 15;

    // VIO state vector: [p, v, q, bg, ba]
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;

    // Feature tracking
    std::vector<cv::Point2f> current_features_;
    std::vector<cv::Point2f> previous_features_;
    std::vector<bool> feature_tracked_;  // Whether feature is successfully tracked
    std::vector<double> feature_depth_;  // Estimated depth for each feature
    std::vector<int> feature_ids_;       // Unique IDs for tracked features

    // IMU integration
    Eigen::Vector3d integrated_velocity_;
    Eigen::Vector3d integrated_position_;
    Eigen::Quaterniond integrated_orientation_;
    double integration_time_;

    // VIO parameters
    double focal_length_;
    cv::Point2d principal_point_;
    double reprojection_threshold_;
    int min_feature_count_;
    int max_feature_count_;

    // Time tracking
    double last_image_time_;
    double last_imu_time_;
    bool initialized_;

    // Gravity vector
    Eigen::Vector3d gravity_world_;

public:
    VIOEstimator(double fx, double fy, double cx, double cy)
        : focal_length_((fx + fy) / 2.0),
          principal_point_(cx, cy),
          reprojection_threshold_(2.0),
          min_feature_count_(50),
          max_feature_count_(300),
          integration_time_(0.0),
          initialized_(false) {

        state_ = Eigen::VectorXd::Zero(VIO_STATE_DIM);
        state_.segment<4>(6) << 0.0, 0.0, 0.0, 1.0;  // Identity quaternion
        covariance_ = Eigen::MatrixXd::Identity(VIO_STATE_DIM, VIO_STATE_DIM) * 0.1;

        gravity_world_ << 0.0, 0.0, -9.81;
        last_image_time_ = 0.0;
        last_imu_time_ = 0.0;
    }

    struct IMUData {
        Eigen::Vector3d accelerometer;
        Eigen::Vector3d gyroscope;
        double timestamp;
    };

    struct ImageData {
        cv::Mat image;
        double timestamp;
    };

    void initialize_vio(const ImageData& image_data,
                       const std::vector<IMUData>& imu_buffer) {
        // Initialize with first image and IMU data
        detect_features(image_data.image);

        if (!imu_buffer.empty()) {
            // Initialize orientation from accelerometer
            Eigen::Vector3d avg_accel = Eigen::Vector3d::Zero();
            for (const auto& imu : imu_buffer) {
                avg_accel += imu.accelerometer;
            }
            avg_accel /= imu_buffer.size();

            // Compute initial orientation from gravity vector
            Eigen::Vector3d gravity_direction = avg_accel.normalized();
            Eigen::Vector3d z_world(0, 0, 1);

            // Simple initialization - in practice, use more sophisticated methods
            Eigen::Quaterniond init_orientation = Eigen::Quaterniond::FromTwoVectors(
                gravity_direction, -z_world);

            state_.segment<4>(6) << init_orientation.x(), init_orientation.y(),
                                  init_orientation.z(), init_orientation.w();
        }

        last_image_time_ = image_data.timestamp;
        initialized_ = true;
    }

    void process_image(const ImageData& image_data) {
        if (!initialized_) {
            initialize_vio(image_data, {});
            return;
        }

        // Track features from previous frame
        track_features(image_data.image);

        // Update state if enough features are available
        if (current_features_.size() >= min_feature_count_) {
            update_from_features(image_data.timestamp);
        }

        // Add new features if needed
        if (current_features_.size() < max_feature_count_) {
            add_new_features(image_data.image);
        }

        last_image_time_ = image_data.timestamp;
    }

    void process_imu(const IMUData& imu_data) {
        if (!initialized_) return;

        double dt = imu_data.timestamp - last_imu_time_;
        if (dt <= 0) return;

        // Remove bias from measurements
        Eigen::Vector3d gyro_corrected = imu_data.gyroscope - state_.segment<3>(12);  // bias_gyro
        Eigen::Vector3d accel_corrected = imu_data.accelerometer - state_.segment<3>(12 + 3);  // bias_accel

        // Integrate IMU measurements
        integrate_imu(accel_corrected, gyro_corrected, dt);

        last_imu_time_ = imu_data.timestamp;
    }

    void update_from_features(double current_time) {
        // Use feature correspondences to update pose estimate
        // This is a simplified implementation - full VIO would use more sophisticated methods

        if (previous_features_.size() != current_features_.size()) return;

        // Calculate motion estimate from feature correspondences
        Eigen::Vector3d translation;
        Eigen::Quaterniond rotation;

        if (estimate_motion_from_features(translation, rotation)) {
            // Update state with motion estimate
            update_state_with_motion(translation, rotation);
        }
    }

    void integrate_imu(const Eigen::Vector3d& corrected_accel,
                      const Eigen::Vector3d& corrected_gyro,
                      double dt) {
        // Update orientation
        Eigen::Vector4d quat_vec(state_.segment<4>(6));
        Eigen::Quaterniond quat(quat_vec(3), quat_vec(0), quat_vec(1), quat_vec(2));

        // Integrate quaternion derivative
        Eigen::Quaterniond omega_quat(0.0, corrected_gyro.x(),
                                     corrected_gyro.y(), corrected_gyro.z());
        Eigen::Quaterniond quat_dot = 0.5 * (quat * omega_quat);

        Eigen::Vector4d new_quat_vec = quat_vec + dt * Eigen::Vector4d(
            quat_dot.x(), quat_dot.y(), quat_dot.z(), quat_dot.w());

        Eigen::Quaterniond new_quat(new_quat_vec(3), new_quat_vec(0),
                                   new_quat_vec(1), new_quat_vec(2));
        new_quat.normalize();

        // Update velocity and position
        Eigen::Matrix3d rot_matrix = new_quat.toRotationMatrix();
        Eigen::Vector3d gravity_in_body = rot_matrix.transpose() * gravity_world_;
        Eigen::Vector3d acceleration = corrected_accel + gravity_in_body;

        Eigen::Vector3d new_vel = state_.segment<3>(3) + dt * acceleration;
        Eigen::Vector3d new_pos = state_.segment<3>(0) + dt * state_.segment<3>(3) +
                                 0.5 * dt * dt * acceleration;

        // Update state vector
        state_.segment<3>(0) = new_pos;  // Position
        state_.segment<3>(3) = new_vel;  // Velocity
        state_.segment<4>(6) << new_quat.x(), new_quat.y(),
                                new_quat.z(), new_quat.w();  // Orientation

        // Update bias states (simplified random walk model)
        // In practice, bias estimation is more complex
    }

    // Real-time safe accessors
    Eigen::Vector3d get_position() const {
        return state_.segment<3>(0);
    }

    Eigen::Quaterniond get_orientation() const {
        Eigen::Quaterniond quat(state_(6 + 3), state_(6), state_(6 + 1), state_(6 + 2));
        return quat;
    }

    Eigen::Vector3d get_velocity() const {
        return state_.segment<3>(3);
    }

    Eigen::Vector3d get_gyro_bias() const {
        return state_.segment<3>(12);
    }

    Eigen::Vector3d get_accel_bias() const {
        return state_.segment<3>(12 + 3);
    }

private:
    void detect_features(const cv::Mat& image) {
        // Use Shi-Tomasi corner detection to find good features to track
        std::vector<cv::Point2f> corners;

        cv::goodFeaturesToTrack(image, corners, max_feature_count_, 0.01, 10);

        current_features_ = corners;
        feature_tracked_.assign(corners.size(), true);
        feature_depth_.assign(corners.size(), -1.0);  // Unknown depth initially

        // Assign unique IDs to features
        feature_ids_.resize(corners.size());
        static int next_feature_id = 0;
        for (size_t i = 0; i < feature_ids_.size(); ++i) {
            feature_ids_[i] = next_feature_id++;
        }
    }

    void track_features(const cv::Mat& current_image) {
        if (previous_features_.empty()) {
            previous_features_ = current_features_;
            return;
        }

        std::vector<uchar> status;
        std::vector<float> error;

        // Use Lucas-Kanade optical flow to track features
        cv::calcOpticalFlowPyrLK(
            previous_image_, current_image,
            previous_features_, current_features_,
            status, error
        );

        // Filter out lost features
        std::vector<cv::Point2f> filtered_features;
        std::vector<bool> filtered_tracked;
        std::vector<double> filtered_depth;
        std::vector<int> filtered_ids;

        for (size_t i = 0; i < current_features_.size(); ++i) {
            if (status[i] && error[i] < reprojection_threshold_) {
                filtered_features.push_back(current_features_[i]);
                filtered_tracked.push_back(true);
                filtered_depth.push_back(feature_depth_[i]);
                filtered_ids.push_back(feature_ids_[i]);
            }
        }

        current_features_ = filtered_features;
        feature_tracked_ = filtered_tracked;
        feature_depth_ = filtered_depth;
        feature_ids_ = filtered_ids;

        // Store current image for next iteration
        previous_image_ = current_image.clone();
        previous_features_ = current_features_;
    }

    void add_new_features(const cv::Mat& image) {
        if (current_features_.size() >= max_feature_count_) return;

        // Detect new features in areas without existing features
        cv::Mat mask = cv::Mat::ones(image.size(), CV_8UC1);

        // Create mask to avoid areas near existing features
        for (const auto& feature : current_features_) {
            cv::circle(mask, feature, 30, cv::Scalar(0), -1);  // Mask out 30-pixel radius
        }

        std::vector<cv::Point2f> new_corners;
        int needed_features = max_feature_count_ - current_features_.size();

        cv::goodFeaturesToTrack(image, new_corners, needed_features, 0.01, 10, mask);

        // Add new features
        for (const auto& corner : new_corners) {
            current_features_.push_back(corner);
            feature_tracked_.push_back(true);
            feature_depth_.push_back(-1.0);  // Unknown depth
            static int next_feature_id = 0;
            feature_ids_.push_back(next_feature_id++);
        }
    }

    bool estimate_motion_from_features(Eigen::Vector3d& translation,
                                     Eigen::Quaterniond& rotation) {
        if (previous_features_.size() < 8) return false;  // Need minimum correspondences

        // Use OpenCV's essential matrix estimation for motion
        std::vector<uchar> mask;

        cv::Mat essential = cv::findEssentialMat(
            previous_features_, current_features_,
            focal_length_, principal_point_,
            cv::RANSAC, 0.999, 1.0, mask
        );

        if (essential.empty()) return false;

        // Decompose essential matrix to get rotation and translation
        cv::Mat R, t;
        int inliers = cv::recoverPose(essential, previous_features_, current_features_,
                                     R, t, focal_length_, principal_point_);

        if (inliers < 8) return false;

        // Convert to Eigen
        rotation = Eigen::Matrix3d(R);
        translation << t.at<double>(0), t.at<double>(1), t.at<double>(2);

        // Normalize translation to unit vector if needed
        if (translation.norm() > 0.001) {  // Avoid division by zero
            translation.normalize();
        }

        return true;
    }

    void update_state_with_motion(const Eigen::Vector3d& translation,
                                 const Eigen::Quaterniond& rotation) {
        // Apply motion estimate to update state
        // This is a simplified approach - full VIO uses optimization-based methods

        // Update orientation
        Eigen::Quaterniond current_quat(state_(6 + 3), state_(6),
                                       state_(6 + 1), state_(6 + 2));
        Eigen::Quaterniond new_quat = current_quat * rotation;
        new_quat.normalize();

        state_.segment<4>(6) << new_quat.x(), new_quat.y(),
                                new_quat.z(), new_quat.w();

        // Update position based on translation
        state_.segment<3>(0) += translation * 0.1;  // Scale appropriately
    }

    cv::Mat previous_image_;
};

} // namespace humanoid_sensors
```

## Multi-Sensor Integration Architecture

### Sensor Fusion Framework for Humanoid Robots

```cpp
// sensor_fusion_framework.cpp - Multi-sensor fusion framework for humanoid robots
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include <mutex>

namespace humanoid_sensors {

// Sensor types for humanoid robots
enum class SensorType {
    IMU,
    CAMERA,
    ENCODER,
    FORCE_TORQUE,
    LIDAR,
    GPS,
    BAROMETER
};

// Sensor data structure
struct SensorData {
    SensorType type;
    Eigen::VectorXd measurement;
    double timestamp;
    int sensor_id;
    Eigen::MatrixXd covariance;  // Measurement uncertainty
};

// State vector for humanoid: position, orientation, velocity, angular velocity, joint positions, joint velocities
struct RobotState {
    Eigen::Vector3d position;           // World position
    Eigen::Quaterniond orientation;     // World orientation
    Eigen::Vector3d linear_velocity;    // World linear velocity
    Eigen::Vector3d angular_velocity;   // Body angular velocity
    std::vector<double> joint_positions;    // Joint positions
    std::vector<double> joint_velocities;   // Joint velocities
    Eigen::Vector3d com_position;       // Center of mass position
    Eigen::Vector3d com_velocity;       // Center of mass velocity
    double timestamp;

    RobotState(int num_joints = 0) : joint_positions(num_joints, 0.0),
                                   joint_velocities(num_joints, 0.0) {}
};

class SensorFusionFramework {
private:
    // Estimators for different sensor combinations
    std::unique_ptr<EKFStateEstimator> ekf_estimator_;
    std::unique_ptr<UKFStateEstimator> ukf_estimator_;
    std::unique_ptr<VIOEstimator> vio_estimator_;

    // Sensor data buffers for time synchronization
    std::map<SensorType, std::deque<SensorData>> sensor_buffers_;
    std::mutex buffer_mutex_;

    // Robot state
    RobotState current_state_;
    Eigen::MatrixXd state_covariance_;

    // Time synchronization parameters
    double time_sync_window_;  // Window for temporal alignment
    bool initialized_;

    // Joint information
    int num_joints_;

public:
    SensorFusionFramework(int num_joints)
        : num_joints_(num_joints), time_sync_window_(0.01), initialized_(false) {

        current_state_ = RobotState(num_joints);
        state_covariance_ = Eigen::MatrixXd::Identity(13, 13) * 0.1;  // Simplified state

        // Initialize estimators
        ekf_estimator_ = std::make_unique<EKFStateEstimator>();
        ukf_estimator_ = std::make_unique<UKFStateEstimator>();
        // vio_estimator_ would need camera parameters to initialize
    }

    void process_sensor_data(const SensorData& sensor_data) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        // Add to appropriate buffer
        sensor_buffers_[sensor_data.type].push_back(sensor_data);

        // Clean old data outside sync window
        clean_old_data(sensor_data.timestamp);

        // Attempt fusion when we have compatible sensor data
        attempt_fusion(sensor_data.timestamp);
    }

    void add_imu_data(const Eigen::Vector3d& accel,
                     const Eigen::Vector3d& gyro,
                     double timestamp) {

        SensorData imu_data;
        imu_data.type = SensorType::IMU;
        imu_data.measurement.resize(6);
        imu_data.measurement << accel, gyro;
        imu_data.timestamp = timestamp;
        imu_data.sensor_id = 0;  // Primary IMU
        imu_data.covariance = Eigen::MatrixXd::Identity(6, 6) * 0.01;

        process_sensor_data(imu_data);
    }

    void add_encoder_data(const std::vector<double>& joint_positions,
                         const std::vector<double>& joint_velocities,
                         double timestamp) {

        int n = joint_positions.size();
        SensorData encoder_data;
        encoder_data.type = SensorType::ENCODER;
        encoder_data.measurement.resize(2 * n);  // Positions and velocities
        for (int i = 0; i < n; ++i) {
            encoder_data.measurement(i) = joint_positions[i];
            encoder_data.measurement(i + n) = joint_velocities[i];
        }
        encoder_data.timestamp = timestamp;
        encoder_data.covariance = Eigen::MatrixXd::Identity(2 * n, 2 * n) * 0.001;

        process_sensor_data(encoder_data);
    }

    void add_force_torque_data(const Eigen::Vector3d& force,
                              const Eigen::Vector3d& torque,
                              int foot_id,  // 0 for left foot, 1 for right foot
                              double timestamp) {

        SensorData ft_data;
        ft_data.type = SensorType::FORCE_TORQUE;
        ft_data.measurement.resize(6);  // Force + Torque
        ft_data.measurement << force, torque;
        ft_data.timestamp = timestamp;
        ft_data.sensor_id = foot_id;
        ft_data.covariance = Eigen::MatrixXd::Identity(6, 6) * 0.1;

        process_sensor_data(ft_data);
    }

    RobotState get_fused_state() const {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        return current_state_;
    }

    Eigen::MatrixXd get_state_covariance() const {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        return state_covariance_;
    }

    // Multi-rate fusion: different sensors update at different rates
    void attempt_fusion(double current_time) {
        // Check if we have data for fusion
        bool has_imu = !sensor_buffers_[SensorType::IMU].empty();
        bool has_encoders = !sensor_buffers_[SensorType::ENCODER].empty();
        bool has_ft = !sensor_buffers_[SensorType::FORCE_TORQUE].empty();

        if (has_imu) {
            // Use IMU for continuous prediction updates
            auto imu_data = sensor_buffers_[SensorType::IMU].back();

            // Convert to EKF IMU format
            EKFStateEstimator::IMUData ekf_imu;
            ekf_imu.accelerometer = imu_data.measurement.head(3);
            ekf_imu.gyroscope = imu_data.measurement.tail(3);
            ekf_imu.timestamp = imu_data.timestamp;

            ekf_estimator_->predict(ekf_imu);
        }

        if (has_encoders) {
            // Use encoder data for position updates
            auto encoder_data = sensor_buffers_[SensorType::ENCODER].back();

            // Update joint state
            int n = num_joints_;
            for (int i = 0; i < n && i < current_state_.joint_positions.size(); ++i) {
                current_state_.joint_positions[i] = encoder_data.measurement(i);
                current_state_.joint_velocities[i] = encoder_data.measurement(i + n);
            }

            // If we have position sensors, update position estimate
            // This would involve more complex kinematic calculations in practice
        }

        if (has_ft && has_imu) {
            // Use force/torque sensors for contact detection and state refinement
            auto ft_data = sensor_buffers_[SensorType::FORCE_TORQUE].back();
            auto imu_data = sensor_buffers_[SensorType::IMU].back();

            // Detect contact and update state based on contact constraints
            if (std::abs(ft_data.measurement.head(3).norm()) > 10.0) {  // Contact threshold
                // Apply contact constraints to state estimate
                apply_contact_constraints(ft_data, imu_data);
            }
        }

        // Update current state from estimator
        if (ekf_estimator_) {
            current_state_.position = ekf_estimator_->get_position();
            current_state_.orientation = ekf_estimator_->get_orientation();
            current_state_.linear_velocity = ekf_estimator_->get_velocity();
            current_state_.angular_velocity = ekf_estimator_->get_angular_velocity();
            state_covariance_ = ekf_estimator_->get_covariance();
        }
    }

private:
    void clean_old_data(double current_time) {
        // Remove sensor data older than the sync window
        for (auto& [sensor_type, buffer] : sensor_buffers_) {
            while (!buffer.empty() &&
                   (current_time - buffer.front().timestamp) > time_sync_window_) {
                buffer.pop_front();
            }
        }
    }

    void apply_contact_constraints(const SensorData& ft_data,
                                  const SensorData& imu_data) {
        // Apply contact constraints to refine state estimate
        // This is where Zero Moment Point (ZMP) calculations and balance constraints would be applied

        // Example: Use force/torque data to refine center of pressure estimate
        Eigen::Vector3d contact_force = ft_data.measurement.head(3);
        Eigen::Vector3d contact_torque = ft_data.measurement.tail(3);

        // Calculate center of pressure (simplified)
        double fz = contact_force(2);
        if (std::abs(fz) > 0.1) {  // Avoid division by zero
            double cop_x = -contact_torque(1) / fz;
            double cop_y = contact_torque(0) / fz;

            // Use this information to refine position estimate
            // In practice, this would involve more sophisticated contact-aided estimation
        }
    }
};

// Real-time safe sensor fusion interface for humanoid control
class RealTimeSensorFusion {
private:
    std::unique_ptr<SensorFusionFramework> fusion_framework_;
    mutable std::mutex state_mutex_;
    RobotState latest_state_;

public:
    RealTimeSensorFusion(int num_joints) {
        fusion_framework_ = std::make_unique<SensorFusionFramework>(num_joints);
    }

    // Thread-safe state access for real-time control
    RobotState get_state() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return latest_state_;
    }

    // Non-blocking sensor data input
    void add_sensor_data(const SensorData& sensor_data) {
        // Process in background or queue for processing
        // In real implementation, this might use lock-free queues
        fusion_framework_->process_sensor_data(sensor_data);

        // Update latest state
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = fusion_framework_->get_fused_state();
    }

    // Specialized methods for common sensor types
    void add_imu_data(const Eigen::Vector3d& accel,
                     const Eigen::Vector3d& gyro,
                     double timestamp) {
        fusion_framework_->add_imu_data(accel, gyro, timestamp);

        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = fusion_framework_->get_fused_state();
    }

    void add_encoder_data(const std::vector<double>& joint_positions,
                         const std::vector<double>& joint_velocities,
                         double timestamp) {
        fusion_framework_->add_encoder_data(joint_positions, joint_velocities, timestamp);

        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = fusion_framework_->get_fused_state();
    }
};

} // namespace humanoid_sensors
```

## Summary

Sensor fusion for humanoid robotics combines data from multiple sensors to provide robust, accurate state estimates essential for balance control, navigation, and manipulation. **Kalman filtering** provides optimal state estimation by combining motion models with sensor measurements, while **Visual-Inertial Odometry** integrates camera and IMU data for accurate pose estimation. **Multi-sensor architectures** coordinate diverse sensor types with different update rates and characteristics.

Key capabilities include:
- **Extended/Unscented Kalman Filters** for nonlinear state estimation with uncertainty quantification
- **Visual-Inertial Odometry** for robust localization using complementary visual and inertial sensing
- **Multi-rate fusion** techniques that handle sensors with different update frequencies
- **Real-time sensor fusion** frameworks optimized for control system integration

In Chapter 12, we will transition to Module 4: Humanoid Integration, beginning with whole-body control strategies that coordinate multiple subsystems for complex humanoid behaviors.

## Review Questions

1. **Conceptual**: Compare the computational complexity and accuracy of EKF vs UKF for humanoid state estimation. When would you choose one over the other?

2. **Applied**: Implement a real-time sensor fusion system that integrates IMU, encoder, and force/torque sensors for humanoid balance control with < 5ms update time.

3. **Structural**: Analyze the observability properties of different sensor combinations for humanoid state estimation (e.g., IMU-only, IMU+encoders, full sensor suite).