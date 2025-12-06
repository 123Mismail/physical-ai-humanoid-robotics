---
id: c10-real-time-algorithms
title: "Chapter 10: Real-Time Control Algorithms for Humanoid Robotics"
sidebar_label: "C10: Real-Time Algorithms"
sidebar_position: 10
---

# Chapter 10: Real-Time Control Algorithms for Humanoid Robotics

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Implement** Model Predictive Control (MPC) algorithms optimized for real-time humanoid robot control
2. **Design** whole-body control frameworks that integrate multiple control objectives simultaneously
3. **Deploy** trajectory optimization algorithms with real-time constraints for humanoid motion planning
4. **Evaluate** computational complexity and timing performance of control algorithms on embedded systems

## Overview

Chapter 9 established the foundational concepts of real-time control systems and embedded hardware platforms for humanoid robotics. Building on that foundation, this chapter delves into the sophisticated control algorithms that enable humanoid robots to perform complex, dynamic behaviors in real-time.

**Real-time control algorithms** for humanoid robotics must balance computational efficiency with control performance to meet strict timing constraints while achieving complex behaviors. Unlike simple PID controllers that operate on individual joints, humanoid robots require advanced algorithms that consider whole-body dynamics, balance constraints, contact forces, and multiple simultaneous objectives.

**Model Predictive Control (MPC)** represents a key approach for humanoid control, predicting future system behavior over a finite horizon and optimizing control inputs to achieve desired objectives while respecting constraints. **Whole-body control** frameworks integrate multiple control tasks (balance, manipulation, locomotion) into unified optimization problems that consider the full robot dynamics.

This chapter explores these advanced control algorithms, focusing on their real-time implementation, computational optimization, and practical deployment on embedded systems. You will learn to implement MPC controllers, design whole-body control frameworks, and optimize algorithms for real-time performance on resource-constrained platforms.

## Key Concepts

- **Model Predictive Control (MPC)**: Control strategy that uses a model of the system to predict future behavior and optimize control inputs over a finite horizon
- **Whole-Body Control**: Framework that formulates multiple control objectives (balance, manipulation, locomotion) as a single optimization problem
- **Quadratic Programming (QP)**: Mathematical optimization technique used to solve control problems with quadratic cost functions and linear constraints
- **Zero Moment Point (ZMP)**: Point where the net moment of ground reaction forces is zero, used for balance control in humanoid robots
- **Centroidal Dynamics**: Simplified model of robot dynamics focusing on the center of mass and angular momentum
- **Trajectory Optimization**: Process of finding optimal robot trajectories that minimize cost functions while satisfying constraints
- **Real-Time Iterative Algorithms**: Optimization methods that provide approximate solutions within real-time constraints by performing limited iterations
- **Constraint Handling**: Techniques for managing physical limits, contact constraints, and safety requirements in real-time control

## Model Predictive Control for Humanoid Robots

Model Predictive Control (MPC) is particularly well-suited for humanoid robotics due to its ability to handle constraints and predict future behavior:

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  Current State  │───►│  MPC Prediction  │───►│  Optimal Control │
│  (Position,     │    │  Model (Horizon │    │  Inputs         │
│   Velocity,     │    │  N steps)       │    │  (Joint Torques │
│   Balance)      │    │                 │    │   or Forces)     │
└─────────────────┘    └──────────────────┘    └──────────────────┘
         │                       │                        │
         ▼                       ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  State Estimation│    │  Constraint     │    │  < 5ms           │
│  (IMU, Encoders)│    │  Optimization   │    │  Computation     │
│                 │    │  (QP Solver)     │    │  Time           │
└─────────────────┘    └──────────────────┘    └──────────────────┘
```

MPC for humanoid robots typically uses simplified models like the Linear Inverted Pendulum Model (LIPM) or centroidal dynamics to make the optimization tractable for real-time execution.

### Linear Inverted Pendulum Model (LIPM) for Balance Control

```cpp
// lipm_mpc.cpp - Linear Inverted Pendulum Model for humanoid balance control
#include <Eigen/Dense>
#include <vector>
#include <chrono>

namespace humanoid_control {

class LIPMMPC {
private:
    // Robot parameters
    double gravity_;
    double height_;  // Constant CoM height
    double omega_;   // Natural frequency sqrt(g/h)

    // MPC parameters
    int horizon_steps_;
    double dt_;      // Control timestep
    double Q_pos_;   // State cost weight
    double R_control_; // Control cost weight

    // State prediction matrices
    Eigen::MatrixXd A_discrete_;
    Eigen::MatrixXd B_discrete_;

    // Optimization matrices
    Eigen::MatrixXd H_qp_;  // Hessian matrix
    Eigen::VectorXd f_qp_;  // Linear term
    Eigen::MatrixXd A_ineq_; // Inequality constraints
    Eigen::VectorXd b_ineq_; // Inequality constraint bounds

public:
    LIPMMPC(double com_height, double dt, int horizon)
        : gravity_(9.81), height_(com_height), dt_(dt), horizon_steps_(horizon) {

        // Calculate natural frequency
        omega_ = std::sqrt(gravity_ / height_);

        // Initialize cost weights
        Q_pos_ = 10.0;      // High weight on position tracking
        R_control_ = 0.1;   // Lower weight on control effort

        // Pre-compute discrete-time system matrices
        compute_discrete_matrices();

        // Initialize optimization matrices
        initialize_optimization_matrices();
    }

    void compute_discrete_matrices() {
        // Continuous-time LIPM system: x_dot = A*x + B*u
        // where x = [px; vx] (CoM position and velocity)
        // and u = px_ref (desired CoM position)

        // Continuous system matrices
        Eigen::MatrixXd A_cont(2, 2);
        A_cont << 0, 1,
                  std::pow(omega_, 2), 0;

        Eigen::MatrixXd B_cont(2, 1);
        B_cont << 0,
                  -std::pow(omega_, 2);

        // Discretize using matrix exponential
        Eigen::MatrixXd AB_cont(2, 3);
        AB_cont.topLeftCorner(2, 2) = A_cont;
        AB_cont.topRightCorner(2, 1) = B_cont;

        AB_cont *= dt_;
        Eigen::MatrixXd AB_disc = AB_cont.exp();

        A_discrete_ = AB_disc.topLeftCorner(2, 2);
        B_discrete_ = AB_disc.topRightCorner(2, 1);
    }

    void initialize_optimization_matrices() {
        int n_states = 2;  // px, vx
        int n_controls = 1; // px_ref
        int total_vars = horizon_steps_ * n_controls;

        // Initialize Hessian matrix (quadratic cost terms)
        H_qp_ = Eigen::MatrixXd::Zero(total_vars, total_vars);
        f_qp_ = Eigen::VectorXd::Zero(total_vars);

        // Initialize constraint matrices
        int total_state_vars = (horizon_steps_ + 1) * n_states;
        A_ineq_ = Eigen::MatrixXd::Zero(2 * horizon_steps_, total_vars + total_state_vars);
        b_ineq_ = Eigen::VectorXd::Zero(2 * horizon_steps_);
    }

    Eigen::VectorXd solve_mpc(const Eigen::Vector2d& current_state,
                             const std::vector<Eigen::Vector2d>& reference_trajectory) {
        // Formulate and solve the QP problem
        // min: 0.5 * x^T * H * x + f^T * x
        // s.t: A*x <= b

        // This is a simplified implementation
        // In practice, you would use a QP solver like OSQP or qpOASES

        // Build the QP problem matrices based on current state and reference
        build_qp_matrices(current_state, reference_trajectory);

        // Solve the QP problem (in practice, use a dedicated solver)
        Eigen::VectorXd solution = solve_qp();

        // Extract the first control input (MPC principle)
        return solution.head(1);
    }

private:
    void build_qp_matrices(const Eigen::Vector2d& current_state,
                          const std::vector<Eigen::Vector2d>& reference_trajectory) {
        // Build the QP cost and constraint matrices
        // This involves expanding the prediction model over the horizon
        // and forming the quadratic cost function

        int n_states = 2;
        int n_controls = 1;
        int total_vars = horizon_steps_ * n_controls;

        // Reset matrices
        H_qp_.setZero();
        f_qp_.setZero();

        Eigen::Vector2d state = current_state;

        for (int k = 0; k < horizon_steps_; ++k) {
            // Predict state evolution: x[k+1] = A*x[k] + B*u[k]
            // For LIPM: x = [px; vx]

            // Cost on state deviation from reference
            double pos_ref = reference_trajectory[k][0];
            double vel_ref = reference_trajectory[k][1];

            // Quadratic terms for state cost
            H_qp_(k, k) += Q_pos_ * B_discrete_(0, 0) * B_discrete_(0, 0);  // Simplified

            // Linear terms for state cost
            f_qp_(k) += -2.0 * Q_pos_ * (state(0) - pos_ref) * B_discrete_(0, 0);

            // Control effort cost
            H_qp_(k, k) += R_control_;

            // Predict next state using control input (simplified)
            state = A_discrete_ * state;  // Assuming u=0 for prediction
        }
    }

    Eigen::VectorXd solve_qp() {
        // In a real implementation, this would call a QP solver
        // For this example, we'll return a simple approximation

        // Use conjugate gradient or other iterative method
        // to solve H*x = -f subject to constraints
        Eigen::VectorXd solution = -H_qp_.inverse() * f_qp_;
        return solution;
    }

public:
    // Real-time safe method to get next control command
    double get_next_control_command(const Eigen::Vector2d& current_state,
                                   const std::vector<Eigen::Vector2d>& reference_trajectory) {
        Eigen::VectorXd solution = solve_mpc(current_state, reference_trajectory);

        // Return first element (current control command)
        return solution(0);
    }
};

} // namespace humanoid_control
```

### Trajectory Optimization with Real-Time Constraints

```cpp
// trajectory_optimizer.cpp - Real-time trajectory optimization for humanoid motion
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <thread>

namespace humanoid_control {

class RealTimeTrajectoryOptimizer {
private:
    // Robot dynamics parameters
    double mass_;
    Eigen::Matrix3d inertia_;

    // Optimization parameters
    int num_waypoints_;
    double dt_;
    double max_computation_time_ms_;

    // Cost function weights
    double weight_position_;
    double weight_velocity_;
    double weight_acceleration_;
    double weight_inequality_constraints_;

    // Constraint parameters
    Eigen::Vector3d com_min_limit_;
    Eigen::Vector3d com_max_limit_;
    double max_velocity_limit_;
    double max_acceleration_limit_;

public:
    RealTimeTrajectoryOptimizer(double robot_mass, const Eigen::Matrix3d& robot_inertia,
                              int waypoints, double timestep, double max_comp_time_ms = 2.0)
        : mass_(robot_mass), inertia_(robot_inertia), num_waypoints_(waypoints),
          dt_(timestep), max_computation_time_ms_(max_comp_time_ms) {

        // Initialize cost weights
        weight_position_ = 1.0;
        weight_velocity_ = 0.1;
        weight_acceleration_ = 0.01;
        weight_inequality_constraints_ = 100.0;

        // Initialize constraint limits
        com_min_limit_ << -0.5, -0.5, 0.5;   // Example limits
        com_max_limit_ << 0.5, 0.5, 1.5;
        max_velocity_limit_ = 1.0;    // m/s
        max_acceleration_limit_ = 5.0; // m/s^2
    }

    struct TrajectorySolution {
        std::vector<Eigen::Vector3d> positions;
        std::vector<Eigen::Vector3d> velocities;
        std::vector<Eigen::Vector3d> accelerations;
        bool success;
        double computation_time_ms;
    };

    TrajectorySolution optimize_trajectory(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& start_vel,
        const Eigen::Vector3d& goal_pos,
        const std::vector<Eigen::Vector3d>& via_points = {},
        const std::vector<Eigen::Vector3d>& obstacle_positions = {}) {

        auto start_time = std::chrono::high_resolution_clock::now();

        // Initialize trajectory with straight-line interpolation
        std::vector<Eigen::Vector3d> trajectory_positions =
            initialize_trajectory(start_pos, goal_pos);

        // Iteratively optimize the trajectory
        TrajectorySolution solution;
        solution.positions = trajectory_positions;
        solution.velocities.resize(num_waypoints_);
        solution.accelerations.resize(num_waypoints_);

        // Calculate velocities and accelerations
        for (int i = 0; i < num_waypoints_; ++i) {
            if (i == 0) {
                solution.velocities[i] = start_vel;
                solution.accelerations[i] = Eigen::Vector3d::Zero();
            } else {
                solution.velocities[i] = (solution.positions[i] - solution.positions[i-1]) / dt_;

                if (i > 1) {
                    solution.accelerations[i] = (solution.velocities[i] - solution.velocities[i-1]) / dt_;
                }
            }
        }

        // Iteratively refine the trajectory to minimize cost while satisfying constraints
        bool converged = false;
        int iteration = 0;
        const int max_iterations = 50;  // Limit iterations for real-time safety

        while (!converged && iteration < max_iterations) {
            // Check if we're approaching the time limit
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(
                current_time - start_time).count();

            if (elapsed_ms > max_computation_time_ms_ * 0.8) {  // Use 80% of allowed time
                break;
            }

            // Apply gradient descent step to optimize trajectory
            converged = optimize_step(solution, start_pos, goal_pos, via_points, obstacle_positions);
            iteration++;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        solution.computation_time_ms = std::chrono::duration<double, std::milli>(
            end_time - start_time).count();
        solution.success = (solution.computation_time_ms < max_computation_time_ms_);

        return solution;
    }

private:
    std::vector<Eigen::Vector3d> initialize_trajectory(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& goal_pos) {

        std::vector<Eigen::Vector3d> trajectory(num_waypoints_);

        for (int i = 0; i < num_waypoints_; ++i) {
            double t = static_cast<double>(i) / (num_waypoints_ - 1);
            trajectory[i] = start_pos + t * (goal_pos - start_pos);
        }

        return trajectory;
    }

    bool optimize_step(TrajectorySolution& solution,
                      const Eigen::Vector3d& start_pos,
                      const Eigen::Vector3d& goal_pos,
                      const std::vector<Eigen::Vector3d>& via_points,
                      const std::vector<Eigen::Vector3d>& obstacle_positions) {

        // Calculate gradients of cost function with respect to each waypoint
        std::vector<Eigen::Vector3d> gradients(num_waypoints_);

        for (int i = 0; i < num_waypoints_; ++i) {
            gradients[i] = calculate_gradient(solution, i, start_pos, goal_pos,
                                            via_points, obstacle_positions);
        }

        // Apply gradient descent step
        double step_size = 0.01;  // Small step for stability

        for (int i = 0; i < num_waypoints_; ++i) {
            // Don't modify start point
            if (i == 0) continue;

            solution.positions[i] -= step_size * gradients[i];

            // Apply position constraints
            solution.positions[i] = apply_position_constraints(solution.positions[i]);
        }

        // Recalculate velocities and accelerations
        for (int i = 1; i < num_waypoints_; ++i) {
            solution.velocities[i] = (solution.positions[i] - solution.positions[i-1]) / dt_;

            if (i > 1) {
                solution.accelerations[i] = (solution.velocities[i] - solution.velocities[i-1]) / dt_;

                // Apply velocity and acceleration constraints
                solution.velocities[i] = apply_velocity_constraints(solution.velocities[i]);
                solution.accelerations[i] = apply_acceleration_constraints(solution.accelerations[i]);
            }
        }

        // Check for convergence (simplified check)
        double max_gradient_norm = 0.0;
        for (const auto& grad : gradients) {
            max_gradient_norm = std::max(max_gradient_norm, grad.norm());
        }

        return max_gradient_norm < 1e-6;  // Converged if gradients are small
    }

    Eigen::Vector3d calculate_gradient(const TrajectorySolution& solution, int waypoint_idx,
                                      const Eigen::Vector3d& start_pos,
                                      const Eigen::Vector3d& goal_pos,
                                      const std::vector<Eigen::Vector3d>& via_points,
                                      const std::vector<Eigen::Vector3d>& obstacle_positions) {

        Eigen::Vector3d gradient = Eigen::Vector3d::Zero();

        // Gradient due to position cost (distance to reference trajectory)
        if (waypoint_idx == num_waypoints_ - 1) {
            // Goal position cost
            gradient += 2.0 * weight_position_ * (solution.positions[waypoint_idx] - goal_pos);
        }

        // Gradient due to velocity smoothness
        if (waypoint_idx > 0 && waypoint_idx < num_waypoints_ - 1) {
            // Velocity difference from neighboring points
            Eigen::Vector3d vel_next = (solution.positions[waypoint_idx + 1] -
                                       solution.positions[waypoint_idx]) / dt_;
            Eigen::Vector3d vel_prev = (solution.positions[waypoint_idx] -
                                       solution.positions[waypoint_idx - 1]) / dt_;

            Eigen::Vector3d acc = (vel_next - vel_prev) / dt_;
            gradient += 2.0 * weight_acceleration_ * acc;
        }

        // Gradient due to obstacle avoidance
        for (const auto& obstacle : obstacle_positions) {
            Eigen::Vector3d to_obstacle = solution.positions[waypoint_idx] - obstacle;
            double distance = to_obstacle.norm();

            if (distance < 0.5) {  // Within safety radius
                double force_magnitude = weight_inequality_constraints_ / (distance * distance + 1e-6);
                Eigen::Vector3d repulsive_force = force_magnitude * to_obstacle.normalized();
                gradient += repulsive_force;
            }
        }

        return gradient;
    }

    Eigen::Vector3d apply_position_constraints(const Eigen::Vector3d& pos) {
        Eigen::Vector3d constrained_pos = pos;

        // Apply position limits
        for (int i = 0; i < 3; ++i) {
            constrained_pos(i) = std::max(com_min_limit_(i),
                                         std::min(constrained_pos(i), com_max_limit_(i)));
        }

        return constrained_pos;
    }

    Eigen::Vector3d apply_velocity_constraints(const Eigen::Vector3d& vel) {
        double vel_norm = vel.norm();
        if (vel_norm > max_velocity_limit_) {
            return (vel / vel_norm) * max_velocity_limit_;
        }
        return vel;
    }

    Eigen::Vector3d apply_acceleration_constraints(const Eigen::Vector3d& acc) {
        double acc_norm = acc.norm();
        if (acc_norm > max_acceleration_limit_) {
            return (acc / acc_norm) * max_acceleration_limit_;
        }
        return acc;
    }
};

} // namespace humanoid_control
```

## Whole-Body Control Framework

Whole-body control integrates multiple control objectives into unified optimization problems:

### Quadratic Programming for Whole-Body Control

```cpp
// whole_body_control.cpp - Whole-body control using quadratic programming
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace humanoid_control {

class WholeBodyController {
private:
    // Robot properties
    int num_joints_;
    double robot_mass_;
    Eigen::Vector3d com_position_;

    // Control hierarchy
    struct Task {
        std::string name;
        int priority;
        Eigen::MatrixXd task_jacobian;
        Eigen::VectorXd desired_values;
        Eigen::VectorXd current_values;
        double weight;
    };

    std::vector<Task> tasks_;

    // QP formulation matrices
    Eigen::MatrixXd H_qp_;  // Hessian (cost matrix)
    Eigen::VectorXd f_qp_;  // Linear term
    Eigen::MatrixXd A_eq_;  // Equality constraints
    Eigen::VectorXd b_eq_;  // Equality constraint values
    Eigen::MatrixXd A_ineq_; // Inequality constraints
    Eigen::VectorXd b_ineq_; // Inequality constraint bounds

    // Joint limits and constraints
    Eigen::VectorXd joint_position_limits_min_;
    Eigen::VectorXd joint_position_limits_max_;
    Eigen::VectorXd joint_velocity_limits_;
    Eigen::VectorXd joint_torque_limits_;

public:
    WholeBodyController(int num_joints, double robot_mass)
        : num_joints_(num_joints), robot_mass_(robot_mass) {

        // Initialize constraint vectors
        joint_position_limits_min_ = Eigen::VectorXd::Constant(num_joints_, -M_PI);
        joint_position_limits_max_ = Eigen::VectorXd::Constant(num_joints_, M_PI);
        joint_velocity_limits_ = Eigen::VectorXd::Constant(num_joints_, 10.0);  // rad/s
        joint_torque_limits_ = Eigen::VectorXd::Constant(num_joints_, 100.0);   // Nm

        // Initialize QP matrices (sizes will be set during solve)
        H_qp_ = Eigen::MatrixXd(0, 0);
        f_qp_ = Eigen::VectorXd(0);
        A_eq_ = Eigen::MatrixXd(0, 0);
        b_eq_ = Eigen::VectorXd(0);
        A_ineq_ = Eigen::MatrixXd(0, 0);
        b_ineq_ = Eigen::VectorXd(0);
    }

    void add_task(const std::string& name, int priority,
                  const Eigen::MatrixXd& jacobian,
                  const Eigen::VectorXd& desired_values,
                  double weight = 1.0) {

        Task task;
        task.name = name;
        task.priority = priority;
        task.task_jacobian = jacobian;
        task.desired_values = desired_values;
        task.current_values = Eigen::VectorXd::Zero(desired_values.size());
        task.weight = weight;

        tasks_.push_back(task);

        // Sort tasks by priority (higher priority first)
        std::sort(tasks_.begin(), tasks_.end(),
                 [](const Task& a, const Task& b) {
                     return a.priority > b.priority;
                 });
    }

    struct WBCSolution {
        Eigen::VectorXd joint_positions;
        Eigen::VectorXd joint_velocities;
        Eigen::VectorXd joint_torques;
        std::vector<double> task_errors;
        bool success;
    };

    WBCSolution solve(const Eigen::VectorXd& current_positions,
                     const Eigen::VectorXd& current_velocities,
                     const Eigen::VectorXd& current_torques) {

        WBCSolution solution;
        solution.joint_positions = current_positions;
        solution.joint_velocities = current_velocities;
        solution.joint_torques = current_torques;
        solution.success = false;

        if (tasks_.empty()) {
            solution.success = true;
            return solution;
        }

        try {
            // Build the QP problem based on control hierarchy
            build_hierarchical_qp(current_positions, current_velocities);

            // Solve the QP problem (in practice, use OSQP, qpOASES, or similar)
            Eigen::VectorXd decision_variables = solve_qp_problem();

            // Extract solution
            int total_vars = decision_variables.size();
            if (total_vars >= num_joints_) {
                solution.joint_torques = decision_variables.head(num_joints_);
                solution.success = true;
            }

            // Calculate task errors
            solution.task_errors.resize(tasks_.size());
            for (size_t i = 0; i < tasks_.size(); ++i) {
                Eigen::VectorXd task_result = tasks_[i].task_jacobian * solution.joint_torques;
                solution.task_errors[i] = (task_result - tasks_[i].desired_values).norm();
            }

        } catch (const std::exception& e) {
            // Handle QP solver errors
            solution.success = false;
        }

        return solution;
    }

private:
    void build_hierarchical_qp(const Eigen::VectorXd& current_positions,
                              const Eigen::VectorXd& current_velocities) {

        // Calculate total number of variables and constraints
        int total_variables = num_joints_;  // Joint torques
        int total_constraints = 0;

        // Count constraints from each task
        for (const auto& task : tasks_) {
            total_constraints += task.desired_values.size();
        }

        // Add joint limit constraints
        total_constraints += 2 * num_joints_;  // Position and velocity limits

        // Resize matrices
        H_qp_ = Eigen::MatrixXd::Zero(total_variables, total_variables);
        f_qp_ = Eigen::VectorXd::Zero(total_variables);
        A_eq_ = Eigen::MatrixXd::Zero(0, total_variables);  // Will be built task by task
        b_eq_ = Eigen::VectorXd::Zero(0);                   // Will be built task by task
        A_ineq_ = Eigen::MatrixXd::Zero(total_constraints, total_variables);
        b_ineq_ = Eigen::VectorXd::Zero(total_constraints);

        // Build cost function: minimize weighted sum of task errors and regularization
        // J = sum_i (weight_i * ||J_i * tau - x_des_i||^2) + reg_weight * ||tau||^2

        double regularization_weight = 0.001;  // Small regularization for stability
        H_qp_ = regularization_weight * Eigen::MatrixXd::Identity(total_variables, total_variables);
        f_qp_ = Eigen::VectorXd::Zero(total_variables);

        int constraint_row = 0;

        // Add each task to the optimization problem
        for (const auto& task : tasks_) {
            int task_size = task.desired_values.size();

            // Add task to cost function: ||J * tau - x_des||^2
            // Expands to: tau^T * (J^T * J) * tau - 2 * x_des^T * J * tau + x_des^T * x_des
            H_qp_ += task.weight * task.task_jacobian.transpose() * task.task_jacobian;
            f_qp_ -= 2.0 * task.weight * task.desired_values.transpose() * task.task_jacobian;

            // Add task equality constraints (for hierarchical control)
            // For now, we'll use inequality constraints to allow some flexibility
            A_ineq_.block(constraint_row, 0, task_size, total_variables) = task.task_jacobian;
            b_ineq_.segment(constraint_row, task_size) = task.desired_values + 0.1;  // Allow small tolerance

            constraint_row += task_size;
        }

        // Add joint limit constraints
        // Position limits: q_min <= q <= q_max
        for (int i = 0; i < num_joints_; ++i) {
            A_ineq_(constraint_row, i) = 1.0;
            b_ineq_(constraint_row) = joint_position_limits_max_(i);
            constraint_row++;

            A_ineq_(constraint_row, i) = -1.0;
            b_ineq_(constraint_row) = -joint_position_limits_min_(i);
            constraint_row++;
        }
    }

    Eigen::VectorXd solve_qp_problem() {
        // In a real implementation, this would call a QP solver
        // For this example, we'll use Eigen's LDLT solver for the KKT system

        // Form KKT system: [H A^T; A 0] * [x; lambda] = [-f; b]
        // This is a simplified approach - real systems use specialized QP solvers

        int n = H_qp_.rows();
        int m = A_ineq_.rows();

        if (n == 0) {
            return Eigen::VectorXd::Zero(num_joints_);
        }

        // For inequality constraints, we'll use a simplified approach
        // In practice, active-set or interior-point methods are used

        // Use Eigen's least squares solver as an approximation
        // min: ||H_qp_ * x + f_qp_||^2 + ||A_ineq_ * x - b_ineq_||^2
        Eigen::MatrixXd combined_A = Eigen::MatrixXd::Zero(n + m, n);
        combined_A.topRows(n) = H_qp_;
        combined_A.bottomRows(m) = A_ineq_;

        Eigen::VectorXd combined_b = Eigen::VectorXd::Zero(n + m);
        combined_b.topRows(n) = -f_qp_;
        combined_b.bottomRows(m) = b_ineq_;

        // Solve using least squares
        Eigen::VectorXd solution = combined_A.colPivHouseholderQr().solve(combined_b);

        return solution.head(num_joints_);  // Return only joint torques
    }
};

// Example usage of whole-body control for humanoid
class HumanoidWBC {
private:
    std::unique_ptr<WholeBodyController> wbc_controller_;

    // Task-specific jacobians (would be computed from robot kinematics)
    Eigen::MatrixXd com_jacobian_;
    Eigen::MatrixXd left_foot_jacobian_;
    Eigen::MatrixXd right_foot_jacobian_;
    Eigen::MatrixXd left_hand_jacobian_;
    Eigen::MatrixXd right_hand_jacobian_;
    Eigen::MatrixXd head_jacobian_;

public:
    HumanoidWBC(int num_joints, double robot_mass) {
        wbc_controller_ = std::make_unique<WholeBodyController>(num_joints, robot_mass);

        // Initialize jacobian matrices (sizes depend on your robot)
        // These would be computed from forward kinematics in a real implementation
        com_jacobian_ = Eigen::MatrixXd::Zero(6, num_joints);      // 6 DoF: pos + orientation
        left_foot_jacobian_ = Eigen::MatrixXd::Zero(6, num_joints);
        right_foot_jacobian_ = Eigen::MatrixXd::Zero(6, num_joints);
        left_hand_jacobian_ = Eigen::MatrixXd::Zero(6, num_joints);
        right_hand_jacobian_ = Eigen::MatrixXd::Zero(6, num_joints);
        head_jacobian_ = Eigen::MatrixXd::Zero(6, num_joints);
    }

    void setup_balance_control_tasks() {
        // High priority: Balance control (ZMP or COM control)
        Eigen::VectorXd com_desired = Eigen::VectorXd::Zero(6);  // [x, y, z, roll, pitch, yaw]
        com_desired.segment(0, 3) << 0.0, 0.0, 0.8;  // Desired CoM position (x, y, height)

        wbc_controller_->add_task("balance_control", 100, com_jacobian_, com_desired, 10.0);
    }

    void setup_foot_placement_tasks() {
        // Medium priority: Foot placement for walking
        Eigen::VectorXd left_foot_desired = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd right_foot_desired = Eigen::VectorXd::Zero(6);

        // Set desired foot positions for walking pattern
        wbc_controller_->add_task("left_foot_placement", 80, left_foot_jacobian_, left_foot_desired, 5.0);
        wbc_controller_->add_task("right_foot_placement", 80, right_foot_jacobian_, right_foot_desired, 5.0);
    }

    void setup_manipulation_tasks() {
        // Lower priority: Arm manipulation tasks
        Eigen::VectorXd left_hand_desired = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd right_hand_desired = Eigen::VectorXd::Zero(6);

        wbc_controller_->add_task("left_hand_control", 50, left_hand_jacobian_, left_hand_desired, 1.0);
        wbc_controller_->add_task("right_hand_control", 50, right_hand_jacobian_, right_hand_desired, 1.0);
    }

    WholeBodyController::WBCSolution execute_control(
        const Eigen::VectorXd& current_positions,
        const Eigen::VectorXd& current_velocities,
        const Eigen::VectorXd& current_torques) {

        return wbc_controller_->solve(current_positions, current_velocities, current_torques);
    }
};

} // namespace humanoid_control
```

## Real-Time Optimization Techniques

### Efficient QP Solving for Real-Time Applications

```cpp
// real_time_qp.cpp - Efficient QP solving techniques for real-time control
#include <Eigen/Dense>
#include <chrono>
#include <thread>

namespace humanoid_control {

class RealTimeQP {
private:
    // Warm-start variables for iterative solvers
    Eigen::VectorXd previous_solution_;
    bool has_previous_solution_;

    // Pre-allocated matrices to avoid allocation during real-time loop
    Eigen::MatrixXd H_cached_;
    Eigen::VectorXd g_cached_;
    Eigen::MatrixXd A_cached_;
    Eigen::VectorXd b_cached_;

    // Iterative solver parameters
    int max_iterations_;
    double tolerance_;
    double penalty_parameter_;  // For penalty methods

public:
    RealTimeQP(int max_vars, int max_constraints)
        : max_iterations_(20), tolerance_(1e-4), penalty_parameter_(1000.0),
          has_previous_solution_(false) {

        // Pre-allocate matrices
        previous_solution_ = Eigen::VectorXd::Zero(max_vars);
        H_cached_ = Eigen::MatrixXd::Zero(max_vars, max_vars);
        g_cached_ = Eigen::VectorXd::Zero(max_vars);
        A_cached_ = Eigen::MatrixXd::Zero(max_constraints, max_vars);
        b_cached_ = Eigen::VectorXd::Zero(max_constraints);
    }

    struct QPSolution {
        Eigen::VectorXd solution;
        bool success;
        int iterations;
        double computation_time_ms;
    };

    QPSolution solve_warm_start(const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
                               const Eigen::MatrixXd& A_eq, const Eigen::VectorXd& b_eq,
                               const Eigen::MatrixXd& A_ineq, const Eigen::VectorXd& b_ineq,
                               const Eigen::VectorXd& initial_guess) {

        auto start_time = std::chrono::high_resolution_clock::now();

        // Use the initial guess (warm start) to speed up convergence
        Eigen::VectorXd x = initial_guess;

        // Check if the initial guess is feasible
        bool feasible = check_feasibility(A_eq, b_eq, A_ineq, b_ineq, x);

        if (!feasible) {
            // Find a feasible point first
            x = find_feasible_point(A_eq, b_eq, A_ineq, b_ineq, initial_guess);
        }

        // Iterative QP solver using active-set method (simplified)
        QPSolution solution;
        solution.solution = x;
        solution.success = true;
        solution.iterations = 0;

        // Use projected gradient method for real-time efficiency
        for (int iter = 0; iter < max_iterations_; ++iter) {
            solution.iterations = iter + 1;

            // Compute gradient: g + H*x
            Eigen::VectorXd gradient = g + H * x;

            // Project gradient onto feasible set (simplified)
            Eigen::VectorXd search_direction = -gradient;

            // Line search with constraint checking
            double step_size = 1.0;
            Eigen::VectorXd new_x = x + step_size * search_direction;

            // Check constraints and reduce step if needed
            int max_step_attempts = 5;
            for (int attempt = 0; attempt < max_step_attempts; ++attempt) {
                if (check_feasibility(A_eq, b_eq, A_ineq, b_ineq, new_x)) {
                    break;
                }
                step_size *= 0.5;
                new_x = x + step_size * search_direction;
            }

            // Update solution
            Eigen::VectorXd x_old = x;
            x = new_x;

            // Check for convergence
            if ((x - x_old).norm() < tolerance_) {
                break;
            }
        }

        solution.solution = x;

        auto end_time = std::chrono::high_resolution_clock::now();
        solution.computation_time_ms = std::chrono::duration<double, std::milli>(
            end_time - start_time).count();

        return solution;
    }

private:
    bool check_feasibility(const Eigen::MatrixXd& A_eq, const Eigen::VectorXd& b_eq,
                          const Eigen::MatrixXd& A_ineq, const Eigen::VectorXd& b_ineq,
                          const Eigen::VectorXd& x) {

        // Check equality constraints: ||A_eq * x - b_eq|| < tolerance
        if (A_eq.rows() > 0) {
            Eigen::VectorXd eq_violation = A_eq * x - b_eq;
            if (eq_violation.norm() > 1e-3) {
                return false;
            }
        }

        // Check inequality constraints: A_ineq * x <= b_ineq
        if (A_ineq.rows() > 0) {
            Eigen::VectorXd ineq_violation = A_ineq * x - b_ineq;
            for (int i = 0; i < ineq_violation.size(); ++i) {
                if (ineq_violation(i) > 1e-3) {
                    return false;
                }
            }
        }

        return true;
    }

    Eigen::VectorXd find_feasible_point(const Eigen::MatrixXd& A_eq, const Eigen::VectorXd& b_eq,
                                       const Eigen::MatrixXd& A_ineq, const Eigen::VectorXd& b_ineq,
                                       const Eigen::VectorXd& initial_guess) {

        // Use penalty method to find a feasible point
        // min: ||A_eq * x - b_eq||^2 + sum(max(0, A_ineq * x - b_ineq)^2)

        Eigen::VectorXd x = initial_guess;
        int max_feasibility_iterations = 50;

        for (int iter = 0; iter < max_feasibility_iterations; ++iter) {
            // Compute constraint violations
            Eigen::VectorXd eq_violation = A_eq * x - b_eq;
            Eigen::VectorXd ineq_violation = A_ineq * x - b_ineq;

            // Apply penalty for inequality violations
            for (int i = 0; i < ineq_violation.size(); ++i) {
                if (ineq_violation(i) > 0) {
                    ineq_violation(i) = ineq_violation(i) * penalty_parameter_;
                } else {
                    ineq_violation(i) = 0;
                }
            }

            // Gradient of penalty function
            Eigen::VectorXd grad = 2.0 * A_eq.transpose() * eq_violation +
                                  2.0 * A_ineq.transpose() * ineq_violation;

            // Gradient descent step
            x -= 0.01 * grad;  // Fixed step size for real-time safety

            // Check if feasible now
            if (check_feasibility(A_eq, b_eq, A_ineq, b_ineq, x)) {
                break;
            }
        }

        return x;
    }
};

} // namespace humanoid_control
```

## Summary

Real-time control algorithms for humanoid robotics enable complex behaviors through sophisticated optimization techniques that balance computational efficiency with control performance. **Model Predictive Control (MPC)** provides predictive capabilities for balance and trajectory following, while **Whole-Body Control** frameworks integrate multiple control objectives into unified optimization problems.

Key capabilities include:
- **MPC algorithms** using simplified models (LIPM) for tractable real-time optimization
- **Trajectory optimization** with iterative methods that meet timing constraints
- **Whole-body control** using hierarchical QP formulations for multi-task coordination
- **Real-time optimization** techniques including warm-starting and iterative methods

In Chapter 11, we will explore sensor fusion techniques that provide the state estimation necessary for these advanced control algorithms, including IMU integration, visual-inertial odometry, and multi-sensor data fusion for humanoid robotics applications.

## Review Questions

1. **Conceptual**: Compare the computational complexity of Model Predictive Control versus PID control for humanoid balance. What are the trade-offs in terms of performance and real-time feasibility?

2. **Applied**: Implement a real-time MPC controller for humanoid balance that runs at 200Hz on an embedded platform, including constraint handling and warm-starting techniques.

3. **Structural**: Analyze the integration challenges when combining whole-body control with trajectory optimization, particularly regarding computational load distribution across control hierarchy levels.