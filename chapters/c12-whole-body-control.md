# Chapter 12: Whole-Body Control for Humanoid Robotics

## Learning Outcomes
By the end of this chapter, students will be able to:
- Understand the principles of whole-body control for humanoid robots
- Implement quadratic programming (QP) based controllers for multi-task optimization
- Design hierarchical control frameworks for simultaneous task execution
- Apply kinematic and dynamic constraints in whole-body control systems
- Evaluate the computational complexity and real-time performance of whole-body controllers

## Overview
Whole-body control represents a sophisticated approach to managing the complex dynamics of humanoid robots by simultaneously optimizing multiple control objectives across all degrees of freedom. This chapter explores the mathematical foundations of whole-body control, focusing on quadratic programming formulations that enable humanoid robots to perform multiple tasks simultaneously while respecting physical constraints. We'll examine hierarchical optimization techniques that prioritize critical tasks while achieving secondary objectives, and explore practical implementations using state-of-the-art control frameworks.

## Key Concepts
- **Quadratic Programming (QP)**: Mathematical optimization technique for solving whole-body control problems with quadratic cost functions and linear constraints
- **Task Prioritization**: Hierarchical approach to control where primary tasks (like balance) take precedence over secondary tasks (like reaching)
- **Constraint Handling**: Integration of joint limits, torque limits, and contact constraints into the control formulation
- **Kinematic vs Dynamic Control**: Trade-offs between position-based and force-based control approaches
- **Real-time Optimization**: Computational considerations for achieving stable control at high frequencies

## 12.1 Mathematical Foundations of Whole-Body Control

Whole-body control addresses the challenge of coordinating multiple control objectives simultaneously in humanoid robots. Unlike traditional decoupled control approaches, whole-body control formulates the problem as a single optimization task that considers all degrees of freedom and constraints together.

### 12.1.1 Optimization Formulation

The whole-body control problem can be formulated as a quadratic program:

```
min_u ||J * u - b||² + λ * ||u||²
subject to: A_eq * u = b_eq
           A_ineq * u <= b_ineq
```

Where:
- `u` represents the control variables (joint velocities, forces, or accelerations)
- `J` is the task Jacobian matrix mapping control variables to task space
- `b` is the desired task velocity or acceleration
- `λ` is a regularization parameter for numerical stability
- Equality and inequality constraints represent physical limits and requirements

### 12.1.2 Hierarchical Optimization

In practice, whole-body control often uses hierarchical optimization to handle multiple tasks with different priorities:

```
Level 1: min ||J₁ * u - b₁||² (Primary task)
Level 2: min ||J₂ * u - b₂||² (Secondary task)
subject to: ||J₁ * u - b₁||² ≤ ε₁ (Primary task constraint)
```

This approach ensures that higher-priority tasks are satisfied while optimizing lower-priority objectives within the remaining solution space.

## 12.2 Implementation with Quadratic Programming

### 12.2.1 Control Architecture

The whole-body control architecture consists of several key components:

1. **Task Definition Layer**: Defines control objectives (position, orientation, force)
2. **Constraint Formulation**: Translates physical limits into optimization constraints
3. **QP Solver**: Solves the optimization problem in real-time
4. **Integration Layer**: Converts control outputs to actuator commands

### 12.2.2 C++ Implementation Example

```cpp
// WholeBodyController.h
#pragma once
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <vector>

struct Task {
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd desired;
    double weight;
    int priority;

    Task(const Eigen::MatrixXd& J, const Eigen::VectorXd& d,
         double w, int p) : jacobian(J), desired(d), weight(w), priority(p) {}
};

class WholeBodyController {
private:
    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;
    Eigen::VectorXd joint_torques_;

    std::vector<Task> tasks_;
    Eigen::MatrixXd joint_limits_;
    Eigen::VectorXd torque_limits_;

    // QP problem variables
    Eigen::MatrixXd hessian_;
    Eigen::VectorXd gradient_;
    Eigen::MatrixXd constraint_matrix_;
    Eigen::VectorXd constraint_bounds_;

    // QP solver
    qpOASES::QProblem qp_solver_;

public:
    WholeBodyController(int num_dofs);
    void addTask(const Task& task);
    void setJointLimits(const Eigen::MatrixXd& limits);
    void setTorqueLimits(const Eigen::VectorXd& limits);
    Eigen::VectorXd computeControl();
    void updateState(const Eigen::VectorXd& q,
                    const Eigen::VectorXd& q_dot);

private:
    void buildOptimizationProblem();
    void solveQP();
    void applyConstraints();
};

// WholeBodyController.cpp
#include "WholeBodyController.h"
#include <iostream>

WholeBodyController::WholeBodyController(int num_dofs)
    : joint_positions_(Eigen::VectorXd::Zero(num_dofs)),
      joint_velocities_(Eigen::VectorXd::Zero(num_dofs)),
      joint_torques_(Eigen::VectorXd::Zero(num_dofs)),
      joint_limits_(Eigen::MatrixXd::Zero(2, num_dofs)),
      torque_limits_(Eigen::VectorXd::Zero(num_dofs)),
      qp_solver_(num_dofs, 0) // No constraints initially
{
    // Initialize with reasonable defaults
    joint_limits_.row(0) = -M_PI * Eigen::VectorXd::Ones(num_dofs); // lower bounds
    joint_limits_.row(1) = M_PI * Eigen::VectorXd::Ones(num_dofs);  // upper bounds
    torque_limits_ = 100.0 * Eigen::VectorXd::Ones(num_dofs);       // 100 Nm limits

    // Initialize QP solver options
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qp_solver_.setOptions(options);
}

void WholeBodyController::addTask(const Task& task) {
    tasks_.push_back(task);
}

void WholeBodyController::setJointLimits(const Eigen::MatrixXd& limits) {
    joint_limits_ = limits;
}

void WholeBodyController::setTorqueLimits(const Eigen::VectorXd& limits) {
    torque_limits_ = limits;
}

void WholeBodyController::updateState(const Eigen::VectorXd& q,
                                     const Eigen::VectorXd& q_dot) {
    joint_positions_ = q;
    joint_velocities_ = q_dot;
}

void WholeBodyController::buildOptimizationProblem() {
    int num_dofs = joint_positions_.size();

    // Clear previous tasks
    hessian_ = Eigen::MatrixXd::Zero(num_dofs, num_dofs);
    gradient_ = Eigen::VectorXd::Zero(num_dofs);

    // Aggregate tasks based on priority
    std::map<int, std::vector<Task>> priority_tasks;
    for (const auto& task : tasks_) {
        priority_tasks[task.priority].push_back(task);
    }

    // Process tasks by priority level
    for (const auto& [priority, level_tasks] : priority_tasks) {
        for (const auto& task : level_tasks) {
            // Add task to Hessian: H += J^T * W * J
            Eigen::MatrixXd weighted_jacobian =
                task.weight * task.jacobian.transpose() * task.jacobian;
            hessian_.topLeftCorner(task.jacobian.rows(), num_dofs) += weighted_jacobian;

            // Add task to gradient: g -= J^T * W * desired
            Eigen::VectorXd task_gradient =
                -task.weight * task.jacobian.transpose() * task.desired;
            gradient_.head(task.jacobian.rows()) += task_gradient;
        }
    }

    // Add regularization term for numerical stability
    double regularization = 1e-6;
    hessian_.diagonal().array() += regularization;

    // Apply joint velocity limits as constraints
    int num_constraints = 2 * num_dofs; // upper and lower bounds
    constraint_matrix_ = Eigen::MatrixXd::Zero(num_constraints, num_dofs);
    constraint_bounds_ = Eigen::VectorXd::Zero(num_constraints);

    for (int i = 0; i < num_dofs; ++i) {
        // Joint velocity limits (assuming max velocity of 10 rad/s)
        constraint_matrix_.row(2*i) = Eigen::VectorXd::Unit(num_dofs, i);
        constraint_bounds_[2*i] = 10.0; // upper bound

        constraint_matrix_.row(2*i + 1) = -Eigen::VectorXd::Unit(num_dofs, i);
        constraint_bounds_[2*i + 1] = 10.0; // lower bound (negative)
    }
}

void WholeBodyController::solveQP() {
    int num_dofs = joint_positions_.size();

    // Convert Eigen matrices to qpOASES format
    qpOASES::real_t* hessian_data = new qpOASES::real_t[num_dofs * num_dofs];
    qpOASES::real_t* gradient_data = new qpOASES::real_t[num_dofs];
    qpOASES::real_t* constraint_data = new qpOASES::real_t[num_dofs * constraint_matrix_.rows()];
    qpOASES::real_t* bounds_data = new qpOASES::real_t[constraint_matrix_.rows()];

    // Copy data
    for (int i = 0; i < num_dofs; ++i) {
        for (int j = 0; j < num_dofs; ++j) {
            hessian_data[i * num_dofs + j] = hessian_(i, j);
        }
        gradient_data[i] = gradient_(i);
    }

    for (int i = 0; i < constraint_matrix_.rows(); ++i) {
        for (int j = 0; j < num_dofs; ++j) {
            constraint_data[i * num_dofs + j] = constraint_matrix_(i, j);
        }
        bounds_data[i] = constraint_bounds_[i];
    }

    // Solve QP problem
    qpOASES::real_t* solution = new qpOASES::real_t[num_dofs];
    int nWSR = 100; // maximum number of working set recalculations

    qpOASES::returnValue returnvalue = qp_solver_.init(
        hessian_data, gradient_data,
        constraint_data,
        nullptr, nullptr, // lower/upper bounds on optimization variables (set to infinity)
        bounds_data,
        nWSR
    );

    if (returnvalue == qpOASES::SUCCESSFUL_RETURN) {
        qp_solver_.getPrimalSolution(solution);

        // Copy solution back to joint velocities
        for (int i = 0; i < num_dofs; ++i) {
            joint_velocities_[i] = solution[i];
        }
    } else {
        std::cerr << "QP solver failed with return value: " << returnvalue << std::endl;
        // Fallback: set to zero velocities
        joint_velocities_.setZero();
    }

    // Clean up
    delete[] hessian_data;
    delete[] gradient_data;
    delete[] constraint_data;
    delete[] bounds_data;
    delete[] solution;
}

Eigen::VectorXd WholeBodyController::computeControl() {
    buildOptimizationProblem();
    solveQP();

    // Clear tasks for next iteration
    tasks_.clear();

    return joint_velocities_;
}

// Example usage
int main() {
    // Create controller for a 28-DOF humanoid
    WholeBodyController controller(28);

    // Set initial state
    Eigen::VectorXd initial_positions = Eigen::VectorXd::Zero(28);
    Eigen::VectorXd initial_velocities = Eigen::VectorXd::Zero(28);
    controller.updateState(initial_positions, initial_velocities);

    // Add a reaching task (example)
    Eigen::MatrixXd reaching_jacobian = Eigen::MatrixXd::Zero(6, 28); // 6-DOF task space
    Eigen::VectorXd reaching_desired = Eigen::VectorXd::Zero(6);
    reaching_desired << 0.1, 0.2, 0.0, 0.0, 0.0, 0.0; // desired position/velocity

    Task reaching_task(reaching_jacobian, reaching_desired, 1.0, 1); // priority 1
    controller.addTask(reaching_task);

    // Add a balance task (example)
    Eigen::MatrixXd balance_jacobian = Eigen::MatrixXd::Zero(2, 28); // 2-DOF CoM control
    Eigen::VectorXd balance_desired = Eigen::VectorXd::Zero(2);
    balance_desired << 0.0, 0.0; // maintain CoM position

    Task balance_task(balance_jacobian, balance_desired, 10.0, 0); // priority 0 (higher)
    controller.addTask(balance_task);

    // Compute control
    Eigen::VectorXd control_output = controller.computeControl();

    std::cout << "Computed joint velocities:\n" << control_output.transpose() << std::endl;

    return 0;
}
```

## 12.3 Hierarchical Task Prioritization

### 12.3.1 Priority-Based Task Allocation

Hierarchical control systems organize tasks by importance, ensuring that critical objectives (like maintaining balance) take precedence over secondary goals (like reaching for objects).

```cpp
// HierarchicalTaskManager.h
#pragma once
#include <vector>
#include <memory>
#include "WholeBodyController.h"

class HierarchicalTaskManager {
private:
    std::vector<std::vector<Task>> priority_levels_;
    WholeBodyController& controller_;

public:
    explicit HierarchicalTaskManager(WholeBodyController& controller);
    void addTask(int priority_level, const Task& task);
    void clearTasks();
    Eigen::VectorXd computeHierarchicalControl();

private:
    void processPriorityLevel(int level,
                            const Eigen::VectorXd& previous_solution = Eigen::VectorXd());
};
```

### 12.3.2 Dynamic Task Reconfiguration

Advanced whole-body controllers can dynamically adjust task priorities based on environmental conditions and robot state.

```cpp
// AdaptiveTaskScheduler.cpp
#include "HierarchicalTaskManager.h"

class AdaptiveTaskScheduler {
private:
    HierarchicalTaskManager& task_manager_;
    double balance_threshold_;
    double contact_threshold_;

public:
    AdaptiveTaskScheduler(HierarchicalTaskManager& tm)
        : task_manager_(tm), balance_threshold_(0.1), contact_threshold_(50.0) {}

    void updatePriorities(const Eigen::VectorXd& robot_state,
                        const std::vector<double>& contact_forces) {
        // Check if robot is in danger of falling
        double com_deviation = calculateCoMDeviation(robot_state);

        if (com_deviation > balance_threshold_) {
            // Increase priority of balance tasks
            adjustTaskPriority("balance", 0); // highest priority
        }

        // Check for contact forces
        for (size_t i = 0; i < contact_forces.size(); ++i) {
            if (contact_forces[i] > contact_threshold_) {
                // Increase priority of contact maintenance tasks
                adjustTaskPriority("contact_" + std::to_string(i), 1);
            }
        }
    }

private:
    double calculateCoMDeviation(const Eigen::VectorXd& state) {
        // Simplified CoM calculation - in practice would use full kinematics
        return std::abs(state[0]); // Example: x-position deviation
    }

    void adjustTaskPriority(const std::string& task_name, int new_priority) {
        // Implementation would search for and reassign task priorities
    }
};
```

## 12.4 Real-Time Performance Considerations

### 12.4.1 Computational Complexity Analysis

Whole-body control systems must operate within strict real-time constraints. The computational complexity depends on:

- Number of optimization variables (degrees of freedom)
- Number of constraints (joint limits, contact constraints)
- QP solver algorithm and implementation
- Task hierarchy depth

Typical performance requirements for humanoid robots:
- Control frequency: 100-1000 Hz
- Solution time: < 5 ms for real-time stability
- Memory usage: < 100 MB for embedded systems

### 12.4.2 Optimization Strategies

```cpp
// EfficientWholeBodyController.h - Optimized version
#pragma once
#include <Eigen/Dense>
#include <memory>

class EfficientWholeBodyController {
private:
    // Pre-allocated matrices to avoid allocation in real-time loop
    Eigen::MatrixXd hessian_buffer_;
    Eigen::VectorXd gradient_buffer_;
    Eigen::MatrixXd constraint_buffer_;
    Eigen::VectorXd bounds_buffer_;

    // Cached factorizations for faster solving
    Eigen::LLT<Eigen::MatrixXd> llt_solver_;

    // Sparse matrix representations for large systems
    Eigen::SparseMatrix<double> sparse_jacobian_;

public:
    EfficientWholeBodyController(int num_dofs);
    Eigen::VectorXd computeControlOptimized();

private:
    void precomputeFactorizations();
    void solveSparseQP();
};
```

## 12.5 Integration with ROS 2 Control Framework

### 12.5.1 ROS 2 Controller Interface

```cpp
// whole_body_controller.hpp
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "WholeBodyController.h"

namespace whole_body_controller {

class WholeBodyController : public controller_interface::ControllerInterface {
public:
    WholeBodyController();
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

private:
    std::vector<hardware_interface::LoanedCommandInterface> joint_command_interfaces_;
    std::vector<hardware_interface::LoanedStateInterface> joint_state_interfaces_;

    std::unique_ptr<WholeBodyController> wb_controller_;

    // ROS 2 parameters for whole-body control
    double control_frequency_;
    std::vector<std::string> joint_names_;

    // Task publishers/subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
    rclcpp::Publisher<std::vector<double>>::SharedPtr control_output_pub_;

    void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
};

} // namespace whole_body_controller
```

## 12.6 Practical Considerations and Limitations

### 12.6.1 Numerical Stability

Whole-body control systems must handle numerical instabilities that can arise from:

- Poorly conditioned Jacobians
- Singular configurations
- Ill-posed optimization problems
- Floating-point precision errors

### 12.6.2 Sensor Integration

Real-world whole-body control requires integration with various sensors:

- IMU for orientation and acceleration
- Force/torque sensors for contact detection
- Joint encoders for position feedback
- Vision systems for environment perception

## Summary

Whole-body control represents a sophisticated approach to managing the complex dynamics of humanoid robots by simultaneously optimizing multiple control objectives. The mathematical foundation in quadratic programming enables the integration of multiple tasks with different priorities while respecting physical constraints. Implementation requires careful attention to computational efficiency, numerical stability, and real-time performance requirements. Integration with ROS 2 control frameworks enables deployment on real robotic platforms.

The success of whole-body control systems depends on proper task prioritization, constraint handling, and computational optimization. Modern implementations leverage efficient QP solvers and hierarchical optimization techniques to achieve stable control at high frequencies necessary for humanoid robot operation.

## Review Questions

1. Explain the mathematical formulation of whole-body control as a quadratic programming problem. What are the key components of the optimization?

2. Describe the hierarchical task prioritization approach in whole-body control. How does it ensure critical tasks take precedence over secondary objectives?

3. What are the computational challenges in implementing whole-body control for humanoid robots? How can these be addressed in real-time systems?

4. Compare kinematic vs. dynamic approaches to whole-body control. What are the trade-offs between these methods?

5. How does constraint handling work in whole-body control systems? What types of constraints are typically considered?

6. Design a whole-body control system for a humanoid robot performing a reaching task while maintaining balance. Specify the task hierarchy and constraints.

7. What are the key performance requirements for real-time whole-body control? How do these impact the choice of optimization algorithms?

8. Explain how sensor integration affects the performance of whole-body control systems. What types of sensors are critical for different control objectives?