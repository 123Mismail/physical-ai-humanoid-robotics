---
id: c8-advanced-simulation
title: "Chapter 8: Advanced Simulation Techniques"
sidebar_label: "C8: Advanced Simulation"
sidebar_position: 8
---

# Chapter 8: Advanced Simulation Techniques

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Implement** domain randomization techniques to improve sim-to-real transfer performance
2. **Design** multi-robot coordination systems in simulation environments
3. **Deploy** reinforcement learning policies from simulation to real hardware platforms
4. **Evaluate** simulation fidelity and transferability metrics for humanoid robotics applications

## Overview

Chapters 5-7 introduced foundational simulation environments: Gazebo for physics-based simulation, NVIDIA Isaac Sim for GPU-accelerated photorealistic rendering, and Unity for real-time visualization and rapid prototyping. While these platforms provide essential capabilities for humanoid robotics development, advanced applications require sophisticated techniques to bridge the reality gap between simulation and hardware deployment.

**Advanced simulation techniques** encompass methods that enhance simulation fidelity, enable transfer of learned behaviors to real robots, and support complex multi-agent scenarios. These include **domain randomization** for robust policy learning, **multi-robot coordination** frameworks for swarm robotics, and **sim-to-real transfer** methodologies that account for modeling inaccuracies and environmental differences.

This chapter explores these advanced techniques, focusing on practical implementation strategies for humanoid robotics. You will learn to configure domain randomization for robust controller training, implement multi-robot coordination algorithms, and deploy reinforcement learning policies from simulation to real hardware using frameworks like RL-SAR (Reinforcement Learning Simulation to ARgumentation).

## Key Concepts

- **Domain Randomization**: Technique that randomizes simulation parameters (masses, friction, lighting, textures) during training to improve policy robustness to real-world variations
- **Sim-to-Real Transfer**: Process of deploying policies trained in simulation to real hardware, addressing the "reality gap" between simulated and physical environments
- **Multi-Robot Coordination**: Frameworks and algorithms for coordinating multiple robots simultaneously in shared environments
- **System Identification**: Process of determining accurate physical parameters (masses, inertias, friction coefficients) of real robots for simulation model refinement
- **Policy Conversion**: Techniques for converting reinforcement learning policies between different frameworks and hardware platforms (PyTorch to ONNX, etc.)
- **Observation Buffer**: Mechanism for maintaining temporal history of sensor observations for policy input in dynamic control tasks
- **Reality Gap**: Discrepancy between simulation and real-world robot behavior due to modeling inaccuracies and environmental differences
- **Systematic Parameter Randomization**: Methodical approach to varying simulation parameters within realistic bounds to improve policy generalization

## Domain Randomization for Robust Policy Learning

Domain randomization addresses the reality gap by training policies across diverse simulation conditions, improving their robustness to real-world variations:

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  Randomization  │───►│  Simulation      │───►│  Policy Training │
│  Parameters     │    │  Environment     │    │  Loop           │
│                 │    │                  │    │                  │
│  - Masses       │    │  - Physics       │    │  - Collect       │
│  - Friction     │    │  - Rendering     │    │    trajectories  │
│  - Lighting     │    │  - Sensors       │    │  - Update        │
│  - Textures     │    │                  │    │    policy        │
│  - Dynamics     │    │                  │    │                  │
└─────────────────┘    └──────────────────┘    └──────────────────┘
         │
         ▼
┌─────────────────┐
│  Policy         │
│  Generalization │
│  (Robust to     │
│   Real-World    │
│   Variations)   │
└─────────────────┘
```

### Systematic Parameter Randomization

```python
# domain_randomization.py - Systematic parameter randomization for humanoid simulation
import numpy as np
import random
from typing import Dict, Tuple, Any

class DomainRandomizer:
    def __init__(self):
        # Define randomization ranges for key parameters
        self.randomization_ranges = {
            # Robot dynamics parameters
            'robot_mass': (0.8, 1.2),  # ±20% mass variation
            'friction_coefficient': (0.5, 1.5),  # Variable friction
            'joint_damping': (0.5, 2.0),  # Joint damping variation
            'actuator_delay': (0.0, 0.02),  # 0-20ms actuator delay

            # Environmental parameters
            'gravity': (9.61, 10.01),  # ±0.2 m/s² gravity variation
            'ground_friction': (0.7, 1.3),  # Ground friction variation

            # Sensor noise parameters
            'imu_noise': (0.001, 0.01),  # IMU noise variation
            'encoder_noise': (0.0001, 0.001),  # Encoder noise variation
        }

        # Store current randomization values
        self.current_params = {}
        self.reset_randomization()

    def reset_randomization(self):
        """Reset all parameters to new random values within ranges"""
        for param, (min_val, max_val) in self.randomization_ranges.items():
            self.current_params[param] = random.uniform(min_val, max_val)
        return self.current_params

    def get_randomized_value(self, param_name: str, nominal_value: float) -> float:
        """Get randomized value for a specific parameter"""
        if param_name in self.current_params:
            return nominal_value * self.current_params[param_name]
        else:
            return nominal_value

    def randomize_robot_dynamics(self, robot_model):
        """Apply randomization to robot dynamics properties"""
        # Randomize link masses
        for link in robot_model.links:
            randomized_mass = self.get_randomized_value(
                'robot_mass', link.mass
            )
            link.mass = randomized_mass

            # Randomize friction coefficients
            randomized_friction = self.get_randomized_value(
                'friction_coefficient', link.friction_coeff
            )
            link.friction_coeff = randomized_friction

            # Randomize joint damping
            for joint in link.joints:
                randomized_damping = self.get_randomized_value(
                    'joint_damping', joint.damping
                )
                joint.damping = randomized_damping

    def randomize_environment(self, simulation_env):
        """Apply randomization to environmental parameters"""
        # Randomize gravity
        randomized_gravity = self.get_randomized_value(
            'gravity', simulation_env.gravity
        )
        simulation_env.gravity = randomized_gravity

        # Randomize ground properties
        randomized_ground_friction = self.get_randomized_value(
            'ground_friction', simulation_env.ground_friction
        )
        simulation_env.ground_friction = randomized_ground_friction

        # Randomize lighting conditions (for vision-based tasks)
        if hasattr(simulation_env, 'lighting'):
            simulation_env.lighting.intensity *= random.uniform(0.7, 1.3)
            simulation_env.lighting.ambient_color = np.random.uniform(0.1, 1.0, 3)

    def randomize_sensors(self, robot_sensors):
        """Apply randomization to sensor properties"""
        for sensor in robot_sensors:
            if sensor.type == 'IMU':
                # Add noise to IMU readings
                sensor.noise_level = self.get_randomized_value(
                    'imu_noise', sensor.nominal_noise
                )
            elif sensor.type == 'ENCODER':
                # Add noise to encoder readings
                sensor.noise_level = self.get_randomized_value(
                    'encoder_noise', sensor.nominal_noise
                )

class DomainRandomizationEnv:
    """Environment wrapper that applies domain randomization"""

    def __init__(self, base_env, randomization_freq=1000):
        self.base_env = base_env
        self.randomizer = DomainRandomizer()
        self.randomization_freq = randomization_freq  # Randomize every N episodes
        self.episode_count = 0

    def reset(self):
        """Reset environment with new randomization parameters"""
        if self.episode_count % self.randomization_freq == 0:
            # Apply new randomization parameters
            self.randomizer.reset_randomization()
            self.randomizer.randomize_robot_dynamics(self.base_env.robot_model)
            self.randomizer.randomize_environment(self.base_env.simulation)
            self.randomizer.randomize_sensors(self.base_env.sensors)

        self.episode_count += 1
        return self.base_env.reset()

    def step(self, action):
        """Execute step in randomized environment"""
        return self.base_env.step(action)

    def get_randomization_stats(self):
        """Get current randomization parameter values"""
        return self.randomizer.current_params
```

### Physics Parameter Randomization

```python
# physics_randomization.py - Advanced physics parameter randomization
import numpy as np
from scipy.spatial.transform import Rotation as R

class PhysicsRandomizer:
    """Advanced randomization of physics parameters for humanoid simulation"""

    def __init__(self, robot_model, simulation_params):
        self.robot_model = robot_model
        self.sim_params = simulation_params
        self.param_history = []  # Track parameter variations over time

    def randomize_inertial_properties(self):
        """Randomize inertial properties of robot links"""
        for link in self.robot_model.links:
            # Randomize mass (±20%)
            mass_variation = np.random.uniform(0.8, 1.2)
            link.mass = link.nominal_mass * mass_variation

            # Randomize center of mass offset
            com_offset = np.random.normal(0, 0.01, 3)  # ±1cm offset
            link.center_of_mass += com_offset

            # Randomize inertia tensor with physical constraints
            # Ensure the inertia tensor remains positive definite
            inertia_perturbation = np.random.uniform(0.9, 1.1, 3)
            link.inertia_tensor = np.diag(inertia_perturbation * np.diag(link.nominal_inertia_tensor))

            # Apply random rotation to inertia tensor
            random_rotation = R.random().as_matrix()
            link.inertia_tensor = random_rotation @ link.inertia_tensor @ random_rotation.T

    def randomize_contact_properties(self):
        """Randomize contact and friction properties"""
        for link in self.robot_model.links:
            # Randomize static friction (Coulomb friction model)
            static_friction = np.random.uniform(0.4, 1.2)
            link.contact_properties.static_friction = static_friction

            # Randomize dynamic friction
            dynamic_friction = np.random.uniform(0.3, static_friction)
            link.contact_properties.dynamic_friction = dynamic_friction

            # Randomize restitution (bounciness)
            restitution = np.random.uniform(0.0, 0.1)
            link.contact_properties.restitution = restitution

            # Randomize stiffness and damping for contact models
            contact_stiffness = np.random.uniform(1000, 5000)  # N/m
            contact_damping = np.random.uniform(50, 200)      # Ns/m
            link.contact_properties.stiffness = contact_stiffness
            link.contact_properties.damping = contact_damping

    def randomize_actuator_dynamics(self):
        """Randomize actuator dynamics and response characteristics"""
        for joint in self.robot_model.joints:
            # Randomize motor torque limits
            torque_limit_factor = np.random.uniform(0.8, 1.2)
            joint.torque_limit = joint.nominal_torque_limit * torque_limit_factor

            # Randomize motor dynamics (first-order model)
            time_constant = np.random.uniform(0.005, 0.020)  # 5-20ms response time
            joint.motor_time_constant = time_constant

            # Randomize gear ratio (for variable transmissions)
            gear_ratio_factor = np.random.uniform(0.95, 1.05)  # ±5% variation
            joint.gear_ratio = joint.nominal_gear_ratio * gear_ratio_factor

            # Randomize motor efficiency
            efficiency = np.random.uniform(0.7, 0.95)  # 70-95% efficiency
            joint.motor_efficiency = efficiency

    def randomize_sensor_dynamics(self):
        """Randomize sensor dynamics and noise characteristics"""
        for sensor in self.robot_model.sensors:
            if sensor.type == 'IMU':
                # Randomize IMU noise parameters
                sensor.accelerometer_noise_density = np.random.uniform(1e-4, 5e-4)
                sensor.gyroscope_noise_density = np.random.uniform(1e-5, 5e-5)
                sensor.accelerometer_random_walk = np.random.uniform(1e-6, 5e-6)
                sensor.gyroscope_random_walk = np.random.uniform(1e-7, 5e-7)

                # Randomize bias parameters
                sensor.accelerometer_bias = np.random.normal(0, 1e-3, 3)
                sensor.gyroscope_bias = np.random.normal(0, 1e-4, 3)

            elif sensor.type == 'ENCODER':
                # Randomize encoder resolution and noise
                resolution_factor = np.random.uniform(0.9, 1.1)
                sensor.resolution = int(sensor.nominal_resolution * resolution_factor)

                noise_factor = np.random.uniform(0.5, 2.0)
                sensor.noise_level = sensor.nominal_noise * noise_factor

            elif sensor.type == 'FORCE_TORQUE':
                # Randomize force/torque sensor characteristics
                sensor.force_noise = np.random.uniform(0.1, 1.0)  # N
                sensor.torque_noise = np.random.uniform(0.01, 0.1)  # Nm
                sensor.bias_drift = np.random.normal(0, 0.05, 6)  # N, Nm

    def randomize_environmental_effects(self):
        """Randomize environmental effects like air resistance, etc."""
        # Randomize air density (affects drag)
        air_density = np.random.uniform(1.1, 1.3)  # kg/m³
        self.sim_params.air_density = air_density

        # Randomize wind effects
        wind_velocity = np.random.normal(0, 0.5, 3)  # m/s
        self.sim_params.wind_velocity = wind_velocity
        self.sim_params.wind_gustiness = np.random.uniform(0.0, 0.5)

        # Randomize temperature effects on materials
        temperature = np.random.uniform(15, 35)  # Celsius
        self.sim_params.temperature = temperature
        # Temperature affects material properties, friction, etc.

    def apply_randomization(self):
        """Apply all randomization effects to the simulation"""
        self.randomize_inertial_properties()
        self.randomize_contact_properties()
        self.randomize_actuator_dynamics()
        self.randomize_sensor_dynamics()
        self.randomize_environmental_effects()

        # Store current parameters for analysis
        current_params = {
            'timestamp': self.sim_params.current_time,
            'mass_variation': [link.mass/link.nominal_mass for link in self.robot_model.links],
            'friction_variation': [link.contact_properties.static_friction for link in self.robot_model.links],
            'torque_limit_variation': [joint.torque_limit/joint.nominal_torque_limit for joint in self.robot_model.joints]
        }
        self.param_history.append(current_params)

        return current_params
```

## Multi-Robot Coordination in Simulation

Multi-robot coordination enables complex behaviors through distributed control and communication:

### Coordination Framework

```python
# multi_robot_coordination.py - Multi-robot coordination framework
import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict, Tuple
import asyncio

class CoordinationStrategy(Enum):
    CENTRALIZED = "centralized"
    DECENTRALIZED = "decentralized"
    HIERARCHICAL = "hierarchical"
    MARKET_BASED = "market_based"

@dataclass
class RobotState:
    """State information for a single robot"""
    robot_id: int
    position: np.ndarray  # 3D position [x, y, z]
    orientation: np.ndarray  # Quaternion [w, x, y, z]
    velocity: np.ndarray  # 3D velocity [vx, vy, vz]
    joint_angles: np.ndarray
    joint_velocities: np.ndarray
    battery_level: float
    communication_range: float

@dataclass
class Task:
    """Task definition for multi-robot coordination"""
    task_id: str
    task_type: str  # 'navigation', 'manipulation', 'formation', etc.
    target_position: np.ndarray
    priority: int
    required_robots: int
    estimated_completion_time: float
    dependencies: List[str]  # Task IDs that must complete first

class CommunicationNetwork:
    """Communication network for multi-robot systems"""

    def __init__(self, max_range: float = 10.0):
        self.max_range = max_range
        self.robot_states: Dict[int, RobotState] = {}
        self.topology = {}  # Adjacency list for communication graph

    def add_robot(self, robot_state: RobotState):
        """Add a robot to the communication network"""
        self.robot_states[robot_state.robot_id] = robot_state
        self.topology[robot_state.robot_id] = []

    def update_robot_state(self, robot_id: int, new_state: RobotState):
        """Update state of a specific robot"""
        if robot_id in self.robot_states:
            self.robot_states[robot_id] = new_state
            self._update_topology()

    def _update_topology(self):
        """Update communication topology based on robot positions"""
        for robot_id in self.robot_states:
            neighbors = []
            robot_pos = self.robot_states[robot_id].position

            for other_id, other_state in self.robot_states.items():
                if robot_id != other_id:
                    distance = np.linalg.norm(robot_pos - other_state.position)
                    if distance <= self.max_range:
                        neighbors.append(other_id)

            self.topology[robot_id] = neighbors

    def broadcast_message(self, sender_id: int, message: dict):
        """Broadcast message to all connected robots"""
        if sender_id not in self.topology:
            return []

        recipients = self.topology[sender_id]
        received_by = []

        for recipient_id in recipients:
            # Simulate message delivery with potential delays
            delivery_delay = np.random.uniform(0.01, 0.1)  # 10-100ms delay
            # In real implementation, this would send the message via ROS topics
            received_by.append(recipient_id)

        return received_by

    def get_neighbors(self, robot_id: int) -> List[int]:
        """Get list of communication neighbors for a robot"""
        return self.topology.get(robot_id, [])

class MultiRobotCoordinator:
    """Main coordinator for multi-robot systems"""

    def __init__(self, strategy: CoordinationStrategy):
        self.strategy = strategy
        self.comm_network = CommunicationNetwork()
        self.tasks: Dict[str, Task] = {}
        self.robot_assignments: Dict[str, List[int]] = {}  # task_id -> list of robot_ids
        self.task_queue = []

    def add_robot(self, robot_state: RobotState):
        """Add robot to the coordination system"""
        self.comm_network.add_robot(robot_state)

    def assign_task(self, task: Task, robot_ids: List[int]):
        """Assign a task to specific robots"""
        self.tasks[task.task_id] = task
        self.robot_assignments[task.task_id] = robot_ids

        # Create task assignment messages based on strategy
        if self.strategy == CoordinationStrategy.DECENTRALIZED:
            self._decentralized_assignment(task, robot_ids)
        elif self.strategy == CoordinationStrategy.CENTRALIZED:
            self._centralized_assignment(task, robot_ids)
        elif self.strategy == CoordinationStrategy.HIERARCHICAL:
            self._hierarchical_assignment(task, robot_ids)
        elif self.strategy == CoordinationStrategy.MARKET_BASED:
            self._market_based_assignment(task, robot_ids)

    def _decentralized_assignment(self, task: Task, robot_ids: List[int]):
        """Decentralized task assignment using consensus"""
        # Each robot makes local decisions based on local information
        for robot_id in robot_ids:
            # Robot evaluates if it should take the task based on its state
            robot_state = self.comm_network.robot_states[robot_id]

            # Simple evaluation: distance to target, battery level, current load
            distance_to_target = np.linalg.norm(
                robot_state.position - task.target_position
            )
            fitness_score = (1.0 / (distance_to_target + 1)) * robot_state.battery_level

            # Broadcast fitness score to neighbors
            message = {
                'type': 'fitness_score',
                'task_id': task.task_id,
                'robot_id': robot_id,
                'fitness': fitness_score,
                'timestamp': self._get_timestamp()
            }
            self.comm_network.broadcast_message(robot_id, message)

    def _centralized_assignment(self, task: Task, robot_ids: List[int]):
        """Centralized task assignment with central controller"""
        # Central controller has global knowledge and makes assignment decisions
        central_controller = CentralizedController(self.comm_network)
        assignment = central_controller.compute_optimal_assignment(task, robot_ids)

        # Send assignment commands to robots
        for robot_id in robot_ids:
            command = {
                'type': 'task_assignment',
                'task_id': task.task_id,
                'robot_id': robot_id,
                'assignment': assignment.get(robot_id, {}),
                'timestamp': self._get_timestamp()
            }
            # Send command to robot (in real system, this would be a ROS service call)

    def _hierarchical_assignment(self, task: Task, robot_ids: List[int]):
        """Hierarchical task assignment with team leaders"""
        # Organize robots into teams with leaders
        teams = self._organize_teams(robot_ids)

        for team_id, team_members in teams.items():
            leader_id = team_members[0]  # First robot is leader
            followers = team_members[1:]

            # Leader coordinates task within team
            leader_command = {
                'type': 'team_coordination',
                'task_id': task.task_id,
                'team_id': team_id,
                'leader_id': leader_id,
                'followers': followers,
                'target_position': task.target_position,
                'timestamp': self._get_timestamp()
            }
            self.comm_network.broadcast_message(leader_id, leader_command)

    def _market_based_assignment(self, task: Task, robot_ids: List[int]):
        """Market-based task assignment using auction/bidding"""
        # Robots bid for tasks based on their capabilities and current state
        bids = {}

        for robot_id in robot_ids:
            robot_state = self.comm_network.robot_states[robot_id]

            # Calculate bid value based on robot capabilities
            bid_value = self._calculate_robot_bid(robot_state, task)
            bids[robot_id] = bid_value

            # Broadcast bid to other robots
            bid_message = {
                'type': 'task_bid',
                'task_id': task.task_id,
                'robot_id': robot_id,
                'bid_value': bid_value,
                'timestamp': self._get_timestamp()
            }
            self.comm_network.broadcast_message(robot_id, bid_message)

        # Determine winner based on bids (simple highest bidder wins)
        winner_id = max(bids, key=bids.get)
        winner_command = {
            'type': 'task_award',
            'task_id': task.task_id,
            'winner_id': winner_id,
            'bid_value': bids[winner_id],
            'timestamp': self._get_timestamp()
        }
        # Broadcast award to all robots
        for robot_id in robot_ids:
            self.comm_network.broadcast_message(robot_id, winner_command)

    def _organize_teams(self, robot_ids: List[int]) -> Dict[int, List[int]]:
        """Organize robots into teams for hierarchical coordination"""
        teams = {}
        team_size = 3  # Each team has 3 robots

        for i, robot_id in enumerate(robot_ids):
            team_id = i // team_size
            if team_id not in teams:
                teams[team_id] = []
            teams[team_id].append(robot_id)

        return teams

    def _calculate_robot_bid(self, robot_state: RobotState, task: Task) -> float:
        """Calculate bid value for a robot-task combination"""
        # Calculate cost based on distance, battery, current load
        distance_cost = np.linalg.norm(robot_state.position - task.target_position)
        battery_factor = robot_state.battery_level
        load_factor = 1.0 / (1.0 + len(self._get_robot_tasks(robot_state.robot_id)))

        # Higher bid value means robot wants the task more
        bid_value = (battery_factor * load_factor) / (distance_cost + 1)
        return bid_value

    def _get_robot_tasks(self, robot_id: int) -> List[str]:
        """Get list of tasks assigned to a specific robot"""
        assigned_tasks = []
        for task_id, robot_list in self.robot_assignments.items():
            if robot_id in robot_list:
                assigned_tasks.append(task_id)
        return assigned_tasks

    def _get_timestamp(self) -> float:
        """Get current timestamp"""
        import time
        return time.time()

class CentralizedController:
    """Centralized controller for multi-robot coordination"""

    def __init__(self, comm_network: CommunicationNetwork):
        self.comm_network = comm_network

    def compute_optimal_assignment(self, task: Task, robot_ids: List[int]) -> Dict[int, dict]:
        """Compute optimal task assignment using optimization"""
        # This is a simplified version - in practice, this would use
        # more sophisticated optimization algorithms like Hungarian algorithm,
        # linear programming, or auction algorithms

        assignment = {}

        if task.required_robots <= len(robot_ids):
            # Assign closest robots to minimize travel time
            robot_distances = []
            for robot_id in robot_ids:
                robot_state = self.comm_network.robot_states[robot_id]
                distance = np.linalg.norm(
                    robot_state.position - task.target_position
                )
                robot_distances.append((robot_id, distance))

            # Sort by distance and assign closest robots
            robot_distances.sort(key=lambda x: x[1])
            assigned_robots = [robot_id for robot_id, _ in robot_distances[:task.required_robots]]

            for robot_id in assigned_robots:
                assignment[robot_id] = {
                    'task_id': task.task_id,
                    'role': 'primary' if robot_id == assigned_robots[0] else 'support',
                    'subtask': self._determine_subtask(robot_id, task, assigned_robots)
                }

        return assignment

    def _determine_subtask(self, robot_id: int, task: Task, assigned_robots: List[int]) -> str:
        """Determine specific subtask for a robot in a multi-robot task"""
        if task.task_type == 'formation':
            # Assign formation positions based on robot index
            robot_idx = assigned_robots.index(robot_id)
            return f'formation_position_{robot_idx}'
        elif task.task_type == 'navigation':
            # Assign navigation roles (leader, follower, scout)
            if robot_idx == 0:
                return 'leader'
            elif robot_idx == len(assigned_robots) - 1:
                return 'scout'
            else:
                return 'follower'
        else:
            return 'general'
```

## Sim-to-Real Transfer Framework

The RL-SAR framework provides a practical approach to sim-to-real transfer:

### Policy Deployment System

```cpp
// rl_sar_integration.cpp - C++ integration for sim-to-real transfer
#include <vector>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>
#include <torch/torch.h>
#include <onnxruntime_cxx_api.h>

// Model factory for different inference backends
class ModelFactory {
public:
    enum ModelType {
        TORCH,
        ONNX,
        TENSORRT
    };

    static std::unique_ptr<InferenceModel> create_model(ModelType type) {
        switch(type) {
            case TORCH:
                return std::make_unique<TorchModel>();
            case ONNX:
                return std::make_unique<ONNXModel>();
            case TENSORRT:
                return std::make_unique<TensorRTModel>();
            default:
                return std::make_unique<TorchModel>();
        }
    }
};

class InferenceModel {
public:
    virtual bool load(const std::string& model_path) = 0;
    virtual std::vector<float> forward(const std::vector<std::vector<float>>& observations) = 0;
    virtual ~InferenceModel() = default;
};

class TorchModel : public InferenceModel {
private:
    torch::jit::script::Module module;
    bool loaded = false;

public:
    bool load(const std::string& model_path) override {
        try {
            module = torch::jit::load(model_path);
            loaded = true;
            return true;
        } catch (const c10::Error& e) {
            std::cerr << "Error loading Torch model: " << e.msg() << std::endl;
            loaded = false;
            return false;
        }
    }

    std::vector<float> forward(const std::vector<std::vector<float>>& observations) override {
        if (!loaded) return {};

        // Convert observations to tensor
        std::vector<torch::jit::IValue> inputs;
        for (const auto& obs : observations) {
            auto tensor = torch::from_blob(
                const_cast<float*>(obs.data()),
                {1, static_cast<long>(obs.size())},
                torch::kFloat
            ).clone();
            inputs.push_back(tensor);
        }

        // Run inference
        at::Tensor output = module.forward(inputs).toTensor();
        auto output_acc = output.accessor<float, 2>();

        std::vector<float> result;
        for (int i = 0; i < output_acc.size(1); ++i) {
            result.push_back(output_acc[0][i]);
        }

        return result;
    }
};

class ONNXModel : public InferenceModel {
private:
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "ONNXModel"};
    Ort::Session *session = nullptr;
    Ort::AllocatorWithDefaultOptions allocator;
    bool loaded = false;

public:
    bool load(const std::string& model_path) override {
        try {
            Ort::SessionOptions session_options;
            session_options.SetIntraOpNumThreads(1);
            session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

            session = new Ort::Session(env, model_path.c_str(), session_options);
            loaded = true;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error loading ONNX model: " << e.what() << std::endl;
            loaded = false;
            return false;
        }
    }

    std::vector<float> forward(const std::vector<std::vector<float>>& observations) override {
        if (!loaded || !session) return {};

        // Prepare input tensor
        std::vector<int64_t> input_shape{1, static_cast<int64_t>(observations[0].size())};
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            const_cast<float*>(observations[0].data()),
            observations[0].size(),
            input_shape.data(),
            input_shape.size()
        );

        // Run inference
        char* input_names[] = {"input"};
        char* output_names[] = {"output"};

        auto output_tensors = session->Run(
            Ort::RunOptions{nullptr},
            input_names,
            &input_tensor,
            1,
            output_names,
            1
        );

        // Extract output
        float* floatarr = output_tensors[0].GetTensorMutableData<float>();
        auto type_info = output_tensors[0].GetTensorTypeAndShapeInfo();
        std::vector<int64_t> output_shape = type_info.GetShape();

        std::vector<float> result;
        int output_size = 1;
        for (auto dim : output_shape) {
            output_size *= dim;
        }

        for (int i = 0; i < output_size; ++i) {
            result.push_back(floatarr[i]);
        }

        return result;
    }
};

// Observation buffer for maintaining temporal history
class ObservationBuffer {
private:
    int num_envs;
    std::vector<int> obs_dims;
    int history_length;
    std::string concat_order;  // "time" or "channel"
    std::vector<std::vector<std::vector<float>>> buffer;  // [env][time][obs_dim]
    std::vector<int> current_indices;

public:
    ObservationBuffer(int n_envs, const std::vector<int>& dims, int hist_len, const std::string& order = "time")
        : num_envs(n_envs), obs_dims(dims), history_length(hist_len), concat_order(order) {

        buffer.resize(num_envs);
        current_indices.resize(num_envs, 0);

        for (int i = 0; i < num_envs; ++i) {
            buffer[i].resize(history_length);
            int total_dims = 0;
            for (int dim : obs_dims) {
                total_dims += dim;
            }
            for (int j = 0; j < history_length; ++j) {
                buffer[i][j].resize(total_dims, 0.0f);
            }
        }
    }

    void reset(const std::vector<int>& env_ids, const std::vector<float>& init_obs) {
        for (int env_id : env_ids) {
            for (int t = 0; t < history_length; ++t) {
                buffer[env_id][t] = init_obs;
            }
            current_indices[env_id] = 0;
        }
    }

    void insert(const std::vector<float>& obs) {
        for (int env = 0; env < num_envs; ++env) {
            buffer[env][current_indices[env]] = obs;
            current_indices[env] = (current_indices[env] + 1) % history_length;
        }
    }

    std::vector<float> get_obs_vec(const std::vector<int>& time_indices) {
        std::vector<float> result;

        if (concat_order == "time") {
            // Concatenate along time dimension: [obs_t-2, obs_t-1, obs_t]
            for (int time_idx : time_indices) {
                int buffer_idx = (current_indices[0] + time_idx) % history_length;
                result.insert(result.end(),
                             buffer[0][buffer_idx].begin(),
                             buffer[0][buffer_idx].end());
            }
        } else {
            // Concatenate along channel dimension
            for (int env = 0; env < num_envs; ++env) {
                for (int time_idx : time_indices) {
                    int buffer_idx = (current_indices[env] + time_idx) % history_length;
                    result.insert(result.end(),
                                 buffer[env][buffer_idx].begin(),
                                 buffer[env][buffer_idx].end());
                }
            }
        }

        return result;
    }
};

// Real robot interface for Unitree robots
class RealRobotInterface {
protected:
    std::string network_interface;
    bool initialized = false;

public:
    RealRobotInterface(const std::string& net_interface) : network_interface(net_interface) {}

    virtual bool initialize() = 0;
    virtual bool send_command(const std::vector<float>& actions) = 0;
    virtual std::vector<float> get_sensor_data() = 0;
    virtual bool shutdown() = 0;
};

// Example implementation for Unitree Go2 robot
class Go2RobotInterface : public RealRobotInterface {
private:
    void* sdk_handle = nullptr;  // Unitree SDK handle
    std::vector<float> last_action;

public:
    Go2RobotInterface(const std::string& net_interface) : RealRobotInterface(net_interface) {}

    bool initialize() override {
        // Initialize Unitree SDK
        // This would involve setting up network connection, calibration, etc.
        // For this example, we'll simulate the initialization

        std::cout << "Initializing Unitree Go2 robot on interface: " << network_interface << std::endl;

        // Initialize SDK
        // unitree_sdk2::init();

        initialized = true;
        return initialized;
    }

    bool send_command(const std::vector<float>& actions) override {
        if (!initialized) return false;

        // Send actions to robot motors
        // In real implementation, this would convert actions to motor commands
        // and send them via the Unitree SDK

        last_action = actions;

        // Example: Convert actions to joint positions/torques
        // for (size_t i = 0; i < actions.size() && i < joint_commands.size(); ++i) {
        //     joint_commands[i].position = actions[i];
        //     joint_commands[i].kp = 0.0;  // Use learned gains if available
        //     joint_commands[i].kd = 0.0;
        // }

        // Send commands via SDK
        // unitree_sdk2::send_joint_commands(joint_commands);

        return true;
    }

    std::vector<float> get_sensor_data() override {
        if (!initialized) return {};

        // Get sensor data from robot
        // In real implementation, this would read from IMU, encoders, etc.

        std::vector<float> sensor_data(45);  // Example: 45-dim observation space

        // Simulate getting sensor data
        // This would typically include:
        // - Joint positions (12 values for Go2)
        // - Joint velocities (12 values)
        // - IMU data (6 values: angular velocity + linear acceleration)
        // - Gravity vector (3 values)
        // - Command history (previous actions)

        // Fill with simulated data
        for (size_t i = 0; i < sensor_data.size(); ++i) {
            sensor_data[i] = static_cast<float>(rand()) / RAND_MAX * 2.0f - 1.0f;  // Random values in [-1, 1]
        }

        return sensor_data;
    }

    bool shutdown() override {
        if (!initialized) return true;

        // Stop robot motors and clean up
        // Send zero commands to all joints
        std::vector<float> zero_commands(12, 0.0f);
        send_command(zero_commands);

        // unitree_sdk2::cleanup();
        initialized = false;

        return true;
    }
};

// Main control loop for sim-to-real transfer
class Sim2RealController {
private:
    std::unique_ptr<InferenceModel> policy_model;
    std::unique_ptr<ObservationBuffer> obs_buffer;
    std::unique_ptr<RealRobotInterface> robot_interface;
    std::unique_ptr<DomainRandomizer> domain_randomizer;

public:
    Sim2RealController(const std::string& model_path, const std::string& robot_type,
                      const std::string& network_interface) {

        // Initialize policy model
        policy_model = ModelFactory::create_model(ModelFactory::ONNX);
        policy_model->load(model_path);

        // Initialize observation buffer (example parameters)
        std::vector<int> obs_dims = {3, 3, 3, 12, 12, 12};  // commands, ang_vel, gravity, pos, vel, actions
        obs_buffer = std::make_unique<ObservationBuffer>(1, obs_dims, 6, "time");

        // Initialize robot interface
        if (robot_type == "go2") {
            robot_interface = std::make_unique<Go2RobotInterface>(network_interface);
        }
        // Add other robot types as needed

        // Initialize domain randomizer for robustness
        domain_randomizer = std::make_unique<DomainRandomizer>();
    }

    bool run_control_loop() {
        if (!robot_interface->initialize()) {
            std::cerr << "Failed to initialize robot interface" << std::endl;
            return false;
        }

        std::cout << "Starting control loop..." << std::endl;

        for (int step = 0; step < 100000; ++step) {  // Run for 100k steps or until stopped
            // Get sensor data from robot
            auto sensor_data = robot_interface->get_sensor_data();
            if (sensor_data.empty()) {
                std::cerr << "Failed to get sensor data, stopping control loop" << std::endl;
                break;
            }

            // Insert observation into buffer
            obs_buffer->insert(sensor_data);

            // Get historical observations for policy input
            std::vector<int> history_indices = {0, 1, 2, 3, 4, 5};  // Last 6 timesteps
            auto policy_input = obs_buffer->get_obs_vec(history_indices);

            // Run policy inference
            auto actions = policy_model->forward({policy_input});

            // Apply domain randomization to actions for robustness
            auto randomized_actions = apply_action_randomization(actions);

            // Send commands to robot
            if (!robot_interface->send_command(randomized_actions)) {
                std::cerr << "Failed to send commands to robot, stopping control loop" << std::endl;
                break;
            }

            // Small delay to control loop frequency
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz control
        }

        robot_interface->shutdown();
        return true;
    }

private:
    std::vector<float> apply_action_randomization(const std::vector<float>& actions) {
        // Apply small randomization to actions for robustness
        std::vector<float> randomized_actions = actions;

        for (auto& action : randomized_actions) {
            // Add small noise (±2% of action range)
            float noise = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.04f;
            action += noise;

            // Clamp to valid range [-1, 1]
            action = std::max(-1.0f, std::min(1.0f, action));
        }

        return randomized_actions;
    }
};
```

### Python Interface for Sim-to-Real Transfer

```python
# sim2real_interface.py - Python interface for sim-to-real transfer
import subprocess
import sys
import os
from pathlib import Path
import numpy as np
import yaml
import onnxruntime as ort
import torch
import time

class PolicyConverter:
    """Convert policies between different formats for deployment"""

    @staticmethod
    def torch_to_onnx(torch_model_path: str, onnx_model_path: str, input_shape: tuple):
        """Convert PyTorch model to ONNX format"""
        try:
            # Load PyTorch model
            model = torch.jit.load(torch_model_path)
            model.eval()

            # Create dummy input
            dummy_input = torch.randn(input_shape)

            # Export to ONNX
            torch.onnx.export(
                model,
                dummy_input,
                onnx_model_path,
                export_params=True,
                opset_version=11,
                do_constant_folding=True,
                input_names=['input'],
                output_names=['output'],
                dynamic_axes={
                    'input': {0: 'batch_size'},
                    'output': {0: 'batch_size'}
                }
            )

            print(f"Successfully converted {torch_model_path} to {onnx_model_path}")

            # Verify conversion
            PolicyConverter.verify_conversion(torch_model_path, onnx_model_path, input_shape)

            return True
        except Exception as e:
            print(f"Error converting model: {e}")
            return False

    @staticmethod
    def onnx_to_torch(onnx_model_path: str, torch_model_path: str):
        """Convert ONNX model back to PyTorch using onnx2torch"""
        try:
            import onnx2torch
            import onnx

            # Load ONNX model
            onnx_model = onnx.load(onnx_model_path)

            # Convert to PyTorch
            torch_model = onnx2torch.convert(onnx_model)

            # Save as TorchScript
            torch.jit.save(torch.jit.script(torch_model), torch_model_path)

            print(f"Successfully converted {onnx_model_path} to {torch_model_path}")

            # Verify conversion
            PolicyConverter.verify_onnx_torch_conversion(onnx_model_path, torch_model_path)

            return True
        except ImportError:
            print("onnx2torch not installed. Install with: pip install onnx2torch")
            return False
        except Exception as e:
            print(f"Error converting ONNX to PyTorch: {e}")
            return False

    @staticmethod
    def verify_conversion(torch_model_path: str, onnx_model_path: str, input_shape: tuple):
        """Verify that PyTorch and ONNX models produce similar outputs"""
        # Load models
        torch_model = torch.jit.load(torch_model_path)
        torch_model.eval()

        ort_session = ort.InferenceSession(onnx_model_path)

        # Create test input
        test_input = torch.randn(input_shape)
        numpy_input = test_input.detach().cpu().numpy()

        # Run inference
        with torch.no_grad():
            torch_output = torch_model(test_input).detach().cpu().numpy()

        onnx_output = ort_session.run(None, {'input': numpy_input})[0]

        # Compare outputs
        max_diff = np.max(np.abs(torch_output - onnx_output))

        if max_diff < 1e-5:
            print(f"✓ Conversion verified successfully. Max difference: {max_diff:.2e}")
        else:
            print(f"✗ Conversion verification failed. Max difference: {max_diff:.2e}")

    @staticmethod
    def verify_onnx_torch_conversion(onnx_model_path: str, torch_model_path: str):
        """Verify ONNX to PyTorch conversion"""
        # Load models
        torch_model = torch.jit.load(torch_model_path)
        torch_model.eval()

        ort_session = ort.InferenceSession(onnx_model_path)

        # Create test input
        input_shape = (1, 45)  # Example: 45-dim input for humanoid
        test_input = torch.randn(input_shape)
        numpy_input = test_input.detach().cpu().numpy()

        # Run inference
        with torch.no_grad():
            torch_output = torch_model(test_input).detach().cpu().numpy()

        onnx_output = ort_session.run(None, {'input': numpy_input})[0]

        # Compare outputs
        max_diff = np.max(np.abs(torch_output - onnx_output))

        if max_diff < 1e-4:
            print(f"✓ ONNX->PyTorch conversion verified. Max difference: {max_diff:.2e}")
        else:
            print(f"✗ ONNX->PyTorch conversion failed. Max difference: {max_diff:.2e}")

class RealRobotController:
    """Controller for deploying policies to real robots"""

    def __init__(self, robot_type: str, network_interface: str, policy_path: str):
        self.robot_type = robot_type
        self.network_interface = network_interface
        self.policy_path = policy_path
        self.ros_workspace = Path.home() / "rl_sar"  # Default RL-SAR workspace

    def deploy_policy(self):
        """Deploy policy to real robot"""
        print(f"Deploying policy to {self.robot_type} robot...")

        # Check if ROS workspace exists
        if not self.ros_workspace.exists():
            print(f"ROS workspace not found at {self.ros_workspace}")
            return False

        # Verify policy file exists
        policy_file = Path(self.policy_path)
        if not policy_file.exists():
            print(f"Policy file not found: {self.policy_path}")
            return False

        # Determine executable based on robot type
        if self.robot_type.lower() == "a1":
            executable = "rl_real_a1"
        elif self.robot_type.lower() == "go2":
            executable = "rl_real_go2"
        elif self.robot_type.lower() == "g1":
            executable = "rl_real_g1"
        else:
            print(f"Unsupported robot type: {self.robot_type}")
            return False

        # Check if policy is in ONNX format (preferred for deployment)
        if policy_file.suffix.lower() == '.pt':
            print("Converting PyTorch policy to ONNX for deployment...")
            onnx_path = policy_file.with_suffix('.onnx')
            if PolicyConverter.torch_to_onnx(str(policy_file), str(onnx_path), (1, 45)):
                self.policy_path = str(onnx_path)
            else:
                print("Failed to convert policy to ONNX, continuing with PyTorch...")

        # Prepare deployment command
        if self.robot_type.lower() == "g1":
            # G1 requires network interface
            command = [
                "ros2", "run", "rl_sar", executable,
                self.network_interface
            ]
        elif self.robot_type.lower() == "go2":
            # Go2 can have optional 'wheel' argument
            command = [
                "ros2", "run", "rl_sar", executable,
                self.network_interface
            ]
        else:
            # A1 and other robots
            command = [
                "ros2", "run", "rl_sar", executable
            ]

        print(f"Executing command: {' '.join(command)}")

        try:
            # Change to workspace directory
            original_cwd = os.getcwd()
            os.chdir(self.ros_workspace)

            # Source ROS environment
            # In practice, you'd want to source the workspace setup file
            # This is just a simulation of the process
            print(f"Changed to workspace: {self.ros_workspace}")
            print(f"Deployment command prepared: {' '.join(command)}")
            print("In a real deployment, this would execute the robot control program.")

            # For simulation purposes, just return success
            return True

        except Exception as e:
            print(f"Error during deployment: {e}")
            return False
        finally:
            os.chdir(original_cwd)

    def setup_onboard_deployment(self):
        """Setup for onboard deployment on robot's Jetson computer"""
        print("Setting up onboard deployment on robot's Jetson computer...")

        setup_steps = [
            "SSH into robot's onboard computer",
            "Clone RL-SAR repository",
            "Build with CMake",
            "Configure systemd service for auto-start",
            "Test deployment"
        ]

        for i, step in enumerate(setup_steps, 1):
            print(f"{i}. {step}")

        # Example commands for Jetson deployment
        jetson_commands = [
            "ssh unitree@192.168.123.18  # Password: 123",
            "git clone --recursive --depth 1 https://github.com/fan-ziqi/rl_sar.git",
            "cd rl_sar && ./build.sh -m",
            "./cmake_build/bin/rl_real_go2 eth0",
            "Setup systemd service for auto-start"
        ]

        print("\nExample commands for Jetson deployment:")
        for cmd in jetson_commands:
            print(f"  $ {cmd}")

        return True

def main():
    """Main function for sim-to-real deployment"""
    print("RL-SAR: Reinforcement Learning Simulation to Real Deployment Tool")
    print("=" * 60)

    # Example usage
    controller = RealRobotController(
        robot_type="go2",
        network_interface="eth0",
        policy_path="policy/go2/himloco/himloco.onnx"
    )

    # Deploy policy
    success = controller.deploy_policy()

    if success:
        print("\n✓ Policy deployment completed successfully!")
        print("The robot should now be executing the learned policy.")
    else:
        print("\n✗ Policy deployment failed!")
        print("Check the error messages above and try again.")

if __name__ == "__main__":
    main()
```

## Summary

Advanced simulation techniques for humanoid robotics encompass domain randomization, multi-robot coordination, and sim-to-real transfer methodologies that bridge the gap between virtual and physical environments. These techniques enable robust policy learning, distributed control of multiple robots, and deployment of simulation-trained behaviors to real hardware platforms.

Key capabilities include:
- **Domain Randomization**: Systematic parameter variation during training to improve policy robustness
- **Multi-Robot Coordination**: Frameworks for distributed control using centralized, decentralized, hierarchical, or market-based strategies
- **Sim-to-Real Transfer**: Deployment of reinforcement learning policies from simulation to hardware using frameworks like RL-SAR

In Chapter 9, we will transition to Module 3: Edge Computing and Embedded Systems, beginning with real-time control systems and embedded hardware architectures for humanoid robotics applications.

## Review Questions

1. **Conceptual**: Explain how domain randomization addresses the "reality gap" in sim-to-real transfer. What are the trade-offs between randomization range and policy performance?

2. **Applied**: Implement a decentralized coordination algorithm for 3 humanoid robots performing a formation task, including communication protocols and collision avoidance.

3. **Structural**: Compare the computational requirements and latency constraints for deploying reinforcement learning policies on embedded systems versus cloud-based systems for humanoid control.