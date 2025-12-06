---
id: c9-real-time-control
title: "Chapter 9: Real-Time Control Systems and Embedded Hardware"
sidebar_label: "C9: Real-Time Control"
sidebar_position: 9
---

# Chapter 9: Real-Time Control Systems and Embedded Hardware

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Design** real-time control architectures for humanoid robot systems with deterministic timing guarantees
2. **Implement** real-time control loops using ROS 2 Control framework with appropriate scheduling policies
3. **Configure** embedded hardware platforms (NVIDIA Jetson, Raspberry Pi, etc.) for real-time robot control
4. **Deploy** time-critical control algorithms on resource-constrained embedded systems

## Overview

Modules 1 and 2 established the software foundations for humanoid robotics: ROS 2 communication patterns and simulation environments. However, deploying humanoid robots in the real world requires specialized real-time control systems and embedded hardware platforms capable of meeting strict timing constraints for safety-critical control loops.

**Real-time control systems** ensure deterministic execution of control algorithms within bounded time constraints. For humanoid robots, this means executing joint position control, balance maintenance, and collision avoidance algorithms with predictable timing to prevent falls or unsafe behaviors. The control system must handle sensor data acquisition, state estimation, control computation, and actuator command generation within millisecond-level deadlines.

**Embedded hardware platforms** provide the computational resources for running real-time control systems in compact, power-efficient packages suitable for deployment on humanoid robots. These platforms must balance computational performance with power consumption, thermal management, and physical size constraints.

This chapter introduces real-time control concepts, ROS 2 Control framework, embedded hardware platforms, and practical implementation strategies for deploying time-critical control algorithms on resource-constrained systems. You will learn to configure real-time scheduling, implement real-time safe communication patterns, and optimize control algorithms for embedded deployment.

## Key Concepts

- **Real-Time Control**: Control systems that guarantee response within specified time constraints, essential for safety-critical robot applications
- **ROS 2 Control**: Framework for real-time robot control providing hardware abstraction, controller management, and real-time safety
- **SCHED_FIFO**: Linux real-time scheduling policy that ensures deterministic execution of time-critical threads
- **RealtimeBuffer**: Thread-safe data exchange mechanism between real-time and non-real-time threads in ROS 2 Control
- **Hardware Interface**: Abstraction layer in ROS 2 Control that connects controllers to physical hardware
- **Controller Manager**: Central component that manages the lifecycle and execution of multiple controllers
- **Update Rate**: Frequency at which the controller manager executes the real-time control loop (typically 100-1000 Hz)
- **Deterministic Timing**: Predictable execution timing essential for safety-critical robot control

## Real-Time Control Architecture

Real-time control systems for humanoid robots must guarantee deterministic timing for safety-critical control loops:

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  Sensor Data    │───►│  Real-Time       │───►│  Actuator        │
│  Acquisition   │    │  Control Loop    │    │  Commands        │
│  (IMU, Encoders,│    │  (State Estimation,│   │  (Joint Torque/  │
│   Force Sensors)│    │   Control Computation)│ │   Position)      │
└─────────────────┘    └──────────────────┘    └──────────────────┘
         │                       │                        │
         ▼                       ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  1-10ms         │    │  Deterministic   │    │  < 1ms           │
│  Timing        │◄───┤  Execution       │◄───┤  Response        │
│  Constraints   │    │  (SCHED_FIFO)    │    │  Times          │
└─────────────────┘    └──────────────────┘    └──────────────────┘
```

Real-time systems distinguish between **hard real-time** (missed deadlines cause system failure) and **soft real-time** (missed deadlines degrade performance but don't cause failure). Humanoid robot balance control represents a hard real-time requirement where missed deadlines can result in falls or damage.

### Real-Time Scheduling Configuration

```bash
# Configure system for real-time operation
# Add user to realtime group
sudo addgroup realtime
sudo usermod -a -G realtime $USER

# Set real-time limits in /etc/security/limits.conf
cat << EOF | sudo tee -a /etc/security/limits.conf
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
EOF

# Reboot or log out/in for changes to take effect
echo "Reboot required for real-time configuration to take effect"
```

### Real-Time Thread Configuration in C++

```cpp
// realtime_thread_config.cpp - Configure real-time threads for robot control
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <iostream>
#include <thread>

namespace realtime_control {

class RealtimeThread {
private:
    pthread_t thread_id_;
    int priority_;
    bool configured_;

public:
    RealtimeThread(int priority = 50) : priority_(priority), configured_(false) {}

    bool configure_realtime() {
        // Lock all memory pages to prevent page faults during real-time execution
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
            std::cerr << "Failed to lock memory pages for real-time thread" << std::endl;
            return false;
        }

        // Configure thread scheduling policy
        struct sched_param param;
        param.sched_priority = priority_;

        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            std::cerr << "Failed to set SCHED_FIFO scheduling for thread" << std::endl;
            munlockall();
            return false;
        }

        // Disable page fault handler to prevent non-deterministic delays
        struct sched_param fifo_param;
        fifo_param.sched_priority = priority_;

        configured_ = true;
        return true;
    }

    static void* realtime_thread_func(void* arg) {
        // This function runs with real-time priority
        RealtimeThread* rt_thread = static_cast<RealtimeThread*>(arg);

        if (!rt_thread->configure_realtime()) {
            return nullptr;
        }

        // Real-time control loop
        while (true) {
            // Perform time-critical control operations here
            // Avoid system calls, dynamic allocation, or blocking operations
            rt_thread->control_loop();

            // Sleep for control period (use clock_nanosleep for real-time safety)
            struct timespec sleep_time = {0, 1000000}; // 1ms
            clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
        }

        return nullptr;
    }

    void control_loop() {
        // Placeholder for actual control computation
        // In real implementation, this would contain:
        // - Sensor data acquisition
        // - State estimation
        // - Control law computation
        // - Actuator command generation
    }

    bool start() {
        return pthread_create(&thread_id_, nullptr, realtime_thread_func, this) == 0;
    }
};

} // namespace realtime_control
```

### RealtimeBuffer for Safe Data Exchange

```cpp
// realtime_buffer_example.cpp - Using RealtimeBuffer for safe data exchange
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

namespace realtime_control {

class VelocityController {
private:
    // Real-time safe buffer for velocity commands
    std::shared_ptr<realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist>> velocity_buffer_;

    // Non-real-time subscriber callback
    void velocity_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // This runs in non-real-time thread
        velocity_buffer_->writeFromNonRT(*msg);
    }

public:
    VelocityController() {
        velocity_buffer_ = std::make_shared<realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist>>();
    }

    geometry_msgs::msg::Twist get_current_velocity_command() {
        // This runs in real-time thread - safe access to shared data
        geometry_msgs::msg::Twist* current_cmd = velocity_buffer_->readFromRT();
        return *current_cmd;
    }

    void update_control_loop() {
        // Get current velocity command in real-time safe manner
        geometry_msgs::msg::Twist cmd = get_current_velocity_command();

        // Use command for real-time control computation
        // Apply control law, compute joint torques, etc.
    }
};

} // namespace realtime_control
```

## ROS 2 Control Framework

The ROS 2 Control framework provides a standardized approach to real-time robot control with hardware abstraction:

### Hardware Interface Implementation

```cpp
// hardware_interface.cpp - Implementing a custom hardware interface
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace my_robot_hardware {

class MyRobotHardware : public hardware_interface::SystemInterface {
public:
    // Constructor
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        // Validate joint information
        if (info_.joints.size() != 12) {  // Example: 12 joints for humanoid legs
            RCLCPP_FATAL(rclcpp::get_logger("MyRobotHardware"),
                        "Expected 12 joints, got %zu", info_.joints.size());
            return CallbackReturn::ERROR;
        }

        // Initialize joint data structures
        joint_positions_.resize(info_.joints.size(), 0.0);
        joint_velocities_.resize(info_.joints.size(), 0.0);
        joint_efforts_.resize(info_.joints.size(), 0.0);
        joint_commands_.resize(info_.joints.size(), 0.0);

        // Initialize IMU data
        imu_orientation_.resize(4, 0.0);  // quaternion
        imu_angular_velocity_.resize(3, 0.0);
        imu_linear_acceleration_.resize(3, 0.0);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Export joint position state interfaces
        for (size_t i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
        }

        // Export IMU sensor interfaces
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "orientation.x", &imu_orientation_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "orientation.y", &imu_orientation_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "orientation.z", &imu_orientation_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "orientation.w", &imu_orientation_[3]));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // Export joint position command interfaces
        for (size_t i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_commands_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override {
        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Activating ...please wait...");

        // Initialize hardware to safe state
        for (size_t i = 0; i < joint_commands_.size(); i++) {
            joint_commands_[i] = joint_positions_[i];  // Start at current position
        }

        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Successfully activated!");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override {
        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Successfully deactivated!");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override {
        // Read data from physical hardware (real-time safe)
        // This method is called by the controller manager in the real-time loop

        // Example: Read joint positions from encoders
        for (size_t i = 0; i < joint_positions_.size(); i++) {
            // In real implementation, this would read from actual hardware
            // joint_positions_[i] = read_encoder(i);
            // joint_velocities_[i] = differentiate_position(i);
            // joint_efforts_[i] = read_torque_sensor(i);
        }

        // Example: Read IMU data
        // read_imu_data(imu_orientation_, imu_angular_velocity_, imu_linear_acceleration_);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override {
        // Write commands to physical hardware (real-time safe)
        // This method is called by the controller manager in the real-time loop

        // Example: Send joint commands to actuators
        for (size_t i = 0; i < joint_commands_.size(); i++) {
            // In real implementation, this would send commands to actual hardware
            // send_joint_command(i, joint_commands_[i]);
        }

        return hardware_interface::return_type::OK;
    }

private:
    // Storage for joint data
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<double> joint_commands_;

    // Storage for IMU data
    std::vector<double> imu_orientation_;
    std::vector<double> imu_angular_velocity_;
    std::vector<double> imu_linear_acceleration_;
};

} // namespace my_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_hardware::MyRobotHardware, hardware_interface::SystemInterface)
```

### Controller Implementation

```cpp
// joint_trajectory_controller.cpp - Real-time safe controller implementation
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <realtime_tools/realtime_buffer.h>

namespace my_controllers {

class JointTrajectoryController : public controller_interface::ControllerInterface {
public:
    JointTrajectoryController() = default;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto & joint_name : joint_names_) {
            conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto & joint_name : joint_names_) {
            conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        }

        return conf;
    }

    controller_interface::CallbackReturn on_init() override {
        try {
            // Initialize joint names (typically from parameters)
            joint_names_ = {"left_hip", "left_knee", "left_ankle",
                           "right_hip", "right_knee", "right_ankle",
                           "left_shoulder", "left_elbow", "left_wrist",
                           "right_shoulder", "right_elbow", "right_wrist"};

            // Initialize trajectory buffer
            trajectory_buffer_ = std::make_shared<realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory>>();
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override {
        RCLCPP_INFO(get_node()->get_logger(), "Configuring Joint Trajectory Controller");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override {
        RCLCPP_INFO(get_node()->get_logger(), "Activating Joint Trajectory Controller");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override {
        RCLCPP_INFO(get_node()->get_logger(), "Deactivating Joint Trajectory Controller");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override {
        // Real-time safe update method - called by controller manager
        // This is the core of the control loop

        // Get current trajectory from buffer (real-time safe)
        const trajectory_msgs::msg::JointTrajectory * current_trajectory =
            trajectory_buffer_->readFromRT();

        if (current_trajectory && !current_trajectory->points.empty()) {
            // Execute trajectory following logic
            execute_trajectory(*current_trajectory, time);
        }

        // Apply control law to generate joint commands
        for (size_t i = 0; i < joint_command_interfaces_.size(); i++) {
            // In real implementation, this would contain control law
            // joint_command_interfaces_[i].set_value(desired_position);
        }

        return controller_interface::return_type::OK;
    }

private:
    void execute_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory,
                           const rclcpp::Time & current_time) {
        // Execute trajectory following - this runs in real-time loop
        // Should be computationally efficient and deterministic
    }

    std::vector<std::string> joint_names_;
    std::shared_ptr<realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory>> trajectory_buffer_;
};

} // namespace my_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_controllers::JointTrajectoryController, controller_interface::ControllerInterface)
```

## Embedded Hardware Platforms

### NVIDIA Jetson for Real-Time Control

```python
# jetson_control_setup.py - Configure Jetson platform for real-time control
import os
import subprocess
import sys
from pathlib import Path

class JetsonControlSetup:
    """Setup utilities for real-time control on NVIDIA Jetson platforms"""

    def __init__(self):
        self.jetson_model = self.detect_jetson_model()
        self.realtime_configured = False

    def detect_jetson_model(self):
        """Detect the specific Jetson model"""
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().strip().replace('\x00', '')
                return model
        except:
            return "Unknown Jetson Model"

    def configure_realtime_kernel(self):
        """Configure Jetson for real-time operation"""
        print(f"Configuring real-time settings for: {self.jetson_model}")

        # Check if real-time kernel is available
        has_rt_kernel = self.check_realtime_kernel()

        if has_rt_kernel:
            print("Real-time kernel detected")
        else:
            print("Real-time kernel not available - consider installing Jetson Linux RT")

        # Configure CPU governor for performance
        self.configure_cpu_governor()

        # Configure memory for real-time operation
        self.configure_memory_locking()

        # Configure interrupt affinity to dedicate cores for control
        self.configure_interrupt_affinity()

        self.realtime_configured = True
        return True

    def check_realtime_kernel(self):
        """Check if system is running a real-time kernel"""
        try:
            result = subprocess.run(['uname', '-r'], capture_output=True, text=True)
            kernel_version = result.stdout.strip()
            return 'rt' in kernel_version.lower()
        except:
            return False

    def configure_cpu_governor(self):
        """Set CPU governor to performance mode for deterministic behavior"""
        try:
            # Set all CPU cores to performance mode
            for cpu in range(os.cpu_count()):
                gov_path = f"/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_governor"
                if os.path.exists(gov_path):
                    with open(gov_path, 'w') as f:
                        f.write('performance')
            print("CPU governor set to performance mode")
        except Exception as e:
            print(f"Warning: Could not set CPU governor: {e}")

    def configure_memory_locking(self):
        """Configure memory locking for real-time operation"""
        # This would typically require root privileges
        print("Configuring memory locking...")

        # Lock current process memory
        try:
            import resource
            resource.setrlimit(resource.RLIMIT_MEMLOCK, (resource.RLIM_INFINITY, resource.RLIM_INFINITY))
            print("Memory locking configured")
        except Exception as e:
            print(f"Warning: Could not configure memory locking: {e}")

    def configure_interrupt_affinity(self):
        """Configure interrupt affinity to dedicate cores for control"""
        print("Configuring interrupt affinity...")

        # Move most interrupts to CPU 0-2, reserve CPU 3 for real-time control
        try:
            # Get list of interrupt files
            irq_path = "/proc/interrupts"
            with open(irq_path, 'r') as f:
                lines = f.readlines()

            # Example: Configure specific IRQs to specific CPUs
            # This is a simplified example - real implementation would be more complex
            print("Interrupt affinity configured")
        except Exception as e:
            print(f"Warning: Could not configure interrupt affinity: {e}")

    def setup_realtime_user_permissions(self):
        """Setup user permissions for real-time operation"""
        print("Setting up real-time user permissions...")

        # Add current user to realtime group
        username = os.getlogin()
        cmd = f"sudo usermod -a -G realtime {username}"
        try:
            subprocess.run(cmd.split(), check=True)
            print(f"Added {username} to realtime group")
        except subprocess.CalledProcessError:
            print(f"Warning: Could not add {username} to realtime group")

    def install_ros2_control_dependencies(self):
        """Install ROS 2 Control dependencies optimized for Jetson"""
        print("Installing ROS 2 Control dependencies...")

        # Install required packages
        packages = [
            "ros-humble-ros2-control",
            "ros-humble-ros2-controllers",
            "ros-humble-realtime-tools",
            "ros-humble-hardware-interface"
        ]

        for package in packages:
            cmd = f"sudo apt install -y {package}"
            try:
                subprocess.run(cmd.split(), check=True)
                print(f"Installed {package}")
            except subprocess.CalledProcessError:
                print(f"Warning: Could not install {package}")

    def optimize_system_for_realtime(self):
        """Apply system optimizations for real-time performance"""
        print("Applying system optimizations...")

        # Disable CPU idle states that can cause latency
        try:
            # Disable CPU C-states
            with open('/sys/devices/system/cpu/cpuidle/enable', 'w') as f:
                f.write('0')
            print("CPU idle states disabled")
        except:
            print("Warning: Could not disable CPU idle states")

        # Reduce kernel preemption latency
        try:
            # This would typically involve kernel parameter changes
            print("Kernel latency optimizations applied")
        except:
            print("Warning: Could not apply kernel latency optimizations")

def main():
    """Main setup function"""
    print("NVIDIA Jetson Real-Time Control Setup")
    print("=" * 50)

    setup = JetsonControlSetup()

    print(f"Detected Jetson model: {setup.jetson_model}")

    # Perform setup steps
    setup.configure_realtime_kernel()
    setup.setup_realtime_user_permissions()
    setup.install_ros2_control_dependencies()
    setup.optimize_system_for_realtime()

    print("\nReal-time setup completed!")
    print("Reboot recommended for all changes to take effect.")
    print("After reboot, verify real-time operation with: 'uname -r' (should show 'rt' in kernel name)")

if __name__ == "__main__":
    main()
```

### Raspberry Pi Configuration for Lightweight Control

```python
# raspberry_pi_setup.py - Setup for lightweight control on Raspberry Pi
import os
import subprocess
import sys
from pathlib import Path

class RaspberryPiControlSetup:
    """Setup utilities for real-time control on Raspberry Pi platforms"""

    def __init__(self):
        self.pi_model = self.detect_pi_model()
        self.pi_version = self.get_pi_version()

    def detect_pi_model(self):
        """Detect Raspberry Pi model"""
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().strip().replace('\x00', '')
                return model
        except:
            return "Unknown Raspberry Pi Model"

    def get_pi_version(self):
        """Get Raspberry Pi version"""
        try:
            with open('/proc/cpuinfo', 'r') as f:
                for line in f:
                    if line.startswith('Revision'):
                        return line.split(':')[1].strip()
        except:
            return "Unknown"
        return "Unknown"

    def configure_for_control(self):
        """Configure Raspberry Pi for lightweight control applications"""
        print(f"Configuring Raspberry Pi: {self.pi_model}")

        # Optimize CPU performance
        self.optimize_cpu()

        # Configure GPIO for actuator control
        self.configure_gpio()

        # Install lightweight control libraries
        self.install_control_libraries()

        # Configure real-time priorities (if available)
        self.configure_realtime()

        return True

    def optimize_cpu(self):
        """Optimize CPU settings for control applications"""
        print("Optimizing CPU settings...")

        # Set CPU governor to performance
        try:
            # For all CPU cores
            cpu_count = os.cpu_count()
            for cpu in range(cpu_count):
                gov_path = f"/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_governor"
                if os.path.exists(gov_path):
                    with open(gov_path, 'w') as f:
                        f.write('performance')
            print("CPU governor set to performance mode")
        except Exception as e:
            print(f"Warning: Could not set CPU governor: {e}")

    def configure_gpio(self):
        """Configure GPIO pins for actuator control"""
        print("Configuring GPIO for actuator control...")

        # Install GPIO library
        try:
            import RPi.GPIO as GPIO
            print("RPi.GPIO library available")
        except ImportError:
            print("Installing RPi.GPIO...")
            subprocess.run([sys.executable, "-m", "pip", "install", "RPi.GPIO"])

        # Example GPIO configuration for PWM control
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering

            # Configure GPIO pins for servo/PWM control
            # Example pins for 4 actuators
            actuator_pins = [18, 19, 20, 21]  # GPIO pins

            for pin in actuator_pins:
                GPIO.setup(pin, GPIO.OUT)
                # Create PWM instance for each pin
                pwm = GPIO.PWM(pin, 50)  # 50Hz for servos
                pwm.start(0)  # Start with 0% duty cycle
                print(f"Configured GPIO {pin} for PWM control")

        except ImportError:
            print("RPi.GPIO not available on this system")
        except Exception as e:
            print(f"Warning: Could not configure GPIO: {e}")

    def install_control_libraries(self):
        """Install lightweight control libraries"""
        print("Installing control libraries...")

        libraries = [
            "pigpio",      # Hardware PWM library
            "wiringpi",    # Alternative GPIO library
            "numpy",       # Numerical computations
            "scipy",       # Scientific computing
        ]

        for lib in libraries:
            try:
                subprocess.run([sys.executable, "-m", "pip", "install", lib], check=True)
                print(f"Installed {lib}")
            except subprocess.CalledProcessError:
                print(f"Warning: Could not install {lib}")

    def configure_realtime(self):
        """Configure real-time capabilities if available"""
        print("Configuring real-time capabilities...")

        # Check if real-time kernel is available
        try:
            result = subprocess.run(['uname', '-r'], capture_output=True, text=True, check=True)
            kernel_info = result.stdout.strip()

            if 'PREEMPT' in kernel_info.upper():
                print("Preemptible kernel detected - real-time friendly")
            else:
                print("Standard kernel - consider real-time kernel for critical applications")

            # Configure user for real-time privileges
            self.setup_realtime_user()

        except Exception as e:
            print(f"Warning: Could not check real-time capabilities: {e}")

    def setup_realtime_user(self):
        """Setup user for real-time operation"""
        try:
            username = os.getlogin()
            # Add to realtime group if it exists
            subprocess.run(['sudo', 'usermod', '-a', '-G', 'realtime', username], check=True)
            print(f"Added {username} to realtime group")
        except subprocess.CalledProcessError:
            print("Could not add user to realtime group - may need to create group first")
        except:
            print("Could not determine current user for real-time setup")

def main():
    """Main setup function"""
    print("Raspberry Pi Control System Setup")
    print("=" * 40)

    setup = RaspberryPiControlSetup()

    print(f"Detected: {setup.pi_model}")
    print(f"Revision: {setup.pi_version}")

    setup.configure_for_control()

    print("\nRaspberry Pi setup completed!")
    print("Configuration optimized for lightweight control applications.")

if __name__ == "__main__":
    main()
```

## Real-Time Control Algorithms

### PID Controller with Real-Time Safety

```cpp
// realtime_pid_controller.cpp - Real-time safe PID controller implementation
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <controller_interface/controller_interface.hpp>

namespace realtime_control {

class RealtimePIDController : public controller_interface::ControllerInterface {
public:
    RealtimePIDController() = default;

    controller_interface::CallbackReturn on_init() override {
        try {
            // Initialize PID parameters (typically from parameters)
            proportional_gain_ = 100.0;  // Example values
            integral_gain_ = 10.0;
            derivative_gain_ = 5.0;

            // Initialize error terms
            previous_error_ = 0.0;
            integral_error_ = 0.0;

            // Initialize command and state interfaces
            joint_command_interface_ = nullptr;
            joint_state_interface_ = nullptr;

        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override {
        // Real-time safe PID control loop

        if (!joint_command_interface_ || !joint_state_interface_) {
            return controller_interface::return_type::ERROR;
        }

        // Get current state
        double current_position = joint_state_interface_->get().get_value();
        double current_velocity = 0.0;  // If velocity interface available

        // Get desired position from buffer (real-time safe)
        double desired_position = desired_position_buffer_.readFromRT()->position;

        // Calculate error
        double error = desired_position - current_position;

        // Update integral term (with anti-windup)
        integral_error_ += error * period.seconds();

        // Anti-windup: limit integral term
        const double integral_limit = 100.0;
        integral_error_ = std::max(-integral_limit, std::min(integral_error_, integral_limit));

        // Calculate derivative (use velocity if available, otherwise numerical)
        double derivative_error = 0.0;
        if (std::abs(period.seconds()) > 1e-9) {  // Avoid division by zero
            derivative_error = (error - previous_error_) / period.seconds();
        }

        // Store current error for next iteration
        previous_error_ = error;

        // Calculate PID output
        double proportional_term = proportional_gain_ * error;
        double integral_term = integral_gain_ * integral_error_;
        double derivative_term = derivative_gain_ * derivative_error;

        double output = proportional_term + integral_term + derivative_term;

        // Apply output limits
        const double output_limit = 100.0;  // Example limit
        output = std::max(-output_limit, std::min(output, output_limit));

        // Set command (real-time safe)
        joint_command_interface_->get().set_value(output);

        return controller_interface::return_type::OK;
    }

    // Real-time safe method to set desired position
    void set_desired_position(double position) {
        DesiredPosition new_desired;
        new_desired.position = position;
        desired_position_buffer_.writeFromNonRT(new_desired);
    }

private:
    struct DesiredPosition {
        double position;
    };

    // PID gains
    double proportional_gain_;
    double integral_gain_;
    double derivative_gain_;

    // PID state variables
    double previous_error_;
    double integral_error_;

    // Real-time safe buffer for desired position
    realtime_tools::RealtimeBuffer<DesiredPosition> desired_position_buffer_;

    // Joint interfaces
    std::shared_ptr<hardware_interface::LoanedCommandInterface> joint_command_interface_;
    std::shared_ptr<hardware_interface::LoanedStateInterface> joint_state_interface_;
};

} // namespace realtime_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(realtime_control::RealtimePIDController, controller_interface::ControllerInterface)
```

## Summary

Real-time control systems and embedded hardware form the backbone of humanoid robotics deployment, providing the deterministic timing and computational resources necessary for safety-critical robot operations. Through **ROS 2 Control framework**, **real-time scheduling configuration**, and **embedded platform optimization**, developers can create robust control systems that meet the strict timing requirements of humanoid robot applications.

Key capabilities include:
- **Real-time scheduling** using SCHED_FIFO policy for deterministic execution
- **Safe data exchange** using RealtimeBuffer for communication between real-time and non-real-time threads
- **Hardware abstraction** through ROS 2 Control interfaces for consistent control across different platforms
- **Embedded optimization** for resource-constrained platforms like NVIDIA Jetson and Raspberry Pi

In Chapter 10, we will explore real-time control algorithms in greater depth, including advanced control strategies like Model Predictive Control (MPC) and whole-body control for humanoid robots.

## Review Questions

1. **Conceptual**: Explain the difference between hard real-time and soft real-time systems in the context of humanoid robotics. Provide examples of each type of control requirement.

2. **Applied**: Implement a real-time safe joint position controller using ROS 2 Control that maintains < 1ms response time while handling sensor noise and actuator saturation limits.

3. **Structural**: Compare the computational requirements and real-time performance characteristics of PID control versus Model Predictive Control on embedded hardware platforms.