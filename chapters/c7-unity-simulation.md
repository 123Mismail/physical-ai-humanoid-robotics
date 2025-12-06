---
id: c7-unity-simulation
title: "Chapter 7: Unity Simulation for Humanoid Robotics"
sidebar_label: "C7: Unity Simulation"
sidebar_position: 7
---

# Chapter 7: Unity Simulation for Humanoid Robotics

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Configure** Unity environments for humanoid robot simulation with physics and rendering optimization
2. **Import** URDF robot models into Unity using specialized loaders and maintain kinematic accuracy
3. **Implement** humanoid control systems using Unity's physics engine and ECS (Entity Component System)
4. **Deploy** Unity simulations for rapid prototyping and visualization of humanoid behaviors

## Overview

Chapter 5 introduced Gazebo simulation with physics-based dynamics and Chapter 6 explored NVIDIA Isaac Sim's GPU-accelerated photorealistic rendering. While these platforms excel at physics accuracy and synthetic data generation respectively, humanoid robotics development also benefits from game engine-based simulation for rapid prototyping, visualization, and interactive development.

**Unity** provides a powerful real-time 3D development platform that combines high-performance physics simulation with sophisticated rendering capabilities, intuitive visual editing tools, and extensive asset libraries. Unity's flexible Entity Component System (ECS) architecture and Data-Oriented Technology Stack (DOTS) make it particularly suitable for simulating complex humanoid systems with multiple interacting components.

This chapter introduces Unity's robotics capabilities, URDF import workflows, physics simulation using Unity Physics, and ECS-based control systems. You will learn to configure Unity environments for humanoid simulation, import and maintain robot models, and implement control systems that leverage Unity's real-time capabilities for interactive development and testing.

## Key Concepts

- **Unity**: Cross-platform real-time 3D development platform for creating interactive applications, games, and simulations
- **URDF Loaders**: Tools and packages that enable importing Robot Operating System (ROS) URDF files into Unity for robot model representation
- **Entity Component System (ECS)**: Unity's data-oriented architecture pattern that separates data (components) from behavior (systems) for high-performance simulation
- **Unity Physics**: High-performance physics engine providing constrained rigid body simulation for Unity projects
- **Data-Oriented Technology Stack (DOTS)**: Unity's collection of technologies (ECS, Burst Compiler, Unity Collections) for building high-performance applications
- **Burst Compiler**: Unity compiler that generates highly optimized native code from C# for significant performance gains
- **PhysicsWorld**: Unity Physics data structure containing all physics simulation state including rigid bodies, colliders, and constraints
- **SimulationSingleton**: Unity Physics component that manages the physics simulation pipeline and step execution

## Unity Robotics Architecture and ECS

Unity's architecture for robotics leverages the Entity Component System (ECS) pattern for high-performance simulation:

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  ROS 2 Bridge   │◄──►│  Unity Robotics  │◄──►│  Unity Physics   │
│  (Controllers,  │    │  Package         │    │  (PhysicsWorld,  │
│   Perception)   │    │                  │    │   Constraints)   │
└─────────────────┘    └──────────────────┘    └──────────────────┘
                                                      │
                                                      ▼
                                               ┌──────────────────┐
                                               │  ECS Architecture│
                                               │  (Entities,      │
                                               │   Components,     │
                                               │   Systems)        │
                                               └──────────────────┘
                                                      │
                                                      ▼
                                               ┌──────────────────┐
                                               │  Burst Compiler  │
                                               │  (Native Code    │
                                               │   Optimization)  │
                                               └──────────────────┘
```

Unity's ECS architecture separates data (Components) from behavior (Systems), enabling efficient parallel processing and memory layout optimization. The **Burst Compiler** transforms C# code into highly optimized native code that runs at near-C++ performance.

### ECS-Based Robot Representation

```csharp
// RobotComponents.cs - Unity ECS Components for Robot Representation
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;

// Robot identification and state
public struct RobotIdentity : IComponentData
{
    public FixedString64Bytes RobotName;
    public int RobotId;
}

public struct RobotState : IComponentData
{
    public float3 Position;
    public quaternion Rotation;
    public float3 Velocity;
    public float3 AngularVelocity;
}

// Joint state component
public struct JointState : IComponentData
{
    public float Position;
    public float Velocity;
    public float Effort;
    public float MinPosition;
    public float MaxPosition;
}

// URDF link mapping
public struct URDFLink : IComponentData
{
    public FixedString64Bytes LinkName;
    public float Mass;
    public float3 CenterOfMass;
    public float3 InertiaTensor;
}

// Physics configuration
public struct PhysicsConfig : IComponentData
{
    public float GravityScale;
    public bool IsKinematic;
    public PhysicsMaterial Material;
}
```

### ECS System for Physics Simulation

```csharp
// RobotPhysicsSystem.cs - Unity ECS System for Robot Physics
using Unity.Burst;
using Unity.Entities;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Mathematics;

[BurstCompile]
public partial struct RobotPhysicsSystem : ISystem
{
    private SimulationSystemGroup simulationSystemGroup;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        simulationSystemGroup = state.World.GetOrCreateSystem<SimulationSystemGroup>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        // Get physics world singleton for read-write access
        var physicsWorld = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>();
        var simulation = simulationSystemGroup.CreateSimulation(ref state);

        // Process all robot entities with physics components
        var robotEntityGroup = SystemAPI.QueryBuilder()
            .WithAll<RobotIdentity, RobotState, PhysicsBody>()
            .Build();

        var robotEntities = robotEntityGroup.ToEntityArray(Allocator.Temp);
        var robotStates = robotEntityGroup.ToComponentDataArray<RobotState>(Allocator.Temp);

        // Update robot physics based on control inputs
        foreach (var entity in robotEntities)
        {
            if (state.EntityManager.HasComponent<JointControl>(entity))
            {
                var jointControl = state.EntityManager.GetComponentData<JointControl>(entity);
                var robotState = state.EntityManager.GetComponentData<RobotState>(entity);

                // Apply joint forces based on control inputs
                ApplyJointForces(ref state, entity, jointControl, robotState);
            }
        }

        robotEntities.Dispose();
        robotStates.Dispose();
    }

    private void ApplyJointForces(ref SystemState state, Entity robotEntity,
                                 JointControl control, RobotState robotState)
    {
        // Implementation for applying physics forces to robot joints
        // This would typically involve calculating torques based on desired positions
        // and current positions, then applying those forces to the physics simulation
    }
}

// Component for joint control commands
public struct JointControl : IComponentData
{
    public DynamicBuffer<JointCommand> Commands;
}

public struct JointCommand : IBufferElementData
{
    public int JointIndex;
    public float TargetPosition;
    public float TargetVelocity;
    public float MaxEffort;
}
```

## URDF Import and Robot Model Integration

Unity supports importing URDF robot models through specialized loader packages that maintain kinematic accuracy and visual representation:

### URDF Loader Implementation

```csharp
// URDFLoader.cs - Loading URDF Models into Unity
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public class URDFLoader : MonoBehaviour
{
    [System.Serializable]
    public struct URDFLoaderOptions
    {
        public System.Action<string, string, System.Action<GameObject[]>> LoadMeshCb;
        public URDFRobot TargetRobot;
        public string WorkingPath;
    }

    public URDFRobot Load(string urdfPath, string packagePath, URDFLoaderOptions options = new URDFLoaderOptions())
    {
        // Read and process the URDF file at the given path
        string urdfContent = System.IO.File.ReadAllText(urdfPath);
        return Parse(urdfContent, packagePath, options);
    }

    public URDFRobot Parse(string urdfContent, string packagePath, URDFLoaderOptions options = new URDFLoaderOptions())
    {
        // Parse URDF XML content and create Unity GameObject hierarchy
        URDFRobot robot = options.TargetRobot ?? new URDFRobot();

        // Process robot name from URDF
        robot.robotName = ExtractRobotName(urdfContent);

        // Parse links and joints from URDF
        var links = ParseLinks(urdfContent);
        var joints = ParseJoints(urdfContent);

        // Create Unity GameObject hierarchy
        foreach (var link in links)
        {
            CreateLinkGameObject(robot, link, options);
        }

        foreach (var joint in joints)
        {
            CreateJointConstraint(robot, joint, options);
        }

        return robot;
    }

    private void CreateLinkGameObject(URDFRobot robot, URDFLinkData linkData, URDFLoaderOptions options)
    {
        GameObject linkObject = new GameObject(linkData.Name);

        // Set transform based on URDF origin
        linkObject.transform.position = URDFToUnityPosition(linkData.Origin.Position);
        linkObject.transform.rotation = URDFToUnityRotation(linkData.Origin.Rotation);

        // Add physics components based on URDF inertial and visual data
        if (linkData.Inertial != null)
        {
            var rb = linkObject.AddComponent<Rigidbody>();
            rb.mass = linkData.Inertial.Mass;
            rb.centerOfMass = URDFToUnityPosition(linkData.Inertial.Origin.Position);

            // Set inertia tensor
            rb.inertiaTensor = URDFToUnityVector(linkData.Inertial.Inertia);
        }

        // Add visual mesh if present
        if (linkData.Visual != null && linkData.Visual.Geometry != null)
        {
            AddVisualMesh(linkObject, linkData.Visual, options);
        }

        // Add collision mesh if present
        if (linkData.Collision != null && linkData.Collision.Geometry != null)
        {
            AddCollisionMesh(linkObject, linkData.Collision, options);
        }

        robot.Links.Add(linkData.Name, linkObject);
    }

    private void AddVisualMesh(GameObject linkObject, URDFVisual visual, URDFLoaderOptions options)
    {
        // Load visual geometry based on URDF specification
        if (visual.Geometry.Mesh != null)
        {
            string meshPath = ResolvePackagePath(visual.Geometry.Mesh.Filename, options.WorkingPath);
            if (options.LoadMeshCb != null)
            {
                options.LoadMeshCb(meshPath, System.IO.Path.GetExtension(meshPath),
                    (gameObjects) => {
                        foreach (var go in gameObjects)
                        {
                            go.transform.SetParent(linkObject.transform);
                            go.AddComponent<MeshRenderer>();
                        }
                    });
            }
        }
    }

    private void AddCollisionMesh(GameObject linkObject, URDFCollision collision, URDFLoaderOptions options)
    {
        // Add collision components based on URDF specification
        if (collision.Geometry.Mesh != null)
        {
            string meshPath = ResolvePackagePath(collision.Geometry.Mesh.Filename, options.WorkingPath);
            var meshCollider = linkObject.AddComponent<MeshCollider>();
            // Load and assign collision mesh
        }
        else if (collision.Geometry.Box != null)
        {
            var boxCollider = linkObject.AddComponent<BoxCollider>();
            boxCollider.size = URDFToUnityVector(collision.Geometry.Box.Size);
        }
        else if (collision.Geometry.Cylinder != null)
        {
            var capsuleCollider = linkObject.AddComponent<CapsuleCollider>();
            capsuleCollider.radius = collision.Geometry.Cylinder.Radius;
            capsuleCollider.height = collision.Geometry.Cylinder.Length;
        }
        else if (collision.Geometry.Sphere != null)
        {
            var sphereCollider = linkObject.AddComponent<SphereCollider>();
            sphereCollider.radius = collision.Geometry.Sphere.Radius;
        }
    }

    // Helper methods for coordinate system conversion
    private Vector3 URDFToUnityPosition(URDFVector3 urdfPos)
    {
        // URDF uses right-handed coordinate system (X forward, Y left, Z up)
        // Unity uses left-handed coordinate system (X right, Y up, Z forward)
        return new Vector3(urdfPos.Y, urdfPos.Z, urdfPos.X);
    }

    private Quaternion URDFToUnityRotation(URDFVector3 urdfRot)
    {
        // Convert URDF Euler angles to Unity quaternion
        Vector3 unityEuler = URDFToUnityPosition(urdfRot);
        return Quaternion.Euler(unityEuler);
    }

    private Vector3 URDFToUnityVector(URDFVector3 urdfVec)
    {
        return new Vector3(urdfVec.Y, urdfVec.Z, urdfVec.X);
    }

    private string ResolvePackagePath(string filename, string workingPath)
    {
        if (filename.StartsWith("package://"))
        {
            // Replace package:// with working path
            return filename.Replace("package://", workingPath + "/");
        }
        return filename;
    }
}

// URDF Data Structures
[System.Serializable]
public class URDFRobot
{
    public string robotName;
    public Dictionary<string, GameObject> Links = new Dictionary<string, GameObject>();
    public Dictionary<string, Joint> Joints = new Dictionary<string, Joint>();
    public Dictionary<string, GameObject> Colliders = new Dictionary<string, GameObject>();
    public Dictionary<string, GameObject> Visuals = new Dictionary<string, GameObject>();
    public Dictionary<string, Transform> Frames = new Dictionary<string, Transform>();
}

[System.Serializable]
public struct URDFLinkData
{
    public string Name;
    public URDFInertial Inertial;
    public URDFVisual Visual;
    public URDFCollision Collision;
    public URDFOrigin Origin;
}

[System.Serializable]
public struct URDFJointData
{
    public string Name;
    public string Type;
    public string ParentLink;
    public string ChildLink;
    public URDFOrigin Origin;
    public URDFAxis Axis;
    public URDFJointLimit Limit;
}

[System.Serializable]
public struct URDFVector3
{
    public float X, Y, Z;
}
```

## Physics Simulation Configuration

Unity's physics engine provides high-performance simulation capabilities for humanoid robots:

### Physics World Setup

```csharp
// PhysicsSetup.cs - Configuring Physics for Humanoid Simulation
using Unity.Entities;
using Unity.Physics;
using Unity.Physics.Systems;
using UnityEngine;

public class PhysicsSetup : MonoBehaviour
{
    public float Gravity = -9.81f;
    public float FixedTimeStep = 0.016666f; // 60 Hz
    public int SolverIterations = 8;
    public PhysicsMaterial DefaultMaterial;

    void Start()
    {
        // Configure physics settings
        Physics.simulationMode = SimulationMode.Script;
        Physics.defaultSolverIterations = SolverIterations;

        // Set gravity (Unity uses Y as up, so gravity is negative Y)
        Physics.gravity = new Vector3(0, Gravity, 0);

        // Initialize physics world through ECS
        InitializePhysicsWorld();
    }

    private void InitializePhysicsWorld()
    {
        var entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;

        // Create physics world singleton if it doesn't exist
        var physicsWorldEntity = entityManager.CreateEntity();
        entityManager.AddComponentData(physicsWorldEntity, new PhysicsWorldSingleton());

        // Configure simulation parameters
        var simulationEntity = entityManager.CreateEntity();
        entityManager.AddComponentData(simulationEntity, new SimulationSingleton());
    }
}

// PhysicsSystemGroup configuration for humanoid robots
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(ExportPhysicsWorld))]
public partial struct HumanoidPhysicsSystemGroup : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        // Configure the simulation group for humanoid physics
        state.RequireForUpdate<SimulationSingleton>();
    }

    public void OnUpdate(ref SystemState state)
    {
        // Custom physics update logic for humanoid robots
        // This could include special handling for balance, contact detection, etc.
    }
}
```

### Contact Detection and Balance Control

```csharp
// BalanceControlSystem.cs - ECS System for Humanoid Balance Control
using Unity.Burst;
using Unity.Entities;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Mathematics;

[BurstCompile]
public partial struct BalanceControlSystem : ISystem
{
    private BuildPhysicsWorld buildPhysicsWorldSystem;
    private SimulationSystemGroup simulationSystemGroup;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        buildPhysicsWorldSystem = state.World.GetOrCreateSystem<BuildPhysicsWorld>();
        simulationSystemGroup = state.World.GetOrCreateSystem<SimulationSystemGroup>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        // Get physics world for contact detection
        var physicsWorld = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>();

        // Query for humanoid entities that need balance control
        var humanoidQuery = SystemAPI.QueryBuilder()
            .WithAll<HumanoidTag, BalanceController, RobotState>()
            .Build();

        var humanoidEntities = humanoidQuery.ToEntityArray(Allocator.Temp);

        foreach (var entity in humanoidEntities)
        {
            // Perform balance control calculations
            var balanceController = state.EntityManager.GetComponentData<BalanceController>(entity);
            var robotState = state.EntityManager.GetComponentData<RobotState>(entity);

            // Detect ground contact for balance
            var groundContacts = DetectGroundContacts(ref physicsWorld.ValueRO, entity);

            // Apply balance corrections
            ApplyBalanceControl(ref state, entity, balanceController, robotState, groundContacts);
        }

        humanoidEntities.Dispose();
    }

    private NativeArray<DistanceHit> DetectGroundContacts(
        ref PhysicsWorldSingleton physicsWorld, Entity humanoidEntity)
    {
        // Perform raycasts to detect ground contact
        var entityManager = SystemAPI.GetEntityManagerRW();
        var humanoidState = entityManager.GetComponentData<RobotState>(humanoidEntity);

        // Cast rays from feet to ground
        var raycastInput = new RaycastInput
        {
            Start = humanoidState.Position + new float3(0, -0.1f, 0), // Slightly below feet
            End = humanoidState.Position + new float3(0, -0.5f, 0),   // Down to ground
            Filter = CollisionFilter.Default
        };

        var hits = new NativeList<DistanceHit>(64, Allocator.Temp);
        physicsWorld.CastRay(raycastInput, hits);

        return hits.AsArray();
    }

    private void ApplyBalanceControl(ref SystemState state, Entity entity,
                                   BalanceController controller, RobotState robotState,
                                   NativeArray<DistanceHit> contacts)
    {
        // Calculate balance corrections based on contact points
        float3 centerOfMass = CalculateCenterOfMass(state.EntityManager, entity);
        float3 centerOfPressure = CalculateCenterOfPressure(contacts);

        // Simple inverted pendulum model for balance
        float3 error = centerOfMass.xz - centerOfPressure.xz;
        float3 correction = error * controller.PGain;

        // Apply correction torques to joints
        ApplyTorquesToJoints(ref state, entity, correction, controller);
    }

    private float3 CalculateCenterOfMass(EntityManager entityManager, Entity entity)
    {
        // Calculate center of mass based on all rigid bodies in the humanoid
        // Implementation would aggregate masses and positions of all links
        return new float3(0, 0, 0);
    }

    private float3 CalculateCenterOfPressure(NativeArray<DistanceHit> contacts)
    {
        // Calculate center of pressure from contact points
        if (contacts.Length == 0) return new float3(0, 0, 0);

        float3 cop = new float3(0, 0, 0);
        float totalForce = 0;

        for (int i = 0; i < contacts.Length; i++)
        {
            cop += contacts[i].Position * contacts[i].Distance;
            totalForce += contacts[i].Distance;
        }

        return totalForce > 0 ? cop / totalForce : new float3(0, 0, 0);
    }

    private void ApplyTorquesToJoints(ref SystemState state, Entity entity,
                                    float3 correction, BalanceController controller)
    {
        // Apply corrective torques to humanoid joints
        // This would typically involve inverse kinematics calculations
    }
}

// Components for balance control
public struct HumanoidTag : IComponentData { }

public struct BalanceController : IComponentData
{
    public float PGain;        // Proportional gain for balance correction
    public float IGain;        // Integral gain for balance correction
    public float DGain;        // Derivative gain for balance correction
    public float MaxTorque;    // Maximum torque to apply
    public float3 CoMTarget;   // Target center of mass position
}

public struct ContactInfo : IComponentData
{
    public float3 ContactPoint;
    public float3 ContactNormal;
    public float ContactForce;
    public bool IsInContact;
}
```

## Unity-Based Humanoid Control Systems

Unity's real-time capabilities make it ideal for interactive humanoid control and rapid prototyping:

### Inverse Kinematics System

```csharp
// InverseKinematicsSystem.cs - ECS System for Inverse Kinematics
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

[BurstCompile]
public partial struct InverseKinematicsSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var job = new InverseKinematicsJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime
        };

        job.Schedule();
    }
}

[BurstCompile]
public partial struct InverseKinematicsJob : IJobEntity
{
    public float DeltaTime;

    void Execute(ref RobotState robotState, in IKTarget target, ref DynamicBuffer<JointState> joints)
    {
        // Perform inverse kinematics to reach target position
        float3 currentPosition = GetEndEffectorPosition(joints);
        float3 error = target.Position - currentPosition;

        if (math.length(error) > target.Tolerance)
        {
            // Calculate joint angle adjustments using Jacobian transpose method
            float3x3 jacobian = CalculateJacobian(joints);
            float3x3 jacobianTranspose = math.transpose(jacobian);

            // Apply small adjustments iteratively
            float3 jointAdjustments = math.mul(jacobianTranspose, error * DeltaTime * target.Gain);

            // Update joint angles
            for (int i = 0; i < joints.Length && i < jointAdjustments.c0.x; i++)
            {
                joints[i] = new JointState
                {
                    Position = joints[i].Position + jointAdjustments[i],
                    Velocity = 0,
                    Effort = 0,
                    MinPosition = joints[i].MinPosition,
                    MaxPosition = joints[i].MaxPosition
                };
            }
        }
    }

    private float3 GetEndEffectorPosition(DynamicBuffer<JointState> joints)
    {
        // Calculate end effector position based on current joint angles
        // This is a simplified representation - real implementation would use forward kinematics
        return new float3(0, 0, 0);
    }

    private float3x3 CalculateJacobian(DynamicBuffer<JointState> joints)
    {
        // Calculate Jacobian matrix for the robot arm
        // This represents the relationship between joint velocities and end-effector velocity
        return float3x3.identity;
    }
}

// Component for IK targets
public struct IKTarget : IComponentData
{
    public float3 Position;      // Target position for end effector
    public quaternion Rotation;  // Target rotation for end effector
    public float Tolerance;      // Acceptable error tolerance
    public float Gain;           // IK solver gain
    public int ChainLength;      // Number of joints in the kinematic chain
}
```

## Visualization and Debugging Tools

Unity's visual editing capabilities provide powerful tools for humanoid robot development:

```csharp
// DebugVisualization.cs - Visualization tools for humanoid debugging
using UnityEngine;
using Unity.Entities;

public class DebugVisualization : MonoBehaviour
{
    public Material CoMVisualizationMaterial;
    public Material ZMPLocationMaterial;
    public float VisualizationScale = 0.1f;

    void OnDrawGizmos()
    {
        // Visualize center of mass
        DrawCenterOfMass();

        // Visualize zero moment point
        DrawZMP();

        // Visualize joint limits
        DrawJointLimits();
    }

    void DrawCenterOfMass()
    {
        // Calculate and visualize center of mass
        Vector3 com = CalculateCenterOfMass();
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(com, VisualizationScale);
        Gizmos.color = Color.white;
        Gizmos.DrawLine(transform.position, com);
    }

    void DrawZMP()
    {
        // Calculate and visualize Zero Moment Point
        Vector3 zmp = CalculateZMP();
        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(zmp, VisualizationScale * 1.5f);
    }

    void DrawJointLimits()
    {
        // Visualize joint limits for debugging
        Gizmos.color = Color.yellow;
        // Draw limit indicators for each joint
    }

    Vector3 CalculateCenterOfMass()
    {
        // Calculate center of mass based on robot configuration
        return transform.position; // Simplified
    }

    Vector3 CalculateZMP()
    {
        // Calculate Zero Moment Point based on forces and torques
        return transform.position; // Simplified
    }
}

// ECS System for real-time visualization
[BurstCompile]
public partial struct VisualizationSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        // Update visualization data for all robots
        var robotQuery = SystemAPI.QueryBuilder()
            .WithAll<RobotIdentity, RobotState>()
            .WithNone<VisualizationDisabled>()
            .Build();

        var robotEntities = robotQuery.ToEntityArray(Allocator.Temp);

        foreach (var entity in robotEntities)
        {
            UpdateVisualizationData(ref state, entity);
        }

        robotEntities.Dispose();
    }

    private void UpdateVisualizationData(ref SystemState state, Entity entity)
    {
        var robotState = state.EntityManager.GetComponentData<RobotState>(entity);
        var robotIdentity = state.EntityManager.GetComponentData<RobotIdentity>(entity);

        // Update visualization components with current robot state
        if (state.EntityManager.HasComponent<VisualizationData>(entity))
        {
            var vizData = state.EntityManager.GetComponentData<VisualizationData>(entity);
            vizData.CurrentPosition = robotState.Position;
            vizData.CurrentRotation = robotState.Rotation;
            vizData.Timestamp = SystemAPI.Time.ElapsedTime;

            state.EntityManager.SetComponentData(entity, vizData);
        }
    }
}

public struct VisualizationData : IComponentData
{
    public float3 CurrentPosition;
    public quaternion CurrentRotation;
    public double Timestamp;
    public FixedString128Bytes RobotName;
}
```

## Summary

Unity simulation provides a powerful platform for humanoid robotics development, combining real-time visualization capabilities with high-performance physics simulation through the Entity Component System (ECS) architecture. By leveraging **URDF loaders** for robot model import, **Unity Physics** for accurate simulation, and **ECS patterns** for high-performance computation, developers can create interactive humanoid simulations suitable for rapid prototyping and visualization.

Key capabilities include:
- **URDF integration** for importing ROS robot models while maintaining kinematic accuracy
- **ECS architecture** for high-performance parallel processing of robot systems
- **Physics simulation** with configurable materials, constraints, and contact detection
- **Real-time visualization** tools for debugging and development

In Chapter 8, we will explore advanced simulation techniques including domain randomization, multi-robot coordination, and simulation-to-reality transfer methods for deploying humanoid robots from simulation to real hardware.

## Review Questions

1. **Conceptual**: Compare Unity's ECS architecture with traditional object-oriented programming for humanoid robot simulation. What are the performance benefits of the data-oriented approach?

2. **Applied**: Modify the BalanceControlSystem to implement a ZMP (Zero Moment Point) controller for humanoid balance, including the mathematical formulation for ZMP calculation.

3. **Structural**: Explain how Unity's Burst Compiler contributes to simulation performance and what constraints it places on C# code for humanoid control systems.