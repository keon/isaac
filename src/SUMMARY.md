# Summary

[Introduction](./introduction.md)

---

# Mental Models

- [Purpose and Scope of Isaac Sim](./mental-models/01-purpose-and-scope.md)
- [Simulation Execution Model](./mental-models/02-simulation-execution-model.md)
- [Physics Stepping and Determinism](./mental-models/03-physics-stepping-determinism.md)
- [Installation and Environment Setup](./mental-models/04-installation-and-setup.md)
- [Debugging Guide](./mental-models/diagnostic-foundations.md)

# USD and Scene Structure

- [OpenUSD for Robotics](./usd-scene/05-openusd-for-robotics.md)
- [Transforms, Units, and Coordinates](./usd-scene/06-transforms-units-coordinates.md)
- [Building and Scaling Environments](./usd-scene/07-building-scaling-environments.md)
- [Debugging Guide](./usd-scene/usd-failure-modes.md)

# Robot Modeling

- [Robot Import Pipelines](./robot-modeling/08-robot-import-pipelines.md)
- [Articulations, Joints, and Drives](./robot-modeling/09-articulations-joints-drives.md)
- [Collision Geometry and Contact Tuning](./robot-modeling/10-collision-geometry-contact.md)
- [Debugging Guide](./robot-modeling/robot-debugging.md)

# Sensors

- [Camera Sensors and Rendering](./sensors/11-camera-sensors-rendering.md)
- [LiDAR and Range Sensors](./sensors/12-lidar-range-sensors.md)
- [IMU, Force/Torque, and Proprioception](./sensors/13-imu-force-proprioception.md)
- [Sensor Fusion Considerations](./sensors/14-sensor-fusion.md)
- [Debugging Guide](./sensors/sensor-debugging.md)

# Programming the Simulator

- [Runtime Architecture](./programming/15-runtime-architecture.md)
- [Python Control Patterns](./programming/16-python-control-patterns.md)
- [Controllers and Multi-Robot Systems](./programming/17-controllers-multi-robot.md)
- [Debugging and Diagnostics](./programming/18-debugging-diagnostics.md)
- [Logging, Headless Execution, and CI](./programming/19-logging-headless-ci.md)
- [Performance Budgets](./programming/performance-budgets.md)

# Integration Paths

- [Integration Architecture Overview](./integration/20-integration-overview.md)
- [ROS 2 Bridge Setup and Constraints](./integration/21-ros2-bridge-setup.md)
- [ROS 2 Time, TF, and Synchronization](./integration/22-ros2-time-tf-sync.md)
- [Direct API Integration](./integration/23-direct-api-integration.md)
- [Performance and System Separation](./integration/24-performance-system-separation.md)
- [Debugging Guide](./integration/integration-debugging.md)

# Synthetic Data Generation

- [When Synthetic Data Works](./synthetic-data/25-synthetic-data-effectiveness.md)
- [Replicator Architecture](./synthetic-data/26-replicator-architecture.md)
- [Domain Randomization and Validation](./synthetic-data/27-domain-randomization-validation.md)
- [Debugging Guide](./synthetic-data/transfer-debugging.md)

# Robot Learning with Isaac Lab

- [Isaac Lab Architecture](./robot-learning/28-isaac-lab-architecture.md)
- [Reward Design and Training Stability](./robot-learning/29-reward-design-stability.md)
- [Scaling Training](./robot-learning/30-scaling-training.md)
- [Policy Evaluation and Deployment](./robot-learning/31-policy-evaluation-deployment.md)
- [Debugging Guide](./robot-learning/training-debugging.md)

# End-to-End Integration

- [Complete Pipeline Walkthrough](./end-to-end/32-complete-pipeline.md)
- [Cross-Stack Failure Analysis](./end-to-end/33-cross-stack-failure-analysis.md)

# Production Operations

- [Performance Engineering](./production/34-performance-engineering.md)
- [Reproducibility and Regression Testing](./production/35-reproducibility-regression.md)
- [Simulation-to-Real Transfer](./production/36-sim-to-real-transfer.md)
- [Version Migration and Upgrades](./production/37-version-migration.md)

---

# Appendices

- [System Contracts Reference](./appendices/a-system-contracts.md)
- [Failure Mode Index](./appendices/b-failure-mode-index.md)
- [Performance Budgets](./appendices/c-performance-budgets.md)
- [Version Compatibility Matrix](./appendices/d-version-compatibility.md)
- [Fast Paths](./appendices/e-fast-paths.md)
