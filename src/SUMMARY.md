# Summary

[Introduction](./introduction.md)

---

# Part I — Mental Models

- [Purpose and Scope of Isaac Sim](./mental-models/01-purpose-and-scope.md)
- [Simulation Execution Model](./mental-models/02-simulation-execution-model.md)
- [Physics Stepping and Determinism](./mental-models/03-physics-stepping-determinism.md)
- [Installation and Environment Setup](./mental-models/04-installation-and-setup.md)
- [Diagnostic Foundations](./mental-models/diagnostic-foundations.md)

# Part II — USD and Scene Structure

- [OpenUSD for Robotics](./usd-scene/05-openusd-for-robotics.md)
- [Transforms, Units, and Coordinate Consistency](./usd-scene/06-transforms-units-coordinates.md)
- [Building and Scaling Environments](./usd-scene/07-building-scaling-environments.md)
- [Common USD Failure Modes](./usd-scene/usd-failure-modes.md)

# Part III — Robot Modeling

- [Robot Import Pipelines](./robot-modeling/08-robot-import-pipelines.md)
- [Articulations, Joints, and Drives](./robot-modeling/09-articulations-joints-drives.md)
- [Collision Geometry and Contact Tuning](./robot-modeling/10-collision-geometry-contact.md)
- [My Robot Doesn't Move Right](./robot-modeling/robot-debugging.md)

# Part IV — Sensors

- [Camera Sensors and Rendering](./sensors/11-camera-sensors-rendering.md)
- [LiDAR and Range Sensors](./sensors/12-lidar-range-sensors.md)
- [IMU, Force/Torque, and Proprioception](./sensors/13-imu-force-proprioception.md)
- [Sensor Fusion Considerations](./sensors/14-sensor-fusion.md)
- [My Sensor Data Looks Wrong](./sensors/sensor-debugging.md)

# Part V — Programming the Simulator

- [Runtime Architecture](./programming/15-runtime-architecture.md)
- [Python Control Patterns](./programming/16-python-control-patterns.md)
- [Controllers and Multi-Robot Systems](./programming/17-controllers-multi-robot.md)
- [Debugging and Diagnostic Workflows](./programming/18-debugging-diagnostics.md)
- [Logging, Headless Execution, and CI](./programming/19-logging-headless-ci.md)
- [Performance Budget Reference](./programming/performance-budgets.md)

# Part VI — Integration Paths

- [Integration Architecture Overview](./integration/20-integration-overview.md)
- [ROS 2 Bridge: Setup and Constraints](./integration/21-ros2-bridge-setup.md)
- [ROS 2: Time, TF, and Synchronization](./integration/22-ros2-time-tf-sync.md)
- [Direct API Integration (Non-ROS)](./integration/23-direct-api-integration.md)
- [Performance and System Separation](./integration/24-performance-system-separation.md)
- [Integration is Broken](./integration/integration-debugging.md)

# Part VII — Synthetic Data Generation

- [When Synthetic Data Works (and When It Doesn't)](./synthetic-data/25-synthetic-data-effectiveness.md)
- [Replicator Architecture](./synthetic-data/26-replicator-architecture.md)
- [Domain Randomization and Dataset Validation](./synthetic-data/27-domain-randomization-validation.md)
- [My Model Doesn't Transfer](./synthetic-data/transfer-debugging.md)

# Part VIII — Robot Learning with Isaac Lab

- [Isaac Lab Architecture](./robot-learning/28-isaac-lab-architecture.md)
- [Reward Design and Training Stability](./robot-learning/29-reward-design-stability.md)
- [Scaling Training: Parallelism and Throughput](./robot-learning/30-scaling-training.md)
- [Policy Evaluation and Deployment](./robot-learning/31-policy-evaluation-deployment.md)
- [Training Isn't Working](./robot-learning/training-debugging.md)

# Part IX — End-to-End Integration

- [Complete Pipeline Walkthrough](./end-to-end/32-complete-pipeline.md)
- [Cross-Stack Failure Analysis](./end-to-end/33-cross-stack-failure-analysis.md)

# Part X — Production Operations

- [Performance Engineering](./production/34-performance-engineering.md)
- [Reproducibility and Regression Testing](./production/35-reproducibility-regression.md)
- [Simulation-to-Real Transfer](./production/36-sim-to-real-transfer.md)
- [Version Migration and Upgrade Strategies](./production/37-version-migration.md)

---

# Appendices

- [System Contracts Reference](./appendices/a-system-contracts.md)
- [Failure Mode Index](./appendices/b-failure-mode-index.md)
- [Performance Budgets](./appendices/c-performance-budgets.md)
- [Version Compatibility Matrix](./appendices/d-version-compatibility.md)
- [Fast Paths](./appendices/e-fast-paths.md)
