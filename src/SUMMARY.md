# Summary

[Introduction](./introduction.md)

---

# Front Matter

- [How to Use This Book](./front-matter/how-to-use.md)

---

# Part I — Mental Models

- [Purpose and Scope of Isaac Sim](./part-1-mental-models/01-purpose-and-scope.md)
- [Simulation Execution Model](./part-1-mental-models/02-simulation-execution-model.md)
- [Physics Stepping and Determinism](./part-1-mental-models/03-physics-stepping-determinism.md)
- [Installation and Environment Setup](./part-1-mental-models/04-installation-and-setup.md)
- [Diagnostic Foundations](./part-1-mental-models/diagnostic-foundations.md)

# Part II — USD and Scene Structure

- [OpenUSD for Robotics](./part-2-usd-scene/05-openusd-for-robotics.md)
- [Transforms, Units, and Coordinate Consistency](./part-2-usd-scene/06-transforms-units-coordinates.md)
- [Building and Scaling Environments](./part-2-usd-scene/07-building-scaling-environments.md)
- [Common USD Failure Modes](./part-2-usd-scene/usd-failure-modes.md)

# Part III — Robot Modeling

- [Robot Import Pipelines](./part-3-robot-modeling/08-robot-import-pipelines.md)
- [Articulations, Joints, and Drives](./part-3-robot-modeling/09-articulations-joints-drives.md)
- [Collision Geometry and Contact Tuning](./part-3-robot-modeling/10-collision-geometry-contact.md)
- [My Robot Doesn't Move Right](./part-3-robot-modeling/robot-debugging.md)

# Part IV — Sensors

- [Camera Sensors and Rendering](./part-4-sensors/11-camera-sensors-rendering.md)
- [LiDAR and Range Sensors](./part-4-sensors/12-lidar-range-sensors.md)
- [IMU, Force/Torque, and Proprioception](./part-4-sensors/13-imu-force-proprioception.md)
- [Sensor Fusion Considerations](./part-4-sensors/14-sensor-fusion.md)
- [My Sensor Data Looks Wrong](./part-4-sensors/sensor-debugging.md)

# Part V — Programming the Simulator

- [Runtime Architecture](./part-5-programming/15-runtime-architecture.md)
- [Python Control Patterns](./part-5-programming/16-python-control-patterns.md)
- [Controllers and Multi-Robot Systems](./part-5-programming/17-controllers-multi-robot.md)
- [Debugging and Diagnostic Workflows](./part-5-programming/18-debugging-diagnostics.md)
- [Logging, Headless Execution, and CI](./part-5-programming/19-logging-headless-ci.md)
- [Performance Budget Reference](./part-5-programming/performance-budgets.md)

# Part VI — Integration Paths

- [Integration Architecture Overview](./part-6-integration/20-integration-overview.md)
- [ROS 2 Bridge: Setup and Constraints](./part-6-integration/21-ros2-bridge-setup.md)
- [ROS 2: Time, TF, and Synchronization](./part-6-integration/22-ros2-time-tf-sync.md)
- [Direct API Integration (Non-ROS)](./part-6-integration/23-direct-api-integration.md)
- [Performance and System Separation](./part-6-integration/24-performance-system-separation.md)
- [Integration is Broken](./part-6-integration/integration-debugging.md)

# Part VII — Synthetic Data Generation

- [When Synthetic Data Works (and When It Doesn't)](./part-7-synthetic-data/25-synthetic-data-effectiveness.md)
- [Replicator Architecture](./part-7-synthetic-data/26-replicator-architecture.md)
- [Domain Randomization and Dataset Validation](./part-7-synthetic-data/27-domain-randomization-validation.md)
- [My Model Doesn't Transfer](./part-7-synthetic-data/transfer-debugging.md)

# Part VIII — Robot Learning with Isaac Lab

- [Isaac Lab Architecture](./part-8-robot-learning/28-isaac-lab-architecture.md)
- [Reward Design and Training Stability](./part-8-robot-learning/29-reward-design-stability.md)
- [Scaling Training: Parallelism and Throughput](./part-8-robot-learning/30-scaling-training.md)
- [Policy Evaluation and Deployment](./part-8-robot-learning/31-policy-evaluation-deployment.md)
- [Training Isn't Working](./part-8-robot-learning/training-debugging.md)

# Part IX — End-to-End Integration

- [Complete Pipeline Walkthrough](./part-9-end-to-end/32-complete-pipeline.md)
- [Cross-Stack Failure Analysis](./part-9-end-to-end/33-cross-stack-failure-analysis.md)

# Part X — Production Operations

- [Performance Engineering](./part-10-production/34-performance-engineering.md)
- [Reproducibility and Regression Testing](./part-10-production/35-reproducibility-regression.md)
- [Simulation-to-Real Transfer](./part-10-production/36-sim-to-real-transfer.md)
- [Version Migration and Upgrade Strategies](./part-10-production/37-version-migration.md)

---

# Appendices

- [System Contracts Reference](./appendices/a-system-contracts.md)
- [Failure Mode Index](./appendices/b-failure-mode-index.md)
- [Performance Budgets](./appendices/c-performance-budgets.md)
- [Version Compatibility Matrix](./appendices/d-version-compatibility.md)
- [Fast Paths](./appendices/e-fast-paths.md)
