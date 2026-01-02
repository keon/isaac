# Appendix E: Fast Paths

Goal-oriented reading paths for different use cases.

## "I just want to train a policy"

**Time estimate:** 6–8 hours reading + experimentation

**Path:**
1. [Purpose and Scope](../part-1-mental-models/01-purpose-and-scope.md) — 20 min
2. [Simulation Execution Model](../part-1-mental-models/02-simulation-execution-model.md) — 30 min
3. [Physics Stepping and Determinism](../part-1-mental-models/03-physics-stepping-determinism.md) — 30 min
4. [Robot Import Pipelines](../part-3-robot-modeling/08-robot-import-pipelines.md) — 1 hour
5. [Articulations, Joints, and Drives](../part-3-robot-modeling/09-articulations-joints-drives.md) — 1 hour
6. [Isaac Lab Architecture](../part-8-robot-learning/28-isaac-lab-architecture.md) — 1 hour
7. [Reward Design and Training Stability](../part-8-robot-learning/29-reward-design-stability.md) — 1.5 hours
8. [Scaling Training](../part-8-robot-learning/30-scaling-training.md) — 1 hour
9. [Policy Evaluation and Deployment](../part-8-robot-learning/31-policy-evaluation-deployment.md) — 1 hour

**Why this path?** Focuses on mental models, robot setup, and learning workflow. Skips sensors, ROS, and synthetic data.

---

## "I just want synthetic data"

**Time estimate:** 5–7 hours reading + experimentation

**Path:**
1. [Purpose and Scope](../part-1-mental-models/01-purpose-and-scope.md) — 20 min
2. [Simulation Execution Model](../part-1-mental-models/02-simulation-execution-model.md) — 30 min
3. [OpenUSD for Robotics](../part-2-usd-scene/05-openusd-for-robotics.md) — 1.5 hours
4. [Transforms, Units, Coordinates](../part-2-usd-scene/06-transforms-units-coordinates.md) — 1 hour
5. [Building and Scaling Environments](../part-2-usd-scene/07-building-scaling-environments.md) — 1.5 hours
6. [When Synthetic Data Works](../part-7-synthetic-data/25-synthetic-data-effectiveness.md) — 45 min
7. [Replicator Architecture](../part-7-synthetic-data/26-replicator-architecture.md) — 1.5 hours
8. [Domain Randomization and Validation](../part-7-synthetic-data/27-domain-randomization-validation.md) — 1.5 hours

**Why this path?** Focuses on scene construction and data generation. Skips robot modeling, control, and learning.

---

## "I'm integrating with an existing ROS stack"

**Time estimate:** 7–9 hours reading + experimentation

**Path:**
1. [Purpose and Scope](../part-1-mental-models/01-purpose-and-scope.md) — 20 min
2. [Simulation Execution Model](../part-1-mental-models/02-simulation-execution-model.md) — 30 min
3. [Physics Stepping and Determinism](../part-1-mental-models/03-physics-stepping-determinism.md) — 30 min
4. [Installation and Setup](../part-1-mental-models/04-installation-and-setup.md) — 1 hour
5. [Robot Import Pipelines](../part-3-robot-modeling/08-robot-import-pipelines.md) — 1 hour
6. [Articulations, Joints, and Drives](../part-3-robot-modeling/09-articulations-joints-drives.md) — 1 hour
7. [Camera Sensors and Rendering](../part-4-sensors/11-camera-sensors-rendering.md) — 1 hour
8. [Integration Architecture Overview](../part-6-integration/20-integration-overview.md) — 30 min
9. [ROS 2 Bridge Setup](../part-6-integration/21-ros2-bridge-setup.md) — 1.5 hours
10. [ROS 2 Time, TF, Sync](../part-6-integration/22-ros2-time-tf-sync.md) — 1.5 hours

**Why this path?** Focuses on robot import, sensors, and ROS integration. Skips learning and synthetic data.

---

## "I have graphics/USD background"

**Time estimate:** 4–6 hours reading

**Path:**
1. [Purpose and Scope](../part-1-mental-models/01-purpose-and-scope.md) — 20 min
2. [Simulation Execution Model](../part-1-mental-models/02-simulation-execution-model.md) — 30 min
3. [Physics Stepping and Determinism](../part-1-mental-models/03-physics-stepping-determinism.md) — 30 min
4. [Robot Import Pipelines](../part-3-robot-modeling/08-robot-import-pipelines.md) — 1 hour
5. [Articulations, Joints, and Drives](../part-3-robot-modeling/09-articulations-joints-drives.md) — 1 hour
6. [Collision Geometry and Contact](../part-3-robot-modeling/10-collision-geometry-contact.md) — 45 min
7. [Camera Sensors and Rendering](../part-4-sensors/11-camera-sensors-rendering.md) — 1 hour
8. [Runtime Architecture](../part-5-programming/15-runtime-architecture.md) — 45 min
9. [Python Control Patterns](../part-5-programming/16-python-control-patterns.md) — 1 hour
10. [Controllers and Multi-Robot](../part-5-programming/17-controllers-multi-robot.md) — 45 min

**Why this path?** Assumes USD/graphics knowledge. Focuses on robotics-specific physics, sensors, and control.

---

## "I need to debug a broken simulation"

**Go directly to troubleshooting sections:**
- [Appendix B: Failure Mode Index](./b-failure-mode-index.md) — start here
- [Part I: Diagnostic Foundations](../part-1-mental-models/diagnostic-foundations.md)
- [Part II: Common USD Failure Modes](../part-2-usd-scene/usd-failure-modes.md)
- [Part III: My Robot Doesn't Move Right](../part-3-robot-modeling/robot-debugging.md)
- [Part IV: My Sensor Data Looks Wrong](../part-4-sensors/sensor-debugging.md)
- [Part VI: Integration is Broken](../part-6-integration/integration-debugging.md)
- [Part VII: My Model Doesn't Transfer](../part-7-synthetic-data/transfer-debugging.md)
- [Part VIII: Training Isn't Working](../part-8-robot-learning/training-debugging.md)

---

## Linear Reading (Comprehensive)

If you're new to Isaac Sim and want deep understanding, read Parts I–X linearly. Estimated total time: **25–30 hours** of focused reading + experimentation.
