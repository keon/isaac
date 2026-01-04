# Physics Stepping and Determinism

## Overview

Determinism in physics simulation means: given identical initial conditions and inputs, the simulation produces identical outputs. This property is valuable for:

- **Debugging**: Reproduce exact failures
- **Regression testing**: Verify changes don't break existing behavior
- **Reproducible research**: Others can replicate your results
- **Training stability**: Same seed should produce same learning trajectory

Isaac Sim, via PhysX 5, provides *conditional* determinism—determinism under specific conditions. Understanding these conditions lets you design appropriately.

## PhysX Determinism Guarantees

NVIDIA PhysX 5 provides the following guarantees for rigid body and articulation simulation:

### Same Hardware, Same Software, Same Configuration

When running on:
- Identical GPU model
- Identical driver version
- Identical PhysX version
- Identical simulation parameters
- Identical initial state
- Identical input sequence

...the simulation will produce bit-identical results.

This is strong enough for most debugging and regression testing workflows, provided you control the environment.

### What's Explicitly NOT Guaranteed

PhysX does *not* guarantee determinism across:

- **Different GPU architectures**: An RTX 4090 and RTX 3080 may produce different results
- **Different driver versions**: Driver updates can change numerical behavior
- **CPU vs GPU execution**: Different code paths, different results
- **Different operating systems**: Floating-point behavior can vary
- **Soft body / cloth simulation**: These solvers don't guarantee determinism

## Sources of Non-Determinism

Understanding where non-determinism enters helps you either eliminate it or work around it.

### Floating-Point Arithmetic

GPUs perform floating-point operations in parallel, and the order of operations can affect results due to floating-point associativity:

```
(a + b) + c ≠ a + (b + c)  # In floating-point, this isn't always equal
```

Different GPU architectures may schedule parallel operations differently, changing the summation order and producing different results.

### Solver Iteration Count

PhysX's constraint solver is iterative. More iterations → more accurate, but also potentially different convergence paths on different hardware. The solver parameters include:

- **Position iteration count**: Iterations for resolving penetrations
- **Velocity iteration count**: Iterations for resolving velocity constraints

These affect both accuracy and determinism characteristics.

### Contact Detection Order

When multiple objects collide simultaneously, the order in which PhysX processes contacts can affect the outcome. On different hardware, thread scheduling differences can change this order.

### Random Seed Handling

Isaac Lab provides seed control:

```python
from omni.isaac.lab.utils.seed import set_seed

# Set seed for reproducibility
set_seed(42)
```

This sets seeds for:
- Python's `random` module
- NumPy
- PyTorch (CPU and CUDA)
- PhysX internal randomization

However, seed setting doesn't overcome hardware-level floating-point differences.

### Runtime Property Changes

Modifying physics material properties during simulation can cause non-determinism, particularly on GPU:

```python
# This can cause non-determinism in GPU simulation
material.set_friction(new_friction_value)  # Runtime change
```

A bug fixed in Isaac Sim 2022.2.1 specifically addressed non-determinism from runtime friction/restitution changes. Ensure you're on a version with this fix if you need runtime material modification.

## Physics Stepping Fundamentals

Understanding how physics steps work helps in both determinism and stability.

### Fixed Timestep

PhysX uses a fixed timestep for stability:

```python
# Typical configuration
physics_dt = 1.0 / 120.0  # 120 Hz physics
```

A fixed timestep means:
- Numerical integration is consistent
- Contact resolution behaves predictably
- Accumulated error is bounded

Variable timesteps introduce instability—avoid them for robotics simulation.

### Substeps

For stability with stiff systems (like robot joints), you may need substeps:

```python
# Physics configuration with substeps
physics_config = {
    "physics_dt": 1/60,
    "substeps": 4  # 4 physics substeps per step
}
```

Each substep runs at `physics_dt / substeps` internally. More substeps → more stable but more compute.

### Solver Parameters

Key PhysX solver parameters:

```python
# From stage physics settings
position_iteration_count = 8   # Default
velocity_iteration_count = 1   # Default
```

Higher iterations improve constraint satisfaction but cost performance. For robotics:
- **Manipulation**: Often needs higher iterations (12-16) for stable grasping
- **Locomotion**: Default iterations often sufficient
- **High-DOF articulations**: May need increased iterations

## Practical Determinism Strategies

Given the limitations, here are practical strategies for working with determinism.

### Strategy 1: Lock Down the Environment

For maximum reproducibility:

```dockerfile
# Docker container for reproducible simulation
FROM nvcr.io/nvidia/isaac-sim:2024.1.0

# Pin all dependencies
# Use specific driver version
# Lock CUDA version
```

Run on identical hardware (same GPU model). This gives you true determinism for debugging and regression testing.

### Strategy 2: Accept Statistical Reproducibility

For training workflows where bit-identical results aren't required:

```python
# Run multiple seeds, report statistics
seeds = [1, 2, 3, 4, 5]
results = [train_policy(seed=s) for s in seeds]
print(f"Mean reward: {np.mean(results)} ± {np.std(results)}")
```

Your claims are "training converges to reward X ± Y across seeds" rather than "training produces exactly reward X with seed 42."

### Strategy 3: Design for Robustness

Build systems that tolerate variation:

```python
# Instead of relying on exact trajectories
class RobustController:
    def compute_action(self, state):
        # Feedback control handles trajectory deviations
        error = self.target - state
        return self.kp * error + self.kd * error_derivative
```

Controllers with feedback naturally handle small numerical differences. Open-loop trajectories don't.

### Strategy 4: Validation Gates

For CI/CD, use statistical tests rather than exact matching:

```python
def test_robot_reaches_target():
    results = []
    for seed in range(5):
        final_position = run_simulation(seed)
        results.append(final_position)
    
    mean_position = np.mean(results, axis=0)
    # Assert target reached on average, within tolerance
    assert np.linalg.norm(mean_position - target) < 0.1
```

This catches regressions while tolerating numerical variation.

## Timestep Selection

Choosing the right timestep balances stability, accuracy, and performance.

### Stability Requirements

Different scenarios need different timesteps:

| Scenario | Typical Timestep | Rationale |
|----------|------------------|-----------|
| Mobile robot navigation | 1/60 s | Slow dynamics, stability easy |
| Manipulation | 1/120 s | Contact events need resolution |
| High-speed manipulation | 1/240 s | Fast motions, stiff contacts |
| Legged locomotion | 1/120 - 1/200 s | Impact events, joint limits |
| Soft contacts (rubber) | 1/240 s+ | Compliant contacts need fine resolution |

### The CFL-Like Condition

While PhysX isn't a traditional PDE solver, there's an analogous stability condition: objects shouldn't move more than their own size per timestep.

```python
# Rough stability check
max_velocity = 10  # m/s
object_size = 0.1  # m
required_dt = object_size / max_velocity  # 0.01 s → 100 Hz minimum
```

If objects move too far per step, tunneling (passing through other objects) and instability occur.

### Joint Stiffness Considerations

Robot joint drives have stiffness and damping:

```python
joint_drive = {
    "stiffness": 1000,  # N·m/rad
    "damping": 100      # N·m·s/rad
}
```

High stiffness requires small timesteps for stability. The relationship is roughly:

```
dt_stable ≈ 2 * sqrt(mass / stiffness)
```

If your joints are oscillating or exploding, reduce timestep or reduce stiffness.

## Numerical Stability Issues

Beyond determinism, numerical stability affects whether simulation produces physically plausible results.

### Gain Wind-Up

Integral controllers accumulate error:

```python
class PIController:
    def __init__(self):
        self.integral = 0
    
    def compute(self, error, dt):
        self.integral += error * dt  # Unbounded growth possible
        return self.kp * error + self.ki * self.integral
```

If the system can't achieve the setpoint (e.g., physics limits prevent it), the integral term grows without bound, causing instability when the constraint is released.

**Fix**: Clamp integral term or use anti-windup schemes.

### Joint Limit Violations

Articulation joints have limits. When drives push past limits:

```python
# If this happens
current_angle = joint.get_position()  # Returns value outside limits
```

It indicates instability. Check:
- Timestep (reduce it)
- Drive stiffness (reduce it)
- Solver iterations (increase them)

### Contact Instability

Stacked objects or complex contact configurations can oscillate:

```
Frame 1: Objects interpenetrate
Frame 2: Solver pushes them apart (overshoot)
Frame 3: Objects separate, then fall
Frame 4: Impact, interpenetrate again
...
```

**Fixes**:
- Increase solver iterations
- Reduce timestep
- Adjust contact parameters (compliance, restitution)
- Use collision geometry with appropriate detail

## GPU vs CPU Physics

Isaac Sim can run physics on CPU or GPU. The choice affects determinism and performance.

### GPU Physics

```python
# Enable GPU physics
physics_context = PhysicsContext()
physics_context.enable_gpu_dynamics(True)
```

**Advantages**:
- Massive parallelism for many bodies/environments
- Essential for RL training throughput

**Disadvantages**:
- Different numerical results than CPU
- Some features limited (see PhysX docs)
- Memory constraints for very large scenes

### CPU Physics

```python
physics_context.enable_gpu_dynamics(False)
```

**Advantages**:
- More predictable behavior
- Better debugging tools
- Some features only available on CPU

**Disadvantages**:
- Much slower for parallel environments
- Bottleneck for RL training

### Hybrid Approach

For development, use CPU physics for debugging, then validate on GPU for training:

```python
def run_simulation(use_gpu: bool):
    physics_context.enable_gpu_dynamics(use_gpu)
    # ... run simulation
```

If behavior differs significantly between CPU and GPU, investigate—there may be configuration or stability issues.

## Running Example

For our warehouse mobile manipulator, determinism considerations affect several design decisions.

### Controller Design

We'll use feedback control rather than open-loop trajectories:

```python
class ArmController:
    def __init__(self, arm):
        self.arm = arm
        self.kp = np.array([100, 100, 100, 50, 50, 50])  # Per-joint gains
        self.kd = np.array([10, 10, 10, 5, 5, 5])
    
    def move_to(self, target_positions):
        current = self.arm.get_joint_positions()
        current_vel = self.arm.get_joint_velocities()
        
        error = target_positions - current
        torque = self.kp * error - self.kd * current_vel
        
        self.arm.apply_joint_torques(torque)
```

This design tolerates small numerical variations—the feedback loop corrects deviations.

### Physics Configuration

For stable manipulation:

```python
physics_config = {
    "physics_dt": 1/120,  # 120 Hz
    "substeps": 2,         # Effective 240 Hz for stability
    "position_iterations": 12,
    "velocity_iterations": 4
}
```

Higher iteration counts for stable grasping contact.

### Testing Strategy

Regression tests will use statistical bounds:

```python
def test_pick_success_rate():
    successes = 0
    trials = 20
    for seed in range(trials):
        result = run_pick_task(seed)
        if result.success:
            successes += 1
    
    success_rate = successes / trials
    assert success_rate >= 0.9, f"Success rate {success_rate} below threshold"
```

This catches regressions while tolerating run-to-run variation.

## Chapter Summary

Key takeaways:

- **Same-configuration determinism**: Same hardware + software + config = identical results
- **Cross-hardware non-determinism**: Different GPUs produce different floating-point outcomes
- **Design for robustness**: Feedback control tolerates variation; open-loop doesn't
- **Statistical testing**: Assert behavior bounds, not exact trajectories
- **Timestep selection**: Match your dynamics (stiffness, velocities, contact requirements)

Next: [Installation and Environment Setup](./04-installation-and-setup.md)—creating reproducible environments that support these determinism strategies.

## References

- [Isaac Lab Reproducibility](https://isaac-sim.github.io/IsaacLab/main/source/features/reproducibility.html) — Seed setting and determinism guarantees
- [PhysX Simulation](https://nvidia-omniverse.github.io/PhysX/physx/5.4.2/docs/Simulation.html) — Solver parameters and timestep configuration
- [PhysX GPU Rigid Bodies](https://nvidia-omniverse.github.io/PhysX/physx/5.4.2/docs/GPURigidBodies.html) — GPU physics behavior and constraints
