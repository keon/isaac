# Debugging Guide

When something breaks, isolate the layer: USD, physics, Python execution, or integration. This guide provides systematic approaches based on the mental models from this section.

## Decision Tree

When something isn't working, follow this diagnostic flow:

```
START: Something is wrong
│
├─► Does Isaac Sim launch?
│   ├─ No → Installation/Driver Issue (Section: Launch Failures)
│   └─ Yes ↓
│
├─► Does the scene load?
│   ├─ No → USD/Asset Issue (Section: Scene Loading)
│   └─ Yes ↓
│
├─► Does physics run when you press Play?
│   ├─ No → Timeline/Physics Configuration (Section: Physics Not Running)
│   └─ Yes ↓
│
├─► Do objects move as expected?
│   ├─ No → Physics/Articulation Issue (Section: Unexpected Behavior)
│   └─ Yes ↓
│
├─► Does your Python code execute?
│   ├─ No → Callback/Lifecycle Issue (Section: Code Not Running)
│   └─ Yes ↓
│
├─► Does your code produce expected results?
│   ├─ No → Logic/Timing Issue (Section: Wrong Results)
│   └─ Yes → Not a simulation problem (check downstream)
```

## Common Patterns

### "Nothing happens when I press play"

**Symptoms**: You click Play, the timeline advances, but nothing in the scene moves.

**Diagnostic Steps**:

1. **Check if physics is enabled on the stage**
   ```python
   from pxr import UsdPhysics
   stage = omni.usd.get_context().get_stage()
   physics_scene = UsdPhysics.Scene.Get(stage, "/physicsScene")
   print(f"Physics scene exists: {physics_scene}")
   ```

2. **Verify objects have physics enabled**
   ```python
   # Check if prim has rigid body
   prim = stage.GetPrimAtPath("/World/MyObject")
   rigid_body = UsdPhysics.RigidBodyAPI.Get(stage, prim.GetPath())
   print(f"Has RigidBodyAPI: {rigid_body}")
   ```

3. **Check for kinematic flag**
   ```python
   # Kinematic bodies don't respond to physics
   if rigid_body:
       kinematic = rigid_body.GetKinematicEnabledAttr().Get()
       print(f"Kinematic (won't simulate): {kinematic}")
   ```

4. **Look for constraints locking the object**
   - Fixed joints to world
   - Locked articulation joints
   - Collision geometry preventing movement

**Common Causes**:
- Missing `PhysicsScene` prim
- Objects don't have `RigidBodyAPI` applied
- Objects are kinematic (animated, not simulated)
- Gravity is set to zero
- Object is constrained by joints or collisions

### "Results change between runs"

**Symptoms**: Same scene, same code, different outcomes each run.

**Diagnostic Steps**:

1. **Check if you're setting seeds**
   ```python
   import random
   import numpy as np
   import torch
   
   seed = 42
   random.seed(seed)
   np.random.seed(seed)
   torch.manual_seed(seed)
   torch.cuda.manual_seed_all(seed)
   
   # For Isaac Lab
   from omni.isaac.lab.utils.seed import set_seed
   set_seed(seed)
   ```

2. **Identify sources of randomness**
   - Random spawning positions
   - Randomized physics properties
   - Random control noise
   - Non-deterministic neural network operations

3. **Check GPU vs CPU physics**
   ```python
   from omni.physx import get_physx_interface
   physx = get_physx_interface()
   print(f"GPU dynamics: {physx.is_gpu_dynamics_enabled()}")
   ```

4. **Verify hardware consistency**
   ```bash
   nvidia-smi  # Check GPU model
   cat /proc/driver/nvidia/version  # Check driver
   ```

**Common Causes**:
- Random seed not set
- GPU physics with hardware variation
- Asynchronous operations with race conditions
- Runtime material property changes (see [Physics Stepping and Determinism](./03-physics-stepping-determinism.md))

### "Simulation is too slow"

**Symptoms**: Simulation runs much slower than real-time, impacting development or training.

**Diagnostic Steps**:

1. **Profile with Isaac Sim's built-in tools**
   - Window → Statistics → Performance
   - Check "Physics", "Rendering", "USD" times

2. **Identify the bottleneck**
   ```python
   import time
   
   start = time.time()
   world.step(render=False)
   physics_time = time.time() - start
   
   start = time.time()
   world.render()
   render_time = time.time() - start
   
   print(f"Physics: {physics_time*1000:.1f}ms, Render: {render_time*1000:.1f}ms")
   ```

3. **Check parallel environment count**
   - More environments = more GPU memory, potentially slower
   - Find the sweet spot for your hardware

4. **Check collision geometry complexity**
   ```python
   # Count collision shapes
   from pxr import UsdGeom
   mesh_count = len([p for p in stage.Traverse() if UsdGeom.Mesh(p)])
   print(f"Mesh count: {mesh_count}")
   ```

**Common Causes**:
- Too many collision primitives (use simplified collision geometry)
- Rendering enabled when not needed
- Too many parallel environments for GPU memory
- Complex shaders/materials when simple would suffice
- Excessive physics substeps

### "Robot joints are unstable"

**Symptoms**: Joints oscillate, explode, or don't reach target positions.

**Diagnostic Steps**:

1. **Check timestep vs joint stiffness**
   ```python
   # Rule of thumb: dt < 2 * sqrt(inertia / stiffness)
   # If stiffness is 10000 and inertia is 0.01:
   # dt < 2 * sqrt(0.01 / 10000) = 0.002s = 500 Hz minimum
   ```

2. **Verify joint drive configuration**
   ```python
   from pxr import UsdPhysics
   joint = UsdPhysics.DriveAPI.Get(stage, joint_prim.GetPath(), "angular")
   print(f"Stiffness: {joint.GetStiffnessAttr().Get()}")
   print(f"Damping: {joint.GetDampingAttr().Get()}")
   ```

3. **Check for joint limit violations**
   ```python
   joint_positions = articulation.get_joint_positions()
   joint_limits = articulation.get_joint_limits()
   for i, (pos, (lo, hi)) in enumerate(zip(joint_positions, joint_limits)):
       if pos < lo or pos > hi:
           print(f"Joint {i} out of limits: {pos} not in [{lo}, {hi}]")
   ```

4. **Increase solver iterations**
   ```python
   # In physics scene settings
   position_iteration_count = 16  # Default is often 8
   velocity_iteration_count = 4   # Default is often 1
   ```

**Common Causes**:
- Stiffness too high for timestep
- Insufficient solver iterations
- Missing damping (pure stiffness oscillates)
- Conflicting joint drives and external forces

### "Sensors return wrong data"

**Symptoms**: Camera shows black image, LiDAR returns no points, IMU reads zero.

**Diagnostic Steps**:

1. **Verify sensor is enabled and configured**
   ```python
   camera = Camera(prim_path="/World/Camera")
   print(f"Resolution: {camera.get_resolution()}")
   print(f"Enabled: {camera.is_valid()}")
   ```

2. **Check if sensor has been initialized after warm-up**
   ```python
   # Sensors often need a few frames to initialize
   for _ in range(5):
       world.step(render=True)
   
   # Now try reading
   rgb = camera.get_rgb()
   print(f"RGB shape: {rgb.shape if rgb is not None else 'None'}")
   ```

3. **Verify render mode for cameras**
   ```python
   # Camera sensors require rendering
   world.step(render=True)  # Not render=False
   ```

4. **Check sensor prim configuration in USD**
   - Camera: focal length, aperture, render products
   - LiDAR: scanning pattern, range
   - IMU: update rate, noise parameters

**Common Causes**:
- Sensor not initialized (need warm-up frames)
- Rendering disabled (`render=False`)
- Sensor prim misconfigured
- Reading sensor at wrong time (before render completes)

### "My callback isn't running"

**Symptoms**: You registered a callback but it never executes.

**Diagnostic Steps**:

1. **Verify registration succeeded**
   ```python
   # Physics callback
   def my_callback(dt):
       print(f"Callback called with dt={dt}")
   
   world.add_physics_callback("my_callback", my_callback)
   
   # Check if registered
   print(f"Callback registered: {'my_callback' in str(world._physics_callbacks)}")
   ```

2. **Check simulation state**
   ```python
   print(f"Playing: {world.is_playing()}")
   print(f"Stopped: {world.is_stopped()}")
   ```

3. **Verify timeline is running**
   ```python
   import omni.timeline
   timeline = omni.timeline.get_timeline_interface()
   print(f"Timeline playing: {timeline.is_playing()}")
   ```

4. **Check callback scope/lifetime**
   ```python
   # Common mistake: callback object goes out of scope
   class Controller:
       def __init__(self, world):
           # This subscription may be garbage collected
           self.sub = world.add_physics_callback("ctrl", self.step)
       
       def step(self, dt):
           pass
   
   # Fix: ensure object stays alive
   controller = Controller(world)  # Keep reference
   ```

**Common Causes**:
- Simulation not playing
- Callback registered to wrong event stream
- Callback object garbage collected
- Exception in callback silently caught

## Logging and Debugging

### Enable Verbose Logging

Isaac Sim logs to `~/.nvidia-omniverse/logs/`:

```bash
# Find recent logs
ls -lt ~/.nvidia-omniverse/logs/Kit/*/

# Search for errors
grep -r "ERROR\|Exception" ~/.nvidia-omniverse/logs/Kit/*/
```

### Python Debugging

For Python scripts:

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Or use carb logging
import carb
carb.log_info("Debug message")
carb.log_warn("Warning message")
carb.log_error("Error message")
```

### Physics Debugging Visualization

Enable physics debug visualization:

```python
from omni.physx import get_physx_visualization_interface

vis = get_physx_visualization_interface()
vis.set_visualization_parameter("Collision Shapes", True)
vis.set_visualization_parameter("Contact Points", True)
vis.set_visualization_parameter("Joint Limits", True)
```

This shows collision geometry, contact points, and joint axes in the viewport.

### Step-Through Debugging

For complex issues, step through simulation manually:

```python
# Pause simulation
world.pause()

# Step one physics step at a time
for i in range(10):
    print(f"Step {i}")
    print(f"  Position: {robot.get_world_pose()}")
    print(f"  Velocity: {robot.get_linear_velocity()}")
    world.step(render=True)
    input("Press Enter for next step...")
```

## Environment Isolation

When debugging, isolate to minimal reproduction:

### Minimal Scene Test

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create minimal world
world = World()
world.scene.add_default_ground_plane()

# Add single test object
cube = world.scene.add(DynamicCuboid(
    prim_path="/World/Cube",
    name="cube",
    position=[0, 0, 1],
    size=0.1
))

world.reset()

# Test
for i in range(100):
    world.step(render=True)
    print(f"Step {i}: pos={cube.get_world_pose()[0]}")

simulation_app.close()
```

If this works, the problem is in your scene complexity. If it doesn't, the problem is environmental.

### Binary Search for Scene Issues

When a complex scene fails:

1. Remove half the objects
2. Does it work now?
   - Yes: Problem is in removed half
   - No: Problem is in remaining half
3. Repeat until you find the problematic element

## Running Example

For our warehouse mobile manipulator, here's a diagnostic checklist:

### Pre-Flight Checks

Before running the simulation:

```python
def preflight_check(world, robot):
    checks = []
    
    # 1. World configuration
    checks.append(("Physics scene exists", 
                   world.physics_sim_view is not None))
    
    # 2. Robot articulation
    checks.append(("Robot is valid", 
                   robot.is_valid()))
    checks.append(("Robot has DOF", 
                   robot.num_dof > 0))
    
    # 3. Joint limits
    positions = robot.get_joint_positions()
    limits = robot.get_joint_limits()
    in_limits = all(lo <= p <= hi for p, (lo, hi) in zip(positions, limits))
    checks.append(("Joints within limits", in_limits))
    
    # 4. Sensors
    for sensor_name, sensor in robot.sensors.items():
        checks.append((f"Sensor {sensor_name} valid", sensor.is_valid()))
    
    # Report
    print("Pre-flight checks:")
    all_passed = True
    for name, passed in checks:
        status = "✓" if passed else "✗"
        print(f"  {status} {name}")
        if not passed:
            all_passed = False
    
    return all_passed
```

### Runtime Health Monitoring

During simulation:

```python
class HealthMonitor:
    def __init__(self, world, robot):
        self.world = world
        self.robot = robot
        self.world.add_physics_callback("health", self._check_health)
        self.warnings = []
    
    def _check_health(self, dt):
        # Check for NaN in positions
        pos = self.robot.get_joint_positions()
        if np.any(np.isnan(pos)):
            self.warnings.append("NaN in joint positions - physics exploded")
        
        # Check for extreme velocities
        vel = self.robot.get_joint_velocities()
        if np.any(np.abs(vel) > 100):
            self.warnings.append(f"Extreme velocity detected: {np.max(np.abs(vel))}")
        
        # Check for joint limit violations
        limits = self.robot.get_joint_limits()
        for i, (p, (lo, hi)) in enumerate(zip(pos, limits)):
            if p < lo - 0.1 or p > hi + 0.1:
                self.warnings.append(f"Joint {i} violating limits: {p}")
```

## Chapter Summary

Key diagnostic strategies:

- **Isolate the layer**: USD, physics, Python, or integration?
- **Minimize reproduction**: Smallest scene that exhibits the problem
- **Check the basics**: Physics enabled? Simulation playing? Seeds set?
- **Use built-in tools**: Physics visualization, logging, performance stats
- **Verify timing**: Callbacks running? Sensors reading fresh data?

With this section complete, you have the mental models to build and debug Isaac Sim simulations. The next section covers USD and scene structure.

## References

- [Isaac Sim Troubleshooting](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/troubleshooting.html) — Common issues and solutions
- [PhysX Visual Debugger](https://nvidia-omniverse.github.io/PhysX/physx/5.4.2/docs/VisualDebugger.html) — Physics debugging visualization
- [Omniverse Logging](https://docs.omniverse.nvidia.com/kit/docs/kit-sdk/latest/guide/logging.html) — Carb logging system
