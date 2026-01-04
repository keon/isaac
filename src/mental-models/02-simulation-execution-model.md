# Simulation Execution Model

## Overview

Isaac Sim orchestrates multiple subsystems that each have their own update cycles:

1. **Physics simulation**: Advances the physical state of the world
2. **Rendering**: Generates visual output (viewport, cameras)
3. **USD stage updates**: Synchronizes scene graph changes
4. **Extension callbacks**: User-registered code execution points
5. **Sensor processing**: Computes sensor outputs from world state

These systems don't run in lockstep. Understanding their relationships—and where your code executes—determines whether your simulation behaves predictably.

## The Core Loop

Isaac Sim runs an event-driven loop managed by Omniverse Kit. Within a single frame:

```
┌─────────────────────────────────────────────────────────────┐
│                     SIMULATION FRAME                        │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐       │
│  │ Pre-physics │ → │  Physics    │ → │Post-physics │       │
│  │  callbacks  │   │  step(s)    │   │  callbacks  │       │
│  └─────────────┘   └─────────────┘   └─────────────┘       │
│         │                                   │               │
│         ▼                                   ▼               │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐       │
│  │ USD stage   │ → │ Pre-render  │ → │  Rendering  │       │
│  │    sync     │   │  callbacks  │   │             │       │
│  └─────────────┘   └─────────────┘   └─────────────┘       │
└─────────────────────────────────────────────────────────────┘
```

**Key insight**: Physics and rendering run at different rates. Multiple physics steps can occur per rendered frame.

### Decoupled Physics and Rendering

Isaac Sim allows physics and rendering to operate at different frequencies:

- **Physics timestep**: Often 1/60s, 1/120s, or 1/240s (fixed)
- **Render rate**: Variable, depends on scene complexity and GPU load

This decoupling means:

- Multiple physics steps can occur per rendered frame
- Or rendering can happen without a physics step (when paused)
- Or neither can happen (when the simulation is stopped)

Your callbacks need to understand which cycle they're attached to.

## Operational Modes

Isaac Sim operates in fundamentally different modes that change the execution model.

### Standalone Python Mode

When running Isaac Sim from a Python script (not through the GUI), you have explicit control:

```python
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# Your setup code here

while simulation_app.is_running():
    # Manually step simulation
    world.step(render=True)
    
    # Your per-step logic here

simulation_app.close()
```

In this mode:

- **You control when steps happen**: `world.step()` advances simulation
- **Physics and rendering are synchronous**: Each `step()` completes before your code continues
- **Callbacks still execute**: But within your explicit step call

This is the most predictable mode for control applications.

### Extension Mode (GUI)

When running in the Isaac Sim GUI application, execution is asynchronous:

- The application manages the main loop
- Play/Pause/Stop buttons control simulation state
- Your code runs via registered callbacks
- Multiple extensions may register callbacks that interleave

This mode is interactive but less predictable—your code doesn't control when it runs.

### Headless Mode

For CI/CD and training workloads, headless mode removes rendering overhead:

```python
simulation_app = SimulationApp({"headless": True})
```

In headless mode:

- No viewport rendering occurs
- Camera sensors still work (they render offscreen)
- Physics proceeds at maximum speed
- GUI callbacks may not execute

## Callback Registration

Isaac Sim provides several callback attachment points. Choosing the right one determines when your code executes.

### Physics Callbacks

Execute in sync with physics steps:

```python
def on_physics_step(step_size: float):
    # This runs every physics step
    # step_size is the physics timestep (e.g., 1/60)
    current_position = robot.get_world_pose()
    robot.apply_action(compute_control(current_position))

world.add_physics_callback("my_controller", on_physics_step)
```

**Use for**: Control loops, force applications, state queries that need physics-rate updates.

**Guarantee**: Executes once per physics step, with consistent `step_size`.

### Render Callbacks

Execute when rendering occurs:

```python
def on_render(event):
    # This runs when the viewport updates
    # Rate depends on GPU load and scene complexity
    update_visualization()

app.get_render_event_stream().create_subscription_to_pop(on_render)
```

**Use for**: Visualization updates, non-physics UI updates.

**Warning**: Rate is variable. Don't use for control loops.

### Timeline Callbacks

Execute based on simulation time (as shown in the timeline):

```python
def on_timeline_event(event):
    if event.type == int(omni.timeline.TimelineEventType.PLAY):
        print("Simulation started")
    elif event.type == int(omni.timeline.TimelineEventType.STOP):
        print("Simulation stopped")
```

**Use for**: Reacting to play/pause/stop state changes.

### Stage Event Callbacks

Execute when the USD stage changes:

```python
def on_stage_event(event):
    if event.type == int(omni.usd.StageEventType.OPENED):
        print("New stage opened")

stage_event_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(on_stage_event)
```

**Use for**: Responding to scene loading, asset changes.

## Timing Relationships

Understanding timing relationships prevents subtle bugs.

### Physics Time vs Wall Time

Physics time is *simulation* time. Wall time is *real* time. They can diverge:

```
Physics time:  0.0 → 0.016 → 0.033 → 0.050 → ...
Wall time:     0.0 → 0.020 → 0.045 → 0.048 → ...
```

If simulation runs faster than real-time (simple scene, powerful GPU), physics time advances faster than wall time.

If simulation runs slower than real-time (complex scene, weak GPU), physics time lags behind.

**Implication**: Never use wall-clock time for control logic. Use simulation time:

```python
# Wrong: wall time
import time
last_time = time.time()

# Right: simulation time
current_time = world.current_time
```

### Physics Step Size vs Render Rate

The physics timestep is typically fixed (configured in stage settings):

```python
# Physics at 120 Hz
physics_dt = 1.0 / 120.0  # 0.00833 seconds per step
```

The render rate varies. If rendering takes 20ms, you get ~50 FPS. But physics still advances at 120 Hz—meaning 2-3 physics steps occur per rendered frame.

**Implication**: A physics callback runs 120 times per second of sim time, regardless of frame rate. A render callback runs ~50 times per second of wall time (in this example).

### Sensor Latency

Sensors don't update instantaneously:

- **Cameras**: Render in the render pass, available after rendering completes
- **LiDAR**: Typically updates at a configurable rate, not every physics step
- **IMU/Force sensors**: Can update at physics rate

If you query camera data in a physics callback, you may get stale data from the previous render. Structure your code to account for this:

```python
def on_physics_step(dt):
    # Physics state is current
    position = robot.get_world_pose()  # Fresh
    
    # Camera data may be from previous frame
    rgb = camera.get_rgb()  # Potentially stale

def on_render(event):
    # Camera data is fresh here
    rgb = camera.get_rgb()  # Fresh after this render
```

## Extension Load Order

When Isaac Sim starts, extensions load in a dependency-determined order. This affects:

- Which callbacks exist when your code runs
- Whether APIs you call are initialized
- What scene state exists

### Startup Sequence

Approximate order:

1. Omniverse Kit core loads
2. USD/Hydra subsystems initialize
3. PhysX extension loads
4. Isaac Sim core extensions load
5. User/custom extensions load
6. Stage opens (if specified)
7. `setup_scene()` / initialization callbacks run

**Common mistake**: Trying to access physics APIs before PhysX is initialized. Use proper lifecycle callbacks:

```python
class MyExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        # Extension loaded, but stage may not be ready
        # Don't query scene objects here
        
        # Register for stage ready event
        self._stage_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event
        )
    
    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.OPENED):
            # Now safe to query stage
            self._setup_scene()
```

## The World Object

Isaac Sim's `World` class centralizes simulation management:

```python
from omni.isaac.core import World

world = World(stage_units_in_meters=1.0, physics_dt=1/120, rendering_dt=1/60)
```

The `World` provides:

- Scene management (adding/removing objects)
- Stepping control (`step()`, `reset()`)
- Callback registration
- Time queries

### Step Variants

The `step()` method has options:

```python
# Step physics and render
world.step(render=True)

# Step physics only (faster for training)
world.step(render=False)

# Step physics multiple times, then render once
for _ in range(4):
    world.step(render=False)
world.render()  # Render once after multiple physics steps
```

For training workloads, skipping rendering dramatically improves throughput.

## Common Pitfalls

### Pitfall 1: Control in Render Callbacks

```python
# Wrong: variable rate control
def on_render(event):
    error = target - robot.get_position()
    robot.apply_force(error * kp)  # Rate varies with frame rate!
```

The control rate depends on rendering speed. On a fast GPU, this might run at 90 Hz. On a slow GPU, 30 Hz. Physics doesn't change, but your control authority does.

**Fix**: Use physics callbacks for control.

### Pitfall 2: Assuming Single-Step Execution

```python
# Wrong: assuming one physics step per step() call
world.step()
position = robot.get_world_pose()
# What if rendering_dt / physics_dt means multiple physics steps ran?
```

If `physics_dt=1/120` and `rendering_dt=1/60`, each `step(render=True)` runs 2 physics steps.

**Fix**: Use physics callbacks for per-physics-step logic, or query step counts explicitly.

### Pitfall 3: Synchronous Expectations in Async Mode

In extension mode, code like this doesn't work as expected:

```python
def do_something():
    robot.set_position([1, 0, 0])
    # Position isn't updated yet! USD changes are queued.
    pos = robot.get_position()  # Still the old position
```

USD operations may be deferred. Force synchronization if needed:

```python
def do_something():
    robot.set_position([1, 0, 0])
    omni.kit.app.get_app().update()  # Process pending updates
    pos = robot.get_position()  # Now updated
```

### Pitfall 4: Not Accounting for Warm-Up

The first few simulation steps often behave differently:

- Physics solver needs time to converge on contacts
- Articulations may have initial constraint violations
- Sensors may need a frame to initialize

**Fix**: Run a few "warm-up" steps before collecting data or starting control:

```python
# Warm-up
for _ in range(10):
    world.step()

# Now run real simulation
```

## Running Example

For our warehouse mobile manipulator, the execution model matters in several ways:

### Control Architecture

We'll structure control to run at physics rate:

```python
class MobileManipulatorController:
    def __init__(self, world, robot):
        self.world = world
        self.robot = robot
        self.world.add_physics_callback("controller", self._control_step)
    
    def _control_step(self, dt):
        # This runs at physics rate (e.g., 120 Hz)
        base_vel = self._compute_base_velocity()
        arm_torques = self._compute_arm_control()
        
        self.robot.apply_base_velocity(base_vel)
        self.robot.apply_joint_torques(arm_torques)
```

### Perception Pipeline

Camera data will be processed at render rate, which is fine for high-level perception:

```python
class PerceptionSystem:
    def __init__(self, world, camera):
        self.camera = camera
        self.last_detection = None
        # Register at render rate - cameras update after rendering
        app.get_render_event_stream().create_subscription_to_pop(self._on_render)
    
    def _on_render(self, event):
        # Process camera data after rendering
        rgb = self.camera.get_rgb()
        self.last_detection = self._detect_objects(rgb)
```

### Timing Coordination

The control loop will query perception results without blocking:

```python
def _control_step(self, dt):
    # Get latest detection (may be 1-2 frames old)
    if self.perception.last_detection is not None:
        target = self.perception.last_detection.position
        self._navigate_to(target)
```

This pattern—control at physics rate, perception at render rate, with asynchronous handoff—reflects real robot architectures where sensing and control operate at different frequencies.

## Chapter Summary

Key takeaways:

- **Physics and rendering are decoupled**: They run at different rates
- **Use physics callbacks for control**: Consistent rate, unlike render callbacks
- **Sensor data has latency**: Cameras update after rendering
- **Standalone mode is predictable**: You control step timing
- **Extension mode is asynchronous**: Callbacks execute when the system decides

Next: [Physics Stepping and Determinism](./03-physics-stepping-determinism.md)—what guarantees PhysX provides.

## References

- [Isaac Sim Core API Tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html) — Physics callbacks and World class usage
- [Simulation Context](https://isaac-sim.github.io/IsaacLab/main/source/api/isaaclab.sim.html) — Isaac Lab simulation context documentation
- [Omniverse Kit SDK](https://docs.omniverse.nvidia.com/kit/docs/kit-sdk/latest/) — Extension lifecycle and event streams
