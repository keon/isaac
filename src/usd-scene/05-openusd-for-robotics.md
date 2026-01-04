# OpenUSD for Robotics

## Overview

Universal Scene Description (USD) is the foundation of scene representation in Isaac Sim. Originally developed by Pixar for film production, USD provides a hierarchical, composable scene format that enables non-destructive editing, asset reuse, and collaborative workflows.

For robotics, USD offers capabilities that traditional formats like URDF or SDF cannot match:
- **Non-destructive composition**: Override properties without modifying source files
- **Layered editing**: Stack modifications in separate layers
- **Asset references**: Reuse robots and environments across projects
- **Variant sets**: Bundle multiple configurations in a single asset
- **Live collaboration**: Multiple users editing simultaneously via Nucleus

Understanding USD's composition model is essential for building maintainable simulation scenes.

## Core Concepts

### Prims and Properties

The USD scene graph consists of **prims** (primitives) organized in a hierarchy:

```
/World
├── /World/GroundPlane
├── /World/Robot
│   ├── /World/Robot/base_link
│   ├── /World/Robot/shoulder_link
│   └── /World/Robot/arm_link
└── /World/Environment
    ├── /World/Environment/Shelf_01
    └── /World/Environment/Shelf_02
```

Each prim has:
- **Type**: What kind of prim (Xform, Mesh, Camera, etc.)
- **Properties**: Attributes (values) and relationships (connections to other prims)
- **Metadata**: Information about the prim itself

```python
from pxr import Usd, UsdGeom

stage = Usd.Stage.Open("scene.usd")
prim = stage.GetPrimAtPath("/World/Robot")

# Prim type
print(prim.GetTypeName())  # e.g., "Xform"

# Get an attribute
xform = UsdGeom.Xformable(prim)
translate_op = xform.GetOrderedXformOps()[0]
print(translate_op.Get())  # Translation value
```

### Schemas

Schemas define what properties a prim can have. Isaac Sim uses several schema types:

**Geometry schemas** (from UsdGeom):
- `Xform`: Transform node
- `Mesh`: Polygonal geometry
- `Cube`, `Sphere`, `Cylinder`: Primitive shapes

**Physics schemas** (from UsdPhysics):
- `RigidBodyAPI`: Makes a prim a rigid body
- `CollisionAPI`: Adds collision behavior
- `ArticulationRootAPI`: Root of an articulated system
- `Joint`: Base class for joints

**Isaac-specific schemas**:
- `RobotAPI`: Organizes robot structure
- `IsaacSensorAPI`: Base for sensor prims

```python
from pxr import UsdPhysics

# Check if a prim has physics
prim = stage.GetPrimAtPath("/World/Robot/base_link")
if prim.HasAPI(UsdPhysics.RigidBodyAPI):
    rigid_body = UsdPhysics.RigidBodyAPI(prim)
    print(f"Rigid body enabled: {rigid_body.GetRigidBodyEnabledAttr().Get()}")
```

### Layers and Opinions

USD scenes are built from **layers**—individual files that contribute data. When multiple layers define the same property, USD resolves conflicts using **opinion strength**.

```
scene.usd (root layer)
├── robot.usd (sublayer)
├── environment.usd (sublayer)  
└── overrides.usd (sublayer - strongest opinions)
```

The layer listed last in sublayers has the strongest opinions (wins conflicts).

```python
# Working with layers
stage = Usd.Stage.Open("scene.usd")
root_layer = stage.GetRootLayer()

# List sublayers
for path in root_layer.subLayerPaths:
    print(f"Sublayer: {path}")

# Add a new sublayer (will have strongest opinions)
root_layer.subLayerPaths.append("my_overrides.usd")
```

## Composition Arcs

Composition arcs define how USD combines data from different sources. Understanding the **LIVRPS** strength ordering is essential:

1. **Local** (direct opinions in the layer)
2. **Inherits** (class-based inheritance)
3. **Variant Sets** (switchable variations)
4. **References** (external asset inclusion)
5. **Payloads** (deferred loading references)
6. **Specializes** (refinement relationship)

### Sublayers

Sublayers stack layers like Photoshop layers. Later sublayers override earlier ones:

```python
# Create a scene with sublayers
from pxr import Sdf

# Base layer
base_layer = Sdf.Layer.CreateNew("base.usd")
base_stage = Usd.Stage.Open(base_layer)
robot = base_stage.DefinePrim("/Robot", "Xform")

# Override layer
override_layer = Sdf.Layer.CreateNew("overrides.usd")

# Compose them
root_layer = Sdf.Layer.CreateNew("scene.usd")
root_layer.subLayerPaths = ["base.usd", "overrides.usd"]
```

**Use sublayers for**: Project-specific overrides, user preferences, runtime modifications.

### References

References pull in external USD files. The referenced content appears as if it were part of the current scene:

```python
# Reference an external robot
stage = Usd.Stage.CreateNew("scene.usd")
robot_prim = stage.DefinePrim("/World/Robot")
robot_prim.GetReferences().AddReference("./assets/franka.usd")

# The robot's contents now appear under /World/Robot
```

**Use references for**: Reusable assets (robots, objects, environments).

### Payloads

Payloads are like references but can be loaded/unloaded on demand:

```python
# Add a payload (deferred loading)
environment_prim = stage.DefinePrim("/World/Environment")
environment_prim.GetPayloads().AddPayload("./assets/warehouse.usd")

# Control loading
stage.Load("/World/Environment")    # Load the payload
stage.Unload("/World/Environment")  # Unload to save memory
```

**Use payloads for**: Heavy assets, optional scene components, memory management.

### Variant Sets

Variant sets bundle multiple versions of an asset:

```python
# Create variants for a gripper
gripper = stage.DefinePrim("/World/Robot/Gripper")
vset = gripper.GetVariantSets().AddVariantSet("gripper_type")

# Add variants
vset.AddVariant("parallel_jaw")
vset.AddVariant("suction")
vset.AddVariant("magnetic")

# Select a variant
vset.SetVariantSelection("parallel_jaw")

# Author content within a variant
with vset.GetVariantEditContext():
    # This content only exists when "parallel_jaw" is selected
    stage.DefinePrim("/World/Robot/Gripper/LeftFinger", "Mesh")
    stage.DefinePrim("/World/Robot/Gripper/RightFinger", "Mesh")
```

**Use variant sets for**: Gripper types, robot configurations, environment layouts.

### Inherits

Inherits create class-based relationships where derived prims receive base prim opinions:

```python
# Define a class (template)
stage.DefinePrim("/_class_/RigidBox", "Cube")
class_prim = stage.GetPrimAtPath("/_class_/RigidBox")
UsdPhysics.RigidBodyAPI.Apply(class_prim)
UsdPhysics.CollisionAPI.Apply(class_prim)

# Derive instances that inherit from the class
box1 = stage.DefinePrim("/World/Box1", "Cube")
box1.GetInherits().AddInherit("/_class_/RigidBox")

box2 = stage.DefinePrim("/World/Box2", "Cube")
box2.GetInherits().AddInherit("/_class_/RigidBox")

# Changes to /_class_/RigidBox automatically propagate to Box1 and Box2
```

**Use inherits for**: Shared physics properties, material templates, sensor configurations.

## Robot Structure in USD

Isaac Sim expects robots to follow a specific structure for physics simulation.

### Articulation Hierarchy

An articulated robot has:
- **ArticulationRootAPI** on the root prim
- **RigidBodyAPI** on each link
- **Joint prims** connecting links

```
/Robot (ArticulationRootAPI)
├── /Robot/base_link (RigidBodyAPI, CollisionAPI)
├── /Robot/shoulder_joint (RevoluteJoint)
├── /Robot/shoulder_link (RigidBodyAPI, CollisionAPI)
├── /Robot/elbow_joint (RevoluteJoint)
└── /Robot/forearm_link (RigidBodyAPI, CollisionAPI)
```

### Applying Robot Schema

For robots imported via URDF, schemas are applied automatically. For manual setup:

```python
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

# Get robot root
robot = stage.GetPrimAtPath("/World/Robot")

# Apply ArticulationRootAPI
UsdPhysics.ArticulationRootAPI.Apply(robot)

# Apply RigidBodyAPI to each link
for link_path in ["/World/Robot/base_link", "/World/Robot/arm_link"]:
    link = stage.GetPrimAtPath(link_path)
    UsdPhysics.RigidBodyAPI.Apply(link)
    UsdPhysics.CollisionAPI.Apply(link)
```

### Joint Configuration

Joints connect bodies and define motion constraints:

```python
from pxr import UsdPhysics, Gf

# Create a revolute joint
joint_path = "/World/Robot/shoulder_joint"
joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)

# Set connected bodies
joint.CreateBody0Rel().SetTargets(["/World/Robot/base_link"])
joint.CreateBody1Rel().SetTargets(["/World/Robot/shoulder_link"])

# Set axis
joint.CreateAxisAttr("Z")

# Set limits (in degrees for revolute)
joint.CreateLowerLimitAttr(-180)
joint.CreateUpperLimitAttr(180)

# Add drive
drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(joint_path), "angular")
drive.CreateTypeAttr("force")
drive.CreateStiffnessAttr(1000)
drive.CreateDampingAttr(100)
```

## Working with the Stage

### Stage Lifecycle

```python
import omni.usd

# Get current stage
stage = omni.usd.get_context().get_stage()

# Create new stage
omni.usd.get_context().new_stage()

# Open existing stage
omni.usd.get_context().open_stage("path/to/scene.usd")

# Save stage
omni.usd.get_context().save_stage()
```

### Traversing the Scene

```python
from pxr import Usd, UsdGeom

stage = omni.usd.get_context().get_stage()

# Iterate all prims
for prim in stage.Traverse():
    print(f"{prim.GetPath()}: {prim.GetTypeName()}")

# Find specific types
meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]

# Get children
robot = stage.GetPrimAtPath("/World/Robot")
for child in robot.GetChildren():
    print(child.GetPath())
```

### Modifying Properties

```python
from pxr import Gf

# Set transform
xform = UsdGeom.Xformable(prim)
xform.ClearXformOpOrder()
xform.AddTranslateOp().Set(Gf.Vec3d(1.0, 0.0, 0.5))
xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 45))
xform.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))

# Set physics property
rigid_body = UsdPhysics.RigidBodyAPI(prim)
rigid_body.CreateVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
```

## Running Example

For our warehouse mobile manipulator, we'll structure the USD scene for maintainability:

### Scene Structure

```
warehouse_scene.usd (root)
├── sublayers:
│   ├── environment/warehouse_base.usd
│   ├── robots/mobile_manipulator.usd
│   └── overrides/simulation_config.usd
│
└── /World
    ├── /World/Environment (payload → warehouse.usd)
    │   ├── /World/Environment/Floor
    │   ├── /World/Environment/Shelves
    │   └── /World/Environment/Lighting
    │
    └── /World/Robot (reference → mobile_manipulator.usd)
        ├── /World/Robot/base (ArticulationRootAPI)
        ├── /World/Robot/arm
        └── /World/Robot/gripper (variant set: gripper_type)
```

### Creating the Scene Programmatically

```python
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

def create_warehouse_scene():
    # Create root stage
    stage = Usd.Stage.CreateNew("warehouse_scene.usd")
    
    # Set stage metadata
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    # Create world root
    world = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(world)
    
    # Add physics scene
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    
    # Reference the robot
    robot = stage.DefinePrim("/World/Robot", "Xform")
    robot.GetReferences().AddReference("./robots/mobile_manipulator.usd")
    
    # Payload the environment (can be unloaded)
    environment = stage.DefinePrim("/World/Environment", "Xform")
    environment.GetPayloads().AddPayload("./environments/warehouse.usd")
    
    stage.Save()
    return stage
```

## Chapter Summary

Key takeaways:

- **Prims and schemas**: USD scenes are hierarchies of typed prims with properties
- **LIVRPS composition**: Local > Inherits > Variants > References > Payloads > Specializes
- **Sublayers for overrides**: Stack layers for non-destructive editing
- **References for reuse**: Pull in external assets without copying
- **Payloads for performance**: Defer loading of heavy assets
- **Variant sets for configurations**: Bundle multiple versions in one asset

Next: [Transforms, Units, and Coordinates](./06-transforms-units-coordinates.md)—the details that cause subtle bugs when mismatched.

## References

- [OpenUSD Documentation](https://openusd.org/release/index.html) — Official USD specification and tutorials
- [NVIDIA Learn OpenUSD](https://docs.nvidia.com/learn-openusd/latest/) — Composition arcs and best practices
- [Isaac Sim USD Overview](https://docs.omniverse.nvidia.com/isaacsim/latest/open_usd.html) — Isaac Sim-specific USD usage
- [UsdPhysics Schema](https://openusd.org/release/api/usd_physics_page_front.html) — Physics schema documentation
