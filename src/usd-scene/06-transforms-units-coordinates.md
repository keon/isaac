# Transforms, Units, and Coordinates

## Overview

Unit and coordinate mismatches are among the most common sources of bugs in Isaac Sim. A robot that's 1000x too large, physics that explode immediately, sensors pointing the wrong direction—these issues waste hours of debugging and often trace back to inconsistent conventions.

Isaac Sim has specific defaults:
- **Units**: Meters
- **Up axis**: Z-up (right-handed coordinate system)
- **Rotation order**: XYZ for Euler angles

Assets from other tools (Blender, Maya, CAD software) often use different conventions. This chapter covers how to identify, diagnose, and fix these mismatches.

## Stage-Level Settings

Every USD stage has metadata defining its units and coordinate system.

### Meters Per Unit

The `metersPerUnit` stage metadata defines the scale:

| metersPerUnit | Interpretation |
|---------------|----------------|
| 1.0 | 1 USD unit = 1 meter |
| 0.01 | 1 USD unit = 1 centimeter |
| 0.001 | 1 USD unit = 1 millimeter |

```python
from pxr import UsdGeom

stage = omni.usd.get_context().get_stage()

# Check current setting
meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
print(f"Meters per unit: {meters_per_unit}")

# Set to meters
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
```

**Common sources of mismatch**:
- Blender exports: Often centimeters (0.01)
- CAD software: Often millimeters (0.001)
- Maya: Can be centimeters or other units
- URDF: Always meters (by specification)

### Up Axis

The `upAxis` metadata defines which axis points up:

```python
# Check up axis
up_axis = UsdGeom.GetStageUpAxis(stage)
print(f"Up axis: {up_axis}")  # "Y" or "Z"

# Set to Z-up (Isaac Sim convention)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
```

**Common conventions**:
- Isaac Sim / robotics: Z-up
- USD default: Y-up
- Blender: Z-up
- Maya / most DCC tools: Y-up

### Checking Stage Settings

```python
def check_stage_settings(stage):
    """Print stage unit and coordinate settings."""
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    up_axis = UsdGeom.GetStageUpAxis(stage)
    
    print(f"Stage settings:")
    print(f"  Meters per unit: {meters_per_unit}")
    print(f"  Up axis: {up_axis}")
    
    # Warnings
    if meters_per_unit != 1.0:
        print(f"  ⚠ Not in meters! Scale factor needed: {1.0/meters_per_unit}")
    if up_axis != UsdGeom.Tokens.z:
        print(f"  ⚠ Not Z-up! May need coordinate transform")
```

## Transform Operations

USD transforms use xformOps—a stack of operations applied in order.

### Transform Order

```python
from pxr import UsdGeom, Gf

prim = stage.GetPrimAtPath("/World/Robot")
xform = UsdGeom.Xformable(prim)

# Get current transform operations
for op in xform.GetOrderedXformOps():
    print(f"{op.GetOpName()}: {op.Get()}")
```

Common xformOp types:
- `xformOp:translate`: Translation (Vec3)
- `xformOp:rotateXYZ`: Euler rotation in degrees
- `xformOp:rotateX`, `rotateY`, `rotateZ`: Single-axis rotation
- `xformOp:orient`: Quaternion rotation
- `xformOp:scale`: Scale (Vec3)
- `xformOp:transform`: Full 4x4 matrix

### Setting Transforms

```python
# Clear existing transforms and set new ones
xform.ClearXformOpOrder()

# Translation (in stage units)
translate_op = xform.AddTranslateOp()
translate_op.Set(Gf.Vec3d(1.0, 2.0, 0.5))

# Rotation (Euler angles in degrees)
rotate_op = xform.AddRotateXYZOp()
rotate_op.Set(Gf.Vec3f(0, 0, 90))  # 90° around Z

# Scale
scale_op = xform.AddScaleOp()
scale_op.Set(Gf.Vec3f(1.0, 1.0, 1.0))
```

### World vs Local Transforms

```python
# Get world transform (accumulated from parents)
world_xform = UsdGeom.XformCache()
world_matrix = world_xform.GetLocalToWorldTransform(prim)

# Extract translation from world transform
translation = world_matrix.ExtractTranslation()
print(f"World position: {translation}")

# Get local transform (just this prim)
local_matrix = xform.GetLocalTransformation()
```

### Quaternion Rotations

For physics and robotics, quaternions avoid gimbal lock:

```python
from pxr import Gf

# Create quaternion (w, x, y, z)
# 90° rotation around Z axis
quat = Gf.Quatf(0.7071, 0.0, 0.0, 0.7071)

# Apply as orient op
xform.ClearXformOpOrder()
translate_op = xform.AddTranslateOp()
orient_op = xform.AddOrientOp()
orient_op.Set(quat)
```

## Metrics Assembler

Isaac Sim includes an automatic unit conversion system called the **Metrics Assembler**.

### How It Works

When enabled, the Metrics Assembler automatically adjusts referenced assets to match the stage's units and up-axis:

1. Detects `metersPerUnit` and `upAxis` of referenced files
2. Applies compensating transforms to align with the root stage
3. Handles mass unit conversion for physics

### Enabling/Disabling

```python
import omni.kit.commands

# Check if enabled
from omni.kit.property.usd.usd_property_widget import MetricsAssemblerWidget
# The setting is typically in Edit > Preferences > Stage

# Or via settings
import carb.settings
settings = carb.settings.get_settings()
enabled = settings.get("/persistent/app/stage/metricsAssembler/enabled")
```

### When Metrics Assembler Causes Issues

The assembler can cause unexpected behavior when:
- Assets have incorrect metadata (claims meters but is actually centimeters)
- Physics properties (mass, inertia) need manual adjustment
- You want explicit control over transforms

**To disable for a specific reference**:

```python
# Add reference without metrics assembly
ref = prim.GetReferences().AddReference(
    assetPath="./asset.usd",
    primPath="/Root"
)
# Then manually handle unit conversion
```

## Common Unit Problems

### Problem: Robot is Huge or Tiny

**Symptom**: Robot appears 100x or 1000x wrong size.

**Diagnosis**:
```python
def diagnose_scale(robot_prim):
    stage = robot_prim.GetStage()
    
    # Check stage units
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    print(f"Stage metersPerUnit: {mpu}")
    
    # Get bounding box
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    bbox = bbox_cache.ComputeWorldBound(robot_prim)
    size = bbox.ComputeAlignedRange().GetSize()
    print(f"Bounding box size: {size} (in stage units)")
    print(f"Actual size in meters: {size * mpu}")
```

**Fix options**:

1. **Fix at source**: Re-export with correct units
2. **Scale on reference**: 
   ```python
   # Apply scale when referencing
   robot = stage.DefinePrim("/World/Robot", "Xform")
   robot.GetReferences().AddReference("./robot_in_cm.usd")
   xform = UsdGeom.Xformable(robot)
   xform.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))  # cm → m
   ```
3. **Fix stage metadata**: If everything is consistently in wrong units

### Problem: Robot is Sideways or Upside Down

**Symptom**: Robot lies on its side or is inverted.

**Diagnosis**:
```python
def diagnose_orientation(stage):
    up_axis = UsdGeom.GetStageUpAxis(stage)
    print(f"Stage up axis: {up_axis}")
    
    # Check referenced file's up axis
    # (requires opening the referenced layer)
```

**Fix options**:

1. **Rotate on import**:
   ```python
   robot = stage.DefinePrim("/World/Robot", "Xform")
   robot.GetReferences().AddReference("./robot_y_up.usd")
   xform = UsdGeom.Xformable(robot)
   # Rotate 90° around X to convert Y-up to Z-up
   xform.AddRotateXOp().Set(-90.0)
   ```

2. **Use Metrics Assembler**: Enable automatic up-axis conversion

3. **Fix at source**: Re-export with Z-up

### Problem: Physics Explodes Immediately

**Symptom**: Objects fly apart or vibrate violently on first frame.

**Causes**:
- Objects interpenetrating due to scale mismatch
- Collision geometry at wrong scale
- Mass/inertia inconsistent with geometry scale

**Diagnosis**:
```python
def check_physics_scale(prim):
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        mass_api = UsdPhysics.MassAPI(prim)
        if mass_api:
            mass = mass_api.GetMassAttr().Get()
            print(f"Mass: {mass} kg")
            
    # Check collision geometry bounds
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        bbox = bbox_cache.ComputeWorldBound(prim)
        print(f"Collision bounds: {bbox.ComputeAlignedRange()}")
```

**Fix**: Ensure collision geometry, mass, and inertia all use consistent units.

## Coordinate Frame Conventions

### Isaac Sim Conventions

Isaac Sim uses:
- **World frame**: Z-up, right-handed
- **Robot base frame**: Typically at robot origin, Z-up
- **Camera frame**: -Z forward, Y up (OpenGL convention)
- **Sensor frames**: Vary by sensor type

### Robot Frame (REP 103/105)

ROS uses:
- X forward
- Y left  
- Z up

When integrating with ROS, ensure transforms align with these conventions.

### Camera Frames

Isaac Sim cameras follow OpenGL conventions:
- -Z: Forward (into the scene)
- +Y: Up
- +X: Right

```python
# Camera looking forward along robot's X axis needs rotation
camera_prim = stage.DefinePrim("/World/Robot/Camera", "Camera")
xform = UsdGeom.Xformable(camera_prim)

# Rotate to align camera -Z with robot +X
# This is a common transform needed for forward-facing cameras
xform.AddRotateYOp().Set(90)  # Point -Z toward +X
```

## Validation Utilities

### Comprehensive Check Script

```python
def validate_scene_units(stage):
    """Validate stage units and identify potential issues."""
    issues = []
    
    # Stage-level checks
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up_axis = UsdGeom.GetStageUpAxis(stage)
    
    if mpu != 1.0:
        issues.append(f"Stage not in meters (metersPerUnit={mpu})")
    
    if up_axis != UsdGeom.Tokens.z:
        issues.append(f"Stage not Z-up (upAxis={up_axis})")
    
    # Check referenced files
    for prim in stage.Traverse():
        refs = prim.GetMetadata("references")
        if refs:
            for ref in refs.GetAddedOrExplicitItems():
                ref_path = ref.assetPath
                try:
                    ref_layer = Sdf.Layer.FindOrOpen(ref_path)
                    if ref_layer:
                        ref_stage = Usd.Stage.Open(ref_layer)
                        ref_mpu = UsdGeom.GetStageMetersPerUnit(ref_stage)
                        ref_up = UsdGeom.GetStageUpAxis(ref_stage)
                        
                        if ref_mpu != mpu:
                            issues.append(
                                f"{prim.GetPath()}: Reference {ref_path} "
                                f"has different units (mpu={ref_mpu})"
                            )
                        if ref_up != up_axis:
                            issues.append(
                                f"{prim.GetPath()}: Reference {ref_path} "
                                f"has different up axis ({ref_up})"
                            )
                except:
                    issues.append(f"Could not open reference: {ref_path}")
    
    return issues

# Usage
issues = validate_scene_units(stage)
for issue in issues:
    print(f"⚠ {issue}")
```

### Physics Scale Validator

```python
def validate_physics_scale(stage):
    """Check for physics objects at unusual scales."""
    issues = []
    
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
            
        # Get world-space bounding box
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        bbox = bbox_cache.ComputeWorldBound(prim)
        size = bbox.ComputeAlignedRange().GetSize()
        max_dim = max(size[0], size[1], size[2])
        
        # Check for unusually small or large objects
        if max_dim < 0.001:  # Less than 1mm
            issues.append(f"{prim.GetPath()}: Very small ({max_dim:.6f}m)")
        elif max_dim > 100:  # Larger than 100m
            issues.append(f"{prim.GetPath()}: Very large ({max_dim:.1f}m)")
            
        # Check mass if available
        mass_api = UsdPhysics.MassAPI(prim)
        if mass_api:
            mass = mass_api.GetMassAttr().Get()
            if mass and max_dim > 0:
                density = mass / (max_dim ** 3)  # Rough density estimate
                if density < 10:  # Much lighter than water
                    issues.append(f"{prim.GetPath()}: Low density ({density:.1f} kg/m³)")
                elif density > 20000:  # Denser than gold
                    issues.append(f"{prim.GetPath()}: High density ({density:.1f} kg/m³)")
    
    return issues
```

## Running Example

For our warehouse scene, we need consistent units across all assets.

### Asset Import Checklist

```python
def import_asset_with_validation(stage, asset_path, target_path):
    """Import an asset with unit validation and correction."""
    
    # Open referenced asset to check units
    ref_layer = Sdf.Layer.FindOrOpen(asset_path)
    ref_stage = Usd.Stage.Open(ref_layer)
    
    ref_mpu = UsdGeom.GetStageMetersPerUnit(ref_stage)
    ref_up = UsdGeom.GetStageUpAxis(ref_stage)
    
    stage_mpu = UsdGeom.GetStageMetersPerUnit(stage)
    stage_up = UsdGeom.GetStageUpAxis(stage)
    
    # Create prim and add reference
    prim = stage.DefinePrim(target_path, "Xform")
    prim.GetReferences().AddReference(asset_path)
    
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    
    # Apply scale correction if needed
    if ref_mpu != stage_mpu:
        scale_factor = ref_mpu / stage_mpu
        print(f"Applying scale correction: {scale_factor}")
        xform.AddScaleOp().Set(Gf.Vec3f(scale_factor, scale_factor, scale_factor))
    
    # Apply rotation correction if needed
    if ref_up != stage_up:
        if ref_up == UsdGeom.Tokens.y and stage_up == UsdGeom.Tokens.z:
            print("Applying Y-up to Z-up rotation")
            xform.AddRotateXOp().Set(-90.0)
        elif ref_up == UsdGeom.Tokens.z and stage_up == UsdGeom.Tokens.y:
            print("Applying Z-up to Y-up rotation")
            xform.AddRotateXOp().Set(90.0)
    
    return prim

# Usage
robot = import_asset_with_validation(
    stage, 
    "./assets/robot_from_cad.usd",  # Might be in mm, Y-up
    "/World/Robot"
)
```

## Chapter Summary

Key takeaways:

- **Check stage metadata**: `metersPerUnit` and `upAxis` must be consistent
- **Isaac Sim defaults**: Meters, Z-up, right-handed
- **Metrics Assembler**: Automatic conversion, but verify it works correctly
- **Transform order matters**: Clear and rebuild xformOps when needed
- **Validate early**: Check units before debugging physics issues
- **Camera conventions**: -Z forward, different from robot conventions

Next: [Building and Scaling Environments](./07-building-scaling-environments.md)—instancing, layers, and performance optimization for large scenes.

## References

- [Isaac Sim Conventions](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/reference_conventions.html) — Units and coordinate system documentation
- [OpenUSD Transform Tutorial](https://openusd.org/release/tut_xforms.html) — Transform operations in USD
- [UsdGeom Stage Metrics](https://openusd.org/release/api/class_usd_geom_stage_cache.html) — Stage unit and axis API
