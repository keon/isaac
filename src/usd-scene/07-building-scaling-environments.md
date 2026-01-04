# Building and Scaling Environments

## Overview

Simple test scenes load instantly. Production warehouse environments with thousands of objects can take minutes to load, consume 40GB of memory, and run at single-digit FPS. The difference is scene architecture.

This chapter covers techniques for building environments that scale: instancing for repeated objects, layers for organization, payloads for memory management, and SimReady assets for physics-enabled content.

## Scene Architecture Principles

### Separation of Concerns

Organize scenes into logical layers:

```
warehouse_scene.usd (root)
├── sublayers:
│   ├── layout/warehouse_layout.usd      # Static geometry, shelves
│   ├── physics/physics_config.usd        # Physics scene settings
│   ├── lighting/warehouse_lighting.usd   # Lights and environment
│   ├── robots/robot_instances.usd        # Robot placements
│   └── objects/manipulable_objects.usd   # Dynamic objects
```

**Benefits**:
- Teams can work on different layers simultaneously
- Swap layouts without affecting robot configurations
- Version control individual components
- Load/unload sections as needed

### Reference vs Sublayer

Choose the right composition for each case:

| Use Case | Composition | Reason |
|----------|-------------|--------|
| Reusable asset (robot, shelf) | Reference | Encapsulated, multiple instances |
| Scene configuration | Sublayer | Overrides, scene-wide settings |
| Heavy environment | Payload | Deferred loading |
| Variations (gripper types) | Variant Set | Switchable without separate files |

## Instancing

Instancing is critical for performance. Instead of copying geometry, instances share the same data with different transforms.

### Point Instancer

For many identical objects (boxes, pallets, products):

```python
from pxr import UsdGeom, Gf, Vt

# Create a point instancer
instancer = UsdGeom.PointInstancer.Define(stage, "/World/Boxes")

# Define the prototype (the object to instance)
proto_container = instancer.CreatePrototypesRel()
box_proto = stage.DefinePrim("/World/Boxes/Prototypes/Box", "Cube")
UsdGeom.Cube(box_proto).CreateSizeAttr(0.3)  # 30cm box
proto_container.AddTarget("/World/Boxes/Prototypes/Box")

# Create instance positions
positions = [
    Gf.Vec3f(0, 0, 0.15),
    Gf.Vec3f(0.5, 0, 0.15),
    Gf.Vec3f(1.0, 0, 0.15),
    Gf.Vec3f(0, 0.5, 0.15),
    # ... hundreds more
]

instancer.CreatePositionsAttr(positions)

# Set which prototype each instance uses (index into prototypes)
instancer.CreateProtoIndicesAttr([0] * len(positions))

# Optional: per-instance orientations and scales
orientations = [Gf.Quath(1, 0, 0, 0)] * len(positions)
instancer.CreateOrientationsAttr(orientations)
```

**Performance**: 1000 instanced boxes use ~1000x less memory than 1000 individual prims.

### Scene Graph Instancing

For complex objects that need individual identity:

```python
# Define a prototype
shelf_proto = stage.DefinePrim("/World/Prototypes/Shelf", "Xform")
shelf_proto.GetReferences().AddReference("./assets/shelf.usd")

# Create instances that share the prototype
for i in range(10):
    instance = stage.DefinePrim(f"/World/Shelves/Shelf_{i}", "Xform")
    instance.GetReferences().AddInternalReference("/World/Prototypes/Shelf")
    
    # Each instance can have unique transform
    xform = UsdGeom.Xformable(instance)
    xform.AddTranslateOp().Set(Gf.Vec3d(i * 2.0, 0, 0))
```

### Instancing for Physics

Point instancers don't support physics directly. For physics-enabled instances:

```python
from pxr import UsdPhysics

# Create individual rigid body instances
for i in range(100):
    box = stage.DefinePrim(f"/World/PhysicsBoxes/Box_{i}", "Cube")
    UsdGeom.Cube(box).CreateSizeAttr(0.1)
    
    # Apply physics
    UsdPhysics.RigidBodyAPI.Apply(box)
    UsdPhysics.CollisionAPI.Apply(box)
    
    # Position
    xform = UsdGeom.Xformable(box)
    xform.AddTranslateOp().Set(Gf.Vec3d(
        (i % 10) * 0.15,
        (i // 10) * 0.15,
        0.5 + (i // 100) * 0.15
    ))
```

For large numbers, use Isaac Sim's `create_prim` utilities which are optimized.

## SimReady Assets

SimReady assets are pre-configured for physics simulation with proper collision geometry, materials, and mass properties.

### Using SimReady Assets

```python
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Get path to SimReady assets
assets_root = get_assets_root_path()
pallet_path = f"{assets_root}/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd"

# Reference into scene
pallet = stage.DefinePrim("/World/Pallet", "Xform")
pallet.GetReferences().AddReference(pallet_path)
```

### SimReady Categories

Available SimReady assets include:
- **Warehouse**: Pallets, boxes, conveyors, racks
- **Robots**: Arms, mobile bases, grippers
- **Props**: Tables, chairs, containers
- **Environments**: Room layouts, outdoor areas

### Custom SimReady Assets

To make your own assets SimReady:

1. **Add collision geometry**: Simplified meshes for physics
2. **Configure rigid body**: Mass, center of mass, inertia
3. **Apply materials**: Physics materials with friction/restitution
4. **Set metadata**: Proper `metersPerUnit` and `upAxis`

```python
def make_simready(prim):
    """Add physics properties to make a prim simulation-ready."""
    
    # Add rigid body
    UsdPhysics.RigidBodyAPI.Apply(prim)
    
    # Add collision (use existing geometry or add simplified)
    UsdPhysics.CollisionAPI.Apply(prim)
    
    # Set mass properties
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(1.0)  # 1 kg
    
    # Create physics material
    material_path = f"{prim.GetPath()}/PhysicsMaterial"
    material = UsdPhysics.MaterialAPI.Apply(
        stage.DefinePrim(material_path, "Material")
    )
    material.CreateStaticFrictionAttr(0.5)
    material.CreateDynamicFrictionAttr(0.4)
    material.CreateRestitutionAttr(0.1)
    
    # Bind material to collision
    collision_api = UsdPhysics.CollisionAPI(prim)
    # Material binding happens via relationship
```

## Layer Organization

### Environment Layers

Structure environment data across layers:

```python
from pxr import Sdf

# Create layer structure
def create_environment_layers(base_path):
    layers = {}
    
    # Static geometry layer
    layers['geometry'] = Sdf.Layer.CreateNew(f"{base_path}/geometry.usd")
    
    # Collision layer (can be simplified geometry)
    layers['collision'] = Sdf.Layer.CreateNew(f"{base_path}/collision.usd")
    
    # Lighting layer
    layers['lighting'] = Sdf.Layer.CreateNew(f"{base_path}/lighting.usd")
    
    # Semantic/annotation layer
    layers['semantics'] = Sdf.Layer.CreateNew(f"{base_path}/semantics.usd")
    
    return layers
```

### Override Layers

Use override layers for per-project customization:

```python
# Base warehouse (shared across projects)
base_warehouse = "nucleus://shared/warehouses/standard_warehouse.usd"

# Project-specific overrides
project_layer = Sdf.Layer.CreateNew("./project_warehouse.usd")
project_stage = Usd.Stage.Open(project_layer)

# Reference the base
project_stage.GetRootLayer().subLayerPaths.append(base_warehouse)

# Override specific properties
with Usd.EditContext(project_stage, project_layer):
    # Change shelf positions for this project
    shelf = project_stage.GetPrimAtPath("/World/Shelves/Shelf_01")
    xform = UsdGeom.Xformable(shelf)
    xform.AddTranslateOp().Set(Gf.Vec3d(5.0, 0, 0))
```

## Payload Management

Payloads allow loading/unloading scene sections on demand.

### Creating Payloaded Sections

```python
# Main scene with payloaded sections
stage = Usd.Stage.CreateNew("warehouse.usd")

# Create areas as payloads
areas = [
    ("Zone_A", "./zones/zone_a.usd"),
    ("Zone_B", "./zones/zone_b.usd"),
    ("Zone_C", "./zones/zone_c.usd"),
]

for area_name, area_path in areas:
    area_prim = stage.DefinePrim(f"/World/{area_name}", "Xform")
    area_prim.GetPayloads().AddPayload(area_path)

stage.Save()
```

### Loading Strategies

```python
# Load nothing initially (fast startup)
stage = Usd.Stage.Open("warehouse.usd", Usd.Stage.LoadNone)

# Load specific areas
stage.Load("/World/Zone_A")

# Load everything
stage.LoadAll()

# Unload to free memory
stage.Unload("/World/Zone_C")

# Check what's loaded
for prim in stage.Traverse():
    if prim.HasPayload():
        is_loaded = prim.IsLoaded()
        print(f"{prim.GetPath()}: {'loaded' if is_loaded else 'unloaded'}")
```

### Payload Load Rules

Set default loading behavior:

```python
# Create load rules
load_rules = Usd.StageLoadRules()

# Load everything under /World/ActiveArea
load_rules.AddRule("/World/ActiveArea", Usd.StageLoadRules.AllRule)

# Don't load /World/InactiveAreas
load_rules.AddRule("/World/InactiveAreas", Usd.StageLoadRules.NoneRule)

# Apply to stage
stage = Usd.Stage.Open("warehouse.usd", load_rules)
```

## Performance Optimization

### Collision Geometry Simplification

Visual geometry is often too detailed for physics:

```python
def create_collision_approximation(visual_prim, collision_prim_path):
    """Create simplified collision geometry for a visual mesh."""
    
    # Get bounding box of visual
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    bbox = bbox_cache.ComputeLocalBound(visual_prim)
    box_range = bbox.ComputeAlignedRange()
    
    size = box_range.GetSize()
    center = box_range.GetMidpoint()
    
    # Create box collision
    collision = UsdGeom.Cube.Define(stage, collision_prim_path)
    collision.CreateSizeAttr(1.0)  # Unit cube, we'll scale it
    
    xform = UsdGeom.Xformable(collision.GetPrim())
    xform.AddTranslateOp().Set(center)
    xform.AddScaleOp().Set(size)
    
    # Make it collision-only (invisible)
    collision.CreatePurposeAttr(UsdGeom.Tokens.guide)
    
    # Apply collision API
    UsdPhysics.CollisionAPI.Apply(collision.GetPrim())
    
    return collision
```

### Level of Detail (LOD)

Use variant sets for LOD:

```python
# Create LOD variants
shelf = stage.DefinePrim("/World/Shelf", "Xform")
lod_vset = shelf.GetVariantSets().AddVariantSet("LOD")

lod_vset.AddVariant("high")
lod_vset.AddVariant("medium")
lod_vset.AddVariant("low")

# Author each LOD level
for lod in ["high", "medium", "low"]:
    lod_vset.SetVariantSelection(lod)
    with lod_vset.GetVariantEditContext():
        mesh = stage.DefinePrim(f"/World/Shelf/Mesh_{lod}", "Mesh")
        mesh.GetReferences().AddReference(f"./shelf_{lod}.usd")

# Select based on distance or performance needs
lod_vset.SetVariantSelection("medium")
```

### Render Settings for Performance

```python
# Reduce render quality for faster iteration
import omni.kit.viewport.utility as viewport_utils

viewport = viewport_utils.get_active_viewport()

# Lower resolution
viewport.resolution = (1280, 720)

# Disable expensive effects
# Access via render settings in the UI or carb settings
```

## Procedural Environment Generation

### Grid-Based Placement

```python
def generate_warehouse_grid(stage, rows, cols, spacing):
    """Generate a grid of shelves."""
    
    shelf_template = "./assets/shelf_unit.usd"
    
    for row in range(rows):
        for col in range(cols):
            shelf_path = f"/World/Shelves/Shelf_R{row}_C{col}"
            shelf = stage.DefinePrim(shelf_path, "Xform")
            shelf.GetReferences().AddReference(shelf_template)
            
            # Position
            x = col * spacing
            y = row * spacing
            xform = UsdGeom.Xformable(shelf)
            xform.AddTranslateOp().Set(Gf.Vec3d(x, y, 0))

# Generate 10x10 warehouse section
generate_warehouse_grid(stage, rows=10, cols=10, spacing=3.0)
```

### Randomized Object Placement

```python
import random

def scatter_objects(stage, parent_path, object_path, count, bounds):
    """Scatter objects randomly within bounds."""
    
    min_x, max_x = bounds[0]
    min_y, max_y = bounds[1]
    min_z, max_z = bounds[2]
    
    for i in range(count):
        obj_path = f"{parent_path}/Object_{i}"
        obj = stage.DefinePrim(obj_path, "Xform")
        obj.GetReferences().AddReference(object_path)
        
        # Random position
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        z = random.uniform(min_z, max_z)
        
        # Random rotation around Z
        rot = random.uniform(0, 360)
        
        xform = UsdGeom.Xformable(obj)
        xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        xform.AddRotateZOp().Set(rot)

# Scatter boxes on a shelf
scatter_objects(
    stage, 
    "/World/ShelfContents",
    "./assets/cardboard_box.usd",
    count=50,
    bounds=[(0, 10), (0, 2), (0.5, 2.5)]
)
```

## Running Example

Building our warehouse environment:

```python
def create_warehouse_environment(stage):
    """Create a complete warehouse environment for mobile manipulation."""
    
    # Stage setup
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    world = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(world)
    
    # Physics scene
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    
    # Ground plane
    ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
    ground.CreatePointsAttr([(-50, -50, 0), (50, -50, 0), (50, 50, 0), (-50, 50, 0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    
    # Shelving units (instanced pattern)
    shelf_template = get_assets_root_path() + "/Isaac/Environments/Simple_Warehouse/Props/SM_RackLarge_01.usd"
    
    for row in range(3):
        for col in range(5):
            shelf_path = f"/World/Shelves/Shelf_R{row}_C{col}"
            shelf = stage.DefinePrim(shelf_path, "Xform")
            shelf.GetReferences().AddReference(shelf_template)
            
            xform = UsdGeom.Xformable(shelf)
            xform.AddTranslateOp().Set(Gf.Vec3d(col * 4.0, row * 6.0, 0))
    
    # Work area (payloaded for optional loading)
    work_area = stage.DefinePrim("/World/WorkArea", "Xform")
    work_area.GetPayloads().AddPayload("./areas/work_station.usd")
    
    # Robot spawn point (marked for easy reference)
    spawn_point = stage.DefinePrim("/World/RobotSpawnPoint", "Xform")
    UsdGeom.Xformable(spawn_point).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    
    return stage
```

## Chapter Summary

Key takeaways:

- **Layer organization**: Separate geometry, physics, lighting, and overrides
- **Instancing is critical**: Point instancers for identical objects, references for complex instances
- **SimReady assets**: Pre-configured physics saves setup time
- **Payloads for memory**: Load/unload scene sections as needed
- **Collision simplification**: Don't use visual geometry for physics
- **Procedural generation**: Scripts for repetitive placement

Next: [Debugging Guide](./usd-failure-modes.md)—common problems and how to diagnose them.

## References

- [USD Point Instancers](https://openusd.org/release/api/class_usd_geom_point_instancer.html) — Instancing large numbers of objects
- [Isaac Sim Assets](https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/assets.html) — SimReady assets documentation
- [USD Best Practices](https://openusd.org/release/glossary.html) — Scene organization guidelines
- [Omniverse Create](https://docs.omniverse.nvidia.com/create/latest/) — Environment building tools
