# Debugging Guide

When something goes wrong with USD scenes, symptoms often appear far from the root cause. This guide catalogs common failure patterns and their solutions.

## Decision Tree

```
START: USD scene problem
│
├─► Scene won't load at all
│   ├─ File not found → Check paths (Section: Path Issues)
│   ├─ Parse error → Check USD syntax (Section: Syntax Errors)
│   └─ Hangs forever → Large payloads or network (Section: Loading Issues)
│
├─► Scene loads but looks wrong
│   ├─ Objects missing → Check composition (Section: Composition Issues)
│   ├─ Wrong scale/position → Check units (see Transforms chapter)
│   └─ Materials wrong → Check material binding (Section: Material Issues)
│
├─► Edits don't take effect
│   ├─ Editing wrong layer → Check edit target (Section: Layer Issues)
│   ├─ Stronger opinion exists → Check LIVRPS (Section: Opinion Strength)
│   └─ Cache not invalidated → Force refresh (Section: Caching Issues)
│
└─► Performance problems
    ├─ Slow load → Too many prims/large assets (Section: Performance)
    └─ Slow render/physics → Scene complexity (see Building Environments chapter)
```

## Path Issues

### "Asset not found" / "Failed to resolve"

**Symptoms**: Error messages about missing files, prims show as placeholders.

**Diagnostic**:
```python
from pxr import Ar

# Check how a path resolves
resolver = Ar.GetResolver()
resolved = resolver.Resolve("./assets/robot.usd")
print(f"Resolved path: {resolved}")

# List unresolved references
def find_unresolved_refs(stage):
    unresolved = []
    for prim in stage.Traverse():
        refs = prim.GetMetadata("references")
        if refs:
            for ref in refs.GetAddedOrExplicitItems():
                resolved = resolver.Resolve(ref.assetPath)
                if not resolved:
                    unresolved.append((prim.GetPath(), ref.assetPath))
    return unresolved

for prim_path, asset_path in find_unresolved_refs(stage):
    print(f"Unresolved: {prim_path} -> {asset_path}")
```

**Common Causes**:
- Relative paths broken by moving files
- Nucleus paths when offline
- Case sensitivity (Linux vs Windows)
- Missing file extensions

**Fixes**:
```python
# Use asset resolver search paths
resolver = Ar.GetResolver()
resolver.ConfigureResolverForAsset("./")  # Add current directory

# Or use absolute paths for critical assets
stage.GetRootLayer().UpdateExternalReference(
    "./robot.usd",
    "/absolute/path/to/robot.usd"
)
```

### Nucleus Path Issues

**Symptoms**: Assets work locally but fail when using `omniverse://` paths.

**Diagnostic**:
```python
import omni.client

# Check Nucleus connection
result, _ = omni.client.stat("omniverse://localhost/")
print(f"Nucleus status: {result}")

# List available servers
result, entries = omni.client.list("omniverse://")
for entry in entries:
    print(f"Server: {entry.relative_path}")
```

**Fixes**:
- Verify Nucleus is running: Check Omniverse Launcher
- Check authentication: Re-login to Nucleus
- Use local paths for offline development

## Composition Issues

### "My changes don't appear"

**Symptoms**: You edit a property but the scene doesn't reflect the change.

**Diagnostic**:
```python
def diagnose_property(stage, prim_path, prop_name):
    """Show all opinions on a property and which wins."""
    prim = stage.GetPrimAtPath(prim_path)
    prop = prim.GetAttribute(prop_name)
    
    if not prop:
        print(f"Property {prop_name} not found")
        return
    
    # Get the resolved value
    print(f"Resolved value: {prop.Get()}")
    
    # Get property stack (all opinions)
    stack = prop.GetPropertyStack()
    print(f"\nOpinion stack (strongest first):")
    for spec in stack:
        layer = spec.layer
        print(f"  {layer.identifier}: {spec.GetInfo('default')}")
```

**Common Causes**:
- Editing a weaker layer (LIVRPS ordering)
- Property defined in a referenced file
- Variant selection hiding your edits
- Edit target set to wrong layer

**Fix - Check and set edit target**:
```python
# See current edit target
edit_target = stage.GetEditTarget()
print(f"Current edit target: {edit_target.GetLayer().identifier}")

# Set edit target to specific layer
my_layer = Sdf.Layer.FindOrOpen("my_overrides.usd")
stage.SetEditTarget(Usd.EditTarget(my_layer))
```

### "Object appears multiple times"

**Symptoms**: Duplicate geometry, object shows up in wrong place.

**Diagnostic**:
```python
def find_prim_sources(stage, target_path):
    """Find all composition arcs contributing to a prim."""
    prim = stage.GetPrimAtPath(target_path)
    
    # Get prim index
    prim_index = prim.GetPrimIndex()
    
    print(f"Composition arcs for {target_path}:")
    for node in prim_index.rootNode.children:
        print(f"  {node.arcType}: {node.site}")
```

**Common Causes**:
- Same asset referenced multiple times
- Inherits and references both pulling same content
- Sublayer ordering creating duplicates

### "I can't override this property"

**Symptoms**: Property changes revert or don't apply.

**Diagnostic**:
```python
# Check if property is from a reference
prim = stage.GetPrimAtPath("/World/Robot")
prop = prim.GetAttribute("xformOp:translate")

# Check if authored locally
if prop.IsAuthored():
    print("Property is authored in current layer")
else:
    print("Property value comes from composed sources")

# Check for locked metadata
if prim.GetMetadata("documentation"):
    print("Prim has documentation/may be locked")
```

**Fix - Create local override**:
```python
# Explicitly author in current layer to override
with Usd.EditContext(stage, stage.GetRootLayer()):
    prop = prim.GetAttribute("xformOp:translate")
    prop.Set(Gf.Vec3d(1, 2, 3))  # This creates a local opinion
```

## Layer Issues

### Wrong Edit Target

**Symptoms**: Edits saved but don't appear after reload.

**Diagnostic**:
```python
# Check where edits are going
print(f"Root layer: {stage.GetRootLayer().identifier}")
print(f"Edit target: {stage.GetEditTarget().GetLayer().identifier}")

# List all sublayers
def print_layer_stack(layer, indent=0):
    print(" " * indent + layer.identifier)
    for sublayer_path in layer.subLayerPaths:
        sublayer = Sdf.Layer.Find(sublayer_path)
        if sublayer:
            print_layer_stack(sublayer, indent + 2)

print_layer_stack(stage.GetRootLayer())
```

**Fix**:
```python
# Always explicitly set edit target before authoring
override_layer = stage.GetRootLayer()  # Or specific sublayer
stage.SetEditTarget(Usd.EditTarget(override_layer))

# Then make edits
prim = stage.GetPrimAtPath("/World/Robot")
# ... edits go to override_layer
```

### Sublayer Order Problems

**Symptoms**: Stronger opinions coming from unexpected layer.

**Diagnostic**:
```python
# Sublayers listed later are STRONGER
root = stage.GetRootLayer()
print("Sublayer order (weakest to strongest):")
for i, path in enumerate(root.subLayerPaths):
    print(f"  {i}: {path}")
```

**Fix**:
```python
# Reorder sublayers (last = strongest)
root = stage.GetRootLayer()
paths = list(root.subLayerPaths)
# Move overrides.usd to end (strongest)
paths.remove("overrides.usd")
paths.append("overrides.usd")
root.subLayerPaths = paths
```

## Syntax and Structure Errors

### Invalid USD File

**Symptoms**: Stage fails to open, parse errors in console.

**Diagnostic**:
```python
# Try to identify the error
from pxr import Sdf

layer = Sdf.Layer.FindOrOpen("problematic.usd")
if not layer:
    print("Layer failed to open - check for syntax errors")
    # Look at console output for specific error
```

**Common Causes**:
- Hand-edited USD with syntax errors
- Incomplete save/file corruption
- Version incompatibility

**Fix**: Use `usdchecker` tool:
```bash
# Validate USD file
usdchecker problematic.usd

# View in text format
usdcat problematic.usd
```

### Invalid Prim Paths

**Symptoms**: Operations fail with path errors.

**Rules for valid prim paths**:
- Must start with `/`
- Components separated by `/`
- No spaces or special characters (except `_`)
- Cannot start with numbers

```python
# Valid paths
"/World/Robot"
"/World/Robot_01"
"/World/Shelf_A/Box"

# Invalid paths
"World/Robot"      # Missing leading /
"/World/Robot 01"  # Space
"/World/01_Robot"  # Component starts with number (context-dependent)
```

## Material and Visual Issues

### Materials Not Appearing

**Symptoms**: Objects render as default gray.

**Diagnostic**:
```python
from pxr import UsdShade

def check_material_binding(prim):
    binding = UsdShade.MaterialBindingAPI(prim)
    material, relationship = binding.ComputeBoundMaterial()
    
    if material:
        print(f"Bound material: {material.GetPath()}")
    else:
        print("No material bound")
        # Check if binding relationship exists but is broken
        rel = prim.GetRelationship("material:binding")
        if rel:
            targets = rel.GetTargets()
            print(f"Binding targets to: {targets}")

check_material_binding(stage.GetPrimAtPath("/World/Robot/Mesh"))
```

**Common Causes**:
- Material path not resolved
- Material schema not applied
- Render delegate doesn't support material type

### Wrong Material Properties

**Symptoms**: Material appears but looks wrong (color, roughness, etc.).

**Diagnostic**:
```python
def inspect_material(material_path):
    material = UsdShade.Material.Get(stage, material_path)
    
    # Get shader outputs
    for output in material.GetOutputs():
        print(f"Output: {output.GetFullName()}")
        
    # Get connected shader
    shader, output_name = material.ComputeSurfaceSource()
    if shader:
        print(f"Surface shader: {shader.GetPath()}")
        for inp in shader.GetInputs():
            print(f"  {inp.GetBaseName()}: {inp.Get()}")
```

## Caching Issues

### Changes Not Reflected

**Symptoms**: File changed but stage shows old data.

**Diagnostic**:
```python
# Check if layer has unsaved changes
layer = stage.GetRootLayer()
print(f"Layer dirty: {layer.dirty}")

# Force reload from disk
layer.Reload()
```

**Fixes**:
```python
# Clear USD caches
from pxr import Ar
Ar.GetResolver().RefreshContext(Ar.ResolverContext())

# Or reload stage entirely
stage_path = stage.GetRootLayer().identifier
omni.usd.get_context().close_stage()
omni.usd.get_context().open_stage(stage_path)
```

### Nucleus Cache Issues

**Symptoms**: Changes pushed to Nucleus not visible to others.

**Fixes**:
- Checkpoint the file in Nucleus
- Other users: refresh their Nucleus connection
- Clear local Nucleus cache (in Omniverse Launcher settings)

## Performance Issues

### Slow Scene Loading

**Diagnostic**:
```python
import time

# Profile loading
start = time.time()
stage = Usd.Stage.Open("large_scene.usd", Usd.Stage.LoadNone)
print(f"Stage open (no payloads): {time.time() - start:.2f}s")

start = time.time()
stage.LoadAll()
print(f"Load all payloads: {time.time() - start:.2f}s")

# Count prims
prim_count = len(list(stage.Traverse()))
print(f"Total prims: {prim_count}")
```

**Common Causes**:
- Too many prims (>100k starts to slow)
- Large textures loading
- Network latency (Nucleus)
- No instancing for repeated objects

**Fixes**:
- Use payloads for heavy sections
- Instance repeated objects
- Use lower-resolution assets for development
- Work with local files, sync to Nucleus for sharing

### Memory Exhaustion

**Symptoms**: Isaac Sim crashes or freezes with large scenes.

**Diagnostic**:
```bash
# Monitor GPU memory
nvidia-smi -l 1

# In Python
import pynvml
pynvml.nvmlInit()
handle = pynvml.nvmlDeviceGetHandleByIndex(0)
info = pynvml.nvmlDeviceGetMemoryInfo(handle)
print(f"GPU memory used: {info.used / 1e9:.2f} GB")
```

**Fixes**:
```python
# Unload unused payloads
stage.Unload("/World/InactiveArea")

# Reduce parallel environment count
# Lower texture resolution
# Simplify collision geometry
```

## Running Example Diagnostics

For our warehouse scene:

```python
def diagnose_warehouse_scene(stage):
    """Run comprehensive diagnostics on warehouse scene."""
    
    issues = []
    
    # 1. Check stage settings
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    if mpu != 1.0:
        issues.append(f"Units: metersPerUnit={mpu}, expected 1.0")
    if up != UsdGeom.Tokens.z:
        issues.append(f"Up axis: {up}, expected Z")
    
    # 2. Check for unresolved references
    resolver = Ar.GetResolver()
    for prim in stage.Traverse():
        refs = prim.GetMetadata("references")
        if refs:
            for ref in refs.GetAddedOrExplicitItems():
                if not resolver.Resolve(ref.assetPath):
                    issues.append(f"Unresolved: {prim.GetPath()} -> {ref.assetPath}")
    
    # 3. Check physics configuration
    physics_scene = UsdPhysics.Scene.Get(stage, "/World/PhysicsScene")
    if not physics_scene:
        issues.append("No PhysicsScene found at /World/PhysicsScene")
    
    # 4. Check robot articulation
    robot = stage.GetPrimAtPath("/World/Robot")
    if robot:
        if not robot.HasAPI(UsdPhysics.ArticulationRootAPI):
            issues.append("Robot missing ArticulationRootAPI")
    else:
        issues.append("Robot not found at /World/Robot")
    
    # 5. Count prims for performance awareness
    prim_count = len(list(stage.Traverse()))
    if prim_count > 50000:
        issues.append(f"High prim count: {prim_count} (may affect performance)")
    
    # Report
    if issues:
        print("⚠ Issues found:")
        for issue in issues:
            print(f"  - {issue}")
    else:
        print("✓ No issues detected")
    
    return issues

# Run diagnostics
diagnose_warehouse_scene(stage)
```

## Chapter Summary

Key diagnostic strategies:

- **Path issues**: Check resolver, use absolute paths for critical assets
- **Composition issues**: Understand LIVRPS, check edit target
- **Layer issues**: Know your sublayer order, explicit edit targets
- **Caching issues**: Reload layers, clear resolver cache
- **Performance issues**: Count prims, profile loading, use payloads

With this section complete, you understand USD scene structure and common pitfalls. The next section covers robot modeling—importing, articulations, and physics configuration.

## References

- [USD Debugging Guide](https://openusd.org/release/tut_debugging.html) — Official debugging techniques
- [usdchecker Tool](https://openusd.org/release/toolset.html#usdchecker) — USD validation utility
- [Asset Resolver](https://openusd.org/release/api/ar_page_front.html) — Path resolution documentation
