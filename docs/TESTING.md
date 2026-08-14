# Testing

The test suite runs outside Fusion 360 using pure Python. No Fusion API is
required for the normal regression suite.

## CI-Equivalent Local Check

From the repository root, run both commands before opening a pull request:

```powershell
python run_tests.py
python scripts/validate_examples.py examples/
```

GitHub Actions runs the same checks on Python 3.10, 3.11, and 3.12.

## Test Suites

`run_tests.py` executes these suites in order:

- `test_core`: data types, naming, unit conversions, serialization, and math
- `test_robot_model`: hierarchy, joint resolution, root detection, rigid
  groups, kinematic topology, and subassembly-container endpoints
- `test_frame_rebaser`: ROS frame conventions, CSV overrides, mesh/collision/
  inertia invariance, and cached offline regeneration
- `test_export_pipeline`: URDF/xacro generation, collision files, package
  layout, rigid-group anchor correction, frame-only links, and validation

Run one suite from the directory containing the checkout:

```powershell
python -m fusion2URDF.tests.test_frame_rebaser
python -m fusion2URDF.tests.test_export_pipeline
```

The synthetic fixtures make these tests deterministic. If a local
`debug/snapshot.json` exists, a few additional real-export checks may run;
tests tied to a different design skip cleanly rather than treating the latest
snapshot as a stable fixture.

## Example Validation

The committed packages under `examples/` are deliberate, reviewable examples,
not runtime scratch. The validator checks:

- XML well-formedness for URDF and xacro files
- resolution of every `package://` mesh reference
- non-degenerate bounding boxes for generated STL collision meshes

Do not commit normal Fusion output or debug caches. The root `.gitignore`
keeps `debug/`, Python caches, personal config, coverage output, and test
scratch out of the repository while leaving the curated examples tracked.

## Offline Tools

From the repository root, validate or visualize a captured Fusion snapshot:

```powershell
python tools/check.py <path-to-snapshot.json>
python tools/visualize.py <path-to-snapshot.json>
```

Reapply `config/frame_overrides.csv` without Fusion or mesh export:

```powershell
python tools/reframe.py <path-to-description-package>
```

The command normally discovers the sibling `debug/frame_model.json`. For an
older package that only has a matching extraction snapshot, bootstrap the
cache once:

```powershell
python tools/reframe.py <path-to-description-package> --snapshot <path-to-debug/snapshot.json>
```

The equivalent module form, when run from the directory containing this
checkout, is `python -m fusion2URDF.tools.reframe ...`.

## Adding Regression Tests

1. Add `test_<description>()` to the most focused suite.
2. Add it to that module's `run_all()` function.
3. Prefer a small synthetic snapshot over the mutable local `debug/` export.
4. Assert physical invariants, not only string output. Frame changes should
   prove that mesh, collision, inertia, and forward kinematics stay fixed.
5. Run the full CI-equivalent local check above.
