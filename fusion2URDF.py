"""
fusion2URDF — Fusion 360 → ROS 2 Exporter
=========================================
Fusion 360 add-in that extracts the active design and generates a
complete ROS 2 robot_description package: hierarchical xacro, visual
+ collision meshes, ros2_control, launch files, and a JSON snapshot
for downstream tooling.

Pipeline:
  Phase 1: Extract → FusionSnapshot + debug report
  Phase 2: Build kinematic model → RobotModel
  Phase 3: Export meshes (DAE or OBJ+MTL + collision STL) + generate
           ROS 2 package

Output:
  <robot>_description/
    urdf/<robot>.urdf.xacro            Top-level xacro (swap-ready)
    urdf/assemblies/<asm>.urdf.xacro   Per-assembly macro
    urdf/<robot>.urdf                  Flat URDF (for validation)
    meshes/<asm>/<link>.dae|.obj+.mtl  Visual meshes
    meshes/<asm>/<link>_collision.stl  Collision (when generated)
    launch/display.launch.py           RViz visualization
    package.xml, CMakeLists.txt        ROS 2 package files

  debug/
    extraction_report.md               Compare with Fusion Properties panel
    snapshot.json                      Raw extraction data
    export_log.md                      Full log with timestamps

Author: Adrian Valaker Eikeland
Licensed under the MIT License — see LICENSE.
"""

import adsk.core
import adsk.fusion
import traceback
import os
from datetime import datetime

from .utils.brainrot import start_brainrot, stop_brainrot

def run(context):
    ui = None
    title = 'fusion2URDF'
    brainrot = None

    try:
        app = adsk.core.Application.get()
        ui = app.userInterface


        # Import pipeline modules
        try:
            from .utils import Logger, snapshot_to_file, explicit_collision_names
            from .core.fusion_extractor import extract, export_meshes
            from .core.snapshot_report import generate_report
            from .core.robot_model import build_model
            from .core.package_generator import generate_package
            from .core.data_types import ExportConfig
        except ImportError as e:
            ui.messageBox(f"Import error:\n\n{e}\n\n{traceback.format_exc()}", title)
            return

        log = Logger(timestamps=True)
        
        from .core.data_types import EXPORTER_VERSION
        log(f"fusion2URDF v{EXPORTER_VERSION}")
        log(f"Time: {datetime.now().isoformat()}")

        # Get active design
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        if not design:
            ui.messageBox('No active Fusion design.\n\nPlease open an assembly.', title)
            return

        root = design.rootComponent
        log(f"Design: {root.name}")
        log(f"Components: {design.allComponents.count}")

        # ════════════════════════════════════════
        #  PHASE 1: EXTRACT
        # ════════════════════════════════════════
        log.section("PHASE 1: EXTRACTION")
        snapshot = extract(design, log)

        summary = (
            f"Extraction complete!\n\n"
            f"  Occurrences: {snapshot.total_occurrences}\n"
            f"  Subassemblies: {snapshot.total_subassemblies}\n"
            f"  Leaf components: {snapshot.total_leaf_components}\n"
            f"  Joints: {snapshot.total_joints}\n"
            f"  Max depth: {snapshot.max_nesting_depth}\n\n"
            f"Generate URDF/XACRO package?"
        )
        result = ui.messageBox(summary, title,
                               adsk.core.MessageBoxButtonTypes.YesNoButtonType)
        if result == adsk.core.DialogResults.DialogNo:
            return

        # Browse for output folder
        folder_dlg = ui.createFolderDialog()
        folder_dlg.title = 'Select output folder for robot_description package'
        if folder_dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            return

        save_dir = folder_dlg.folder
        
        # Load user config from the plugin-local xacro_export.toml
        # (repo-root fallback for older setups).  Anything in the file
        # overrides the dialog defaults below.  Returns an empty dict
        # when no file is present, so running without a config is a no-op.
        from .core.user_config import find_config_path, load_toml, apply_to_config
        from .core.data_types import ExportConfig as _ExportConfig
        cfg_path = find_config_path()
        cfg_data = load_toml(cfg_path) if cfg_path else {}
        if cfg_path:
            log(f"  Loaded config: {cfg_path}")

        # Build a draft ExportConfig with TOML-applied defaults; the
        # rest of the dialog flow may further override fields below.
        draft_cfg = _ExportConfig()
        cfg_changes = apply_to_config(draft_cfg, cfg_data) if cfg_data else []
        for ch in cfg_changes:
            log(f"    config: {ch}")

        debug_dir = os.path.abspath(os.path.join(save_dir, "debug"))
        debug_write_errors = []
        # Always write Phase 1 debug data, even when the user opted out
        # via ``include_debug = false`` in xacro_export.toml.  Cost is
        # tiny (a few hundred KB), and when validation aborts later
        # this is the only artifact left to diagnose from.  We delete
        # the folder at the end on a clean run if include_debug=false
        # (see the post-export cleanup below the EXPORT COMPLETE log).
        os.makedirs(debug_dir, exist_ok=True)

        log.section("PHASE 1: DEBUG DATA")
        try:
            report_md = generate_report(snapshot)
            with open(os.path.join(debug_dir, "extraction_report.md"), 'w', encoding='utf-8') as f:
                f.write(report_md)
            log(f"  extraction_report.md")
        except Exception as e:
            log.error(f"Report generation failed: {e}")
            debug_write_errors.append(f"extraction_report.md: {e}")

        try:
            snapshot_to_file(snapshot, os.path.join(debug_dir, "snapshot.json"))
            log(f"  snapshot.json")
        except Exception as e:
            log.error(f"JSON export failed: {e}")
            debug_write_errors.append(f"snapshot.json: {e}")

        try:
            from .core.transform_dump import dump_transforms
            dump_transforms(design, os.path.join(debug_dir, "fusion_transforms.json"))
            log(f"  fusion_transforms.json")
        except Exception as e:
            log.error(f"Transform dump failed: {e}")
            debug_write_errors.append(f"fusion_transforms.json: {e}")

        # ════════════════════════════════════════
        #  PHASE 2: BUILD MODEL
        # ════════════════════════════════════════
        log.section("PHASE 2: BUILD ROBOT MODEL")
        model = build_model(snapshot, log)

        if model.errors:
            # Hard abort.  Validation errors (orphan links, missing root,
            # self-referencing joints, dangling joint endpoints) all
            # produce broken URDFs that no consumer can recover from —
            # there is no "Continue anyway" answer that makes sense.
            # Save the export log to debug/ so the user has a complete
            # diagnostic trail (snapshot.json, fusion_transforms.json,
            # extraction_report.md, export_log.md).
            try:
                from .core.package_generator import generate_validation_report
                abort_cfg = _ExportConfig()
                abort_cfg.package_name = f"{model.name}_description"
                abort_cfg.output_dir = save_dir
                generate_validation_report(model, abort_cfg, debug_dir)
                log(f"  validation.md")
            except Exception as e:
                log.error(f"Validation report export failed: {e}")
                debug_write_errors.append(f"validation.md: {e}")

            try:
                log.save_markdown(
                    os.path.join(debug_dir, "export_log.md"),
                    title=f"Export Log (aborted): {root.name}",
                )
            except Exception as e:
                debug_write_errors.append(f"export_log.md: {e}")

            debug_files = []
            try:
                debug_files = sorted(
                    name for name in os.listdir(debug_dir)
                    if os.path.isfile(os.path.join(debug_dir, name))
                )
            except Exception as e:
                debug_write_errors.append(f"debug folder listing: {e}")
            error_msg = (
                "Export aborted — model has errors:\n\n"
                + "\n".join(f"  • {e}" for e in model.errors)
                + f"\n\nDebug data written to:\n  {debug_dir}\n"
            )
            if debug_files:
                error_msg += "  Files: " + ", ".join(debug_files) + "\n"
            else:
                error_msg += "  No debug files were found in that folder.\n"

            tree_text = getattr(model, "_kinematic_tree_text", "")
            if tree_text:
                tree_lines = tree_text.splitlines()
                max_tree_lines = 35
                tree_preview = "\n".join(tree_lines[:max_tree_lines])
                if len(tree_lines) > max_tree_lines:
                    tree_preview += (
                        f"\n  ... {len(tree_lines) - max_tree_lines} more lines "
                        f"(see debug/validation.md)"
                    )
                error_msg += "\nKinematic tree preview:\n\n" + tree_preview + "\n"
            if debug_write_errors:
                error_msg += (
                    "\nSome debug files could not be written:\n"
                    + "\n".join(f"  ! {e}" for e in debug_write_errors)
                    + "\n"
                )
            error_msg += (
                "\nFix the issues in Fusion and re-run the export.\n"
                "Send the debug folder for help diagnosing."
            )
            ui.messageBox(error_msg, title)
            return

        # ════════════════════════════════════════
        #  PHASE 3: MESHES + PACKAGE
        # ════════════════════════════════════════
        
        override_explicit = False
        explicit_names = explicit_collision_names(snapshot)

        # Collision method: prefer TOML when set.  The dialog is yes/no, so
        # config is also the way to select advanced methods like convex_hull.
        if cfg_data.get("mesh", {}).get("collision_method"):
            collision_method = draft_cfg.collision_auto_method
            log(f"  collision_method from config: {collision_method}")
        else:
            collision_msg = (
                "Collision geometry for links without explicit collision:\n\n"
                "YES = Generate simplified collision primitives\n"
                "  (auto-fit box/cylinder/sphere per link)\n\n"
                "NO = Use visual mesh as collision\n"
                "  (exact shape, heavier for physics)\n\n"
                "Choose collision method:"
            )
            collision_result = ui.messageBox(
                collision_msg, title,
                adsk.core.MessageBoxButtonTypes.YesNoButtonType
            )

            collision_method = "primitive"
            if collision_result == adsk.core.DialogResults.DialogNo:
                collision_method = "visual_reuse"

                if explicit_names:
                    override_msg = (
                        "Your design contains explicit collision geometry\n"
                        "(collision_ components or rigid groups).\n\n"
                        "YES = Use visual mesh for ALL links\n"
                        "  (ignore explicit collision components)\n\n"
                        "NO = Keep explicit collision where it exists,\n"
                        "  visual mesh only for the rest\n\n"
                        "Override explicit collision?"
                    )
                    override_result = ui.messageBox(
                        override_msg, title,
                        adsk.core.MessageBoxButtonTypes.YesNoButtonType
                    )
                    if override_result != adsk.core.DialogResults.DialogNo:
                        override_explicit = True
            elif explicit_names:
                # Primary = primitive.  Confirm the mixed-mode default
                # (explicit STL where the user made it, primitive elsewhere)
                # so it's visible rather than silent.
                names_list = ", ".join(explicit_names)
                precise_msg = (
                    f"Explicit collision geometry detected for:\n"
                    f"  {names_list}\n\n"
                    f"YES = Use precise STL collision for those components,\n"
                    f"  primitive for the rest (mixed)\n\n"
                    f"NO = Override with primitives for ALL links\n"
                    f"  (ignore your manual collision)\n\n"
                    f"Use precise STL where you made it?"
                )
                precise_result = ui.messageBox(
                    precise_msg, title,
                    adsk.core.MessageBoxButtonTypes.YesNoButtonType
                )
                if precise_result == adsk.core.DialogResults.DialogNo:
                    override_explicit = True
        
        # Visual format: prefer the TOML-configured value if set,
        # otherwise ask via dialog.  DAE is friendlier downstream (Gazebo,
        # RViz, Isaac Sim all read it natively) and skips the cm→m
        # conversion in URDF.
        if cfg_data.get("mesh", {}).get("visual_format"):
            visual_format = draft_cfg.visual_format
            log(f"  visual_format from config: {visual_format}")
        else:
            format_msg = (
                "Visual mesh format:\n\n"
                "YES = DAE (Collada)\n"
                "  Single self-contained .dae file in meters, scale=\"1\" in URDF.\n"
                "  Friendliest for Gazebo / RViz / Isaac Sim.\n\n"
                "NO = OBJ + MTL\n"
                "  Fusion native pair, centimeters, scale=\"0.01\" in URDF.\n"
                "  Choose this if a downstream tool needs OBJ specifically.\n\n"
                "Use DAE?"
            )
            format_result = ui.messageBox(
                format_msg, title,
                adsk.core.MessageBoxButtonTypes.YesNoButtonType
            )
            visual_format = "dae" if format_result != adsk.core.DialogResults.DialogNo else "obj"
        brainrot_enabled = cfg_data.get("brainrot", True) is not False
        if brainrot_enabled :
            brainrot = start_brainrot()

        # Build the final ExportConfig — start from the TOML-applied
        # draft, then layer in the dialog answers and the package name.
        config = draft_cfg
        config.package_name = f"{model.name}_description"
        config.output_dir = save_dir
        config.collision_auto_method = collision_method
        config.override_explicit_collision = override_explicit
        config.visual_format = visual_format
        if not config.mesh_refinement:
            config.mesh_refinement = "medium"
        pkg_dir = os.path.join(save_dir, config.package_name)

        # 3a: Export meshes from Fusion (OBJ+MTL or DAE visual, STL collision)
        export_meshes(model, snapshot, pkg_dir, config, log)

        # 3b: Capture robot screenshot for README (optional)
        if config.include_screenshot:
            log.section("PHASE 3: SCREENSHOT")
            from .core.fusion_extractor import capture_screenshot
            capture_screenshot(pkg_dir, log=log)

        # 3c: Generate package (collision resolve + xacro + flat URDF + launch)
        generate_package(model, config, log)

        # 3d: Generate validation report (only if debug is on)
        if config.include_debug:
            from .core.package_generator import generate_validation_report
            generate_validation_report(model, config, debug_dir)

        # ════════════════════════════════════════
        #  DONE
        # ════════════════════════════════════════
        log.section("EXPORT COMPLETE")
        # Save the export log unconditionally so a successful run still
        # has it on disk while we decide whether to keep the folder.
        log_path = os.path.join(debug_dir, "export_log.md")
        log.save_markdown(log_path, title=f"Export Log: {root.name}")

        # Clean-run cleanup: when the user opted out of debug via
        # ``include_debug = false``, remove the folder now that the
        # export succeeded.  We always wrote it earlier (so an aborted
        # run still has diagnostic data) — only delete on success.
        if not config.include_debug:
            try:
                import shutil as _shutil
                _shutil.rmtree(debug_dir)
                log(f"  Debug folder removed (config.include_debug=false)")
            except Exception as e:
                log.warning(f"  Could not remove debug folder: {e}")

        # Optional ZIP packaging — wrap the package directory in a
        # .zip and remove the directory.  Useful when shipping a
        # description package over a network or to a tool that wants
        # one file instead of a tree.
        if config.zip_output:
            import zipfile, shutil as _shutil
            zip_basename = config.zip_name or (config.package_name + ".zip")
            zip_path = os.path.join(save_dir, zip_basename)
            log(f"  Packaging → {zip_basename}")
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
                for dirpath, _, filenames in os.walk(pkg_dir):
                    for fn in filenames:
                        abs_fp = os.path.join(dirpath, fn)
                        # Store relative to save_dir so the zip contains
                        # the package root directory.
                        zf.write(abs_fp, os.path.relpath(abs_fp, save_dir))
            try:
                _shutil.rmtree(pkg_dir)
                log(f"  Removed unzipped directory")
            except Exception as e:
                log.warning(f"  Could not remove {pkg_dir}: {e}")
            pkg_dir = zip_path  # surface the zip path in the final message
            
        stop_brainrot(brainrot)
        warning_text = ""
        if model.warnings:
            warning_text = f"\n\nWarnings ({len(model.warnings)}):\n"
            for w in model.warnings[:5]:
                warning_text += f"  ! {w}\n"
            if len(model.warnings) > 5:
                warning_text += f"  ... and {len(model.warnings) - 5} more (see log)"

        ui.messageBox(
            f"Export complete!\n\n"
            f"Package: {config.package_name}/\n"
            f"  Links: {len(model.links)}\n"
            f"  Joints: {len(model.joints)}\n"
            f"  Assemblies: {len(model.assemblies)}\n"
            f"  Root: {model.root_link}\n\n"
            f"Xacro:  urdf/{model.name}.urdf.xacro\n"
            f"Launch: ros2 launch {config.package_name} display.launch.py\n\n"
            f"Debug:  debug/"
            f"{warning_text}",
            title
        )

    except Exception as e:
        msg = f"Export failed:\n\n{e}\n\n{traceback.format_exc()}"
        if ui:
            ui.messageBox(msg, title)
        stop_brainrot(brainrot)
        print(msg)

    finally:
        stop_brainrot(brainrot)

def stop(context):
    pass
