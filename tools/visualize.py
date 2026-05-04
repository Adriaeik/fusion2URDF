"""
Visualize — Generate interactive 3D visualization of robot kinematic chain.

Reads snapshot.json, produces a standalone HTML file with:
  - Links as spheres (color-coded by assembly, sized by mass)
  - Joints as lines connecting parent → child
  - Joint axes shown as arrows
  - Assembly grouping
  - Click on objects for detailed properties

Usage (from the fusion2URDF directory):
    python tools/visualize.py snapshot.json [-o output.html]

Or from the parent directory:
    python -m fusion2URDF.tools.visualize path/to/snapshot.json

No external dependencies — only stdlib.

Author: Adrian Valaker Eikeland
"""

import json
import math
import os
import sys
import html as html_lib


# Dynamic assembly color palette — assigned in discovery order
COLOR_PALETTE = [
    '#e74c3c',  # Red
    '#3498db',  # Blue
    '#2ecc71',  # Green
    '#9b59b6',  # Purple
    '#e67e22',  # Orange
    '#1abc9c',  # Teal
    '#f39c12',  # Yellow
    '#d35400',  # Dark Orange
    '#c0392b',  # Dark Red
    '#16a085',  # Dark Teal
    '#8e44ad',  # Dark Purple
    '#2980b9',  # Dark Blue
    '#27ae60',  # Dark Green
    '#f1c40f',  # Gold
    '#e91e63',  # Pink
]
ROOT_COLOR = '#95a5a6'  # Gray for root assembly


def generate_visualization(snapshot_path: str, output_path: str = None, model=None):
    """Generate interactive HTML visualization from snapshot.json."""
    
    with open(snapshot_path) as f:
        snap = json.load(f)
    
    if output_path is None:
        output_path = os.path.splitext(snapshot_path)[0] + '_viz.html'
    
    # Parse occurrences and joints
    links = []
    for path, occ in snap['occurrences'].items():
        if occ['is_subassembly']:
            continue
        
        segs = occ['path_segments']
        asm = segs[-2] if len(segs) >= 2 else 'ROOT'
        gp = occ['global_position']
        
        links.append({
            'name': occ['clean_name'],
            'assembly': asm,
            'x': gp[0] * 1000,  # mm for display
            'y': gp[1] * 1000,
            'z': gp[2] * 1000,
            'mass_g': occ['mass_kg'] * 1000,
            'material': occ.get('material_name', ''),
            'path': path,
            'com_global': [v * 1000 for v in occ.get('com_global', [0, 0, 0])],
        })
    
    # Parse joints — resolve origins to global mm
    joint_lines = []
    assembly_offsets = {}
    for path, occ in snap['occurrences'].items():
        if occ['is_subassembly']:
            assembly_offsets[occ['clean_name']] = [v * 1000 for v in occ['global_position']]
    
    for jname, j in snap['joints'].items():
        origin = [v * 1000 for v in j['origin_global_m']]
        source = j['origin_source']
        defining = j['defining_component']
        
        # Apply assembly offset if origin is assembly-local
        if source in ('geometry.origin', 'geometryOrOriginOne', 'geometryOrOriginTwo'):
            if defining in assembly_offsets and defining != snap.get('design_name_clean', ''):
                off = assembly_offsets[defining]
                origin = [origin[i] + off[i] for i in range(3)]
        
        # Find parent and child positions
        parent_pos = _find_link_pos(j['occurrence_two_clean'], j['occurrence_two_path'],
                                     j['defining_component'], links)
        child_pos = _find_link_pos(j['occurrence_one_clean'], j['occurrence_one_path'],
                                    j['defining_component'], links)
        
        # Axis
        axis = j.get('axis_vector', [0, 0, 1])
        axis_len = 30  # mm for display
        
        joint_lines.append({
            'name': jname,
            'type': j['motion_type'],
            'origin': origin,
            'parent_pos': parent_pos,
            'child_pos': child_pos,
            'parent_name': j['occurrence_two_clean'],
            'child_name': j['occurrence_one_clean'],
            'axis': axis,
            'axis_end': [origin[i] + axis[i] * axis_len for i in range(3)],
            'source': source,
            'defining': defining,
        })
    
    # Generate HTML
    html_content = _generate_html(snap, links, joint_lines, model)
    
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html_content)
    
    print(f"Visualization saved to: {output_path}")
    return output_path


def _find_link_pos(clean_name, local_path, defining, links):
    """Find link position by matching name and assembly context."""
    # Try exact match by assembly
    for l in links:
        if l['name'] == clean_name and (l['assembly'] == defining or local_path in l.get('path', '')):
            return [l['x'], l['y'], l['z']]
    # Fallback: first match by name
    for l in links:
        if l['name'] == clean_name:
            return [l['x'], l['y'], l['z']]
    return [0, 0, 0]


def _generate_html(snap, links, joints, model=None):
    """Generate standalone HTML with Three.js visualization."""
    
    design_name = snap.get('design_name_clean', 'robot')
    
    # Assign colors to assemblies dynamically
    all_asms = sorted(set(l['assembly'] for l in links))
    asm_colors = {}
    color_idx = 0
    for asm in all_asms:
        if asm == 'ROOT':
            asm_colors[asm] = ROOT_COLOR
        else:
            asm_colors[asm] = COLOR_PALETTE[color_idx % len(COLOR_PALETTE)]
            color_idx += 1
    
    # Serialize data to JSON for embedding
    links_json = json.dumps(links)
    joints_json = json.dumps(joints)
    colors_json = json.dumps(asm_colors)
    
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>{html_lib.escape(design_name)} — Joint Visualization</title>
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ font-family: -apple-system, 'Segoe UI', sans-serif; background: #1a1a2e; color: #eee; overflow: hidden; }}
#canvas-container {{ width: 100vw; height: 100vh; }}
canvas {{ display: block; }}
#info {{
    position: absolute; top: 12px; left: 12px;
    background: rgba(0,0,0,0.8); padding: 12px 16px; border-radius: 8px;
    font-size: 13px; line-height: 1.5; max-width: 350px; pointer-events: none;
}}
#info h2 {{ font-size: 15px; margin-bottom: 6px; color: #3498db; }}
#legend {{
    position: absolute; bottom: 12px; left: 12px;
    background: rgba(0,0,0,0.8); padding: 10px 14px; border-radius: 8px;
    font-size: 12px;
}}
.legend-item {{ display: flex; align-items: center; gap: 6px; margin: 3px 0; }}
.legend-dot {{ width: 10px; height: 10px; border-radius: 50%; }}
#tooltip {{
    position: absolute; display: none; background: rgba(0,0,0,0.9);
    padding: 8px 12px; border-radius: 6px; font-size: 12px; pointer-events: none;
    border: 1px solid #555; max-width: 400px; white-space: pre-line;
}}
#controls {{
    position: absolute; top: 12px; right: 12px;
    background: rgba(0,0,0,0.8); padding: 10px 14px; border-radius: 8px;
    font-size: 12px;
}}
#controls label {{ display: block; margin: 3px 0; cursor: pointer; }}
#controls input {{ margin-right: 6px; }}
</style>
</head>
<body>
<div id="canvas-container"></div>
<div id="info">
    <h2>{html_lib.escape(design_name)}</h2>
    <div>Links: {len(links)} | Joints: {len(joints)}</div>
    <div style="margin-top:4px;font-size:11px;color:#888">Drag to rotate · Scroll to zoom · Click link/joint for details</div>
</div>
<div id="legend"></div>
<div id="tooltip"></div>
<div id="controls">
    <strong>Show:</strong>
    <label><input type="checkbox" id="show-links" checked> Links</label>
    <label><input type="checkbox" id="show-joints" checked> Joint connections</label>
    <label><input type="checkbox" id="show-axes" checked> Joint axes</label>
    <label><input type="checkbox" id="show-labels" checked> Labels</label>
    <label><input type="checkbox" id="show-com" > Center of Mass</label>
    <label><input type="checkbox" id="show-origin" checked> Origin axes</label>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
// Data
const links = {links_json};
const joints = {joints_json};
const asmColors = {colors_json};

// Scene setup
const container = document.getElementById('canvas-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a2e);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 10000);
camera.position.set(800, 600, 800);
camera.lookAt(0, 300, 0);

const renderer = new THREE.WebGLRenderer({{ antialias: true }});
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);

// Lights
scene.add(new THREE.AmbientLight(0x404040, 0.6));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(500, 500, 500);
scene.add(dirLight);
scene.add(new THREE.DirectionalLight(0xffffff, 0.3).position.set(-300, 200, -300));

// Groups for toggle
const linkGroup = new THREE.Group();
const jointGroup = new THREE.Group();
const axisGroup = new THREE.Group();
const labelGroup = new THREE.Group();
const comGroup = new THREE.Group();
const originGroup = new THREE.Group();
scene.add(linkGroup, jointGroup, axisGroup, labelGroup, comGroup, originGroup);

// Grid and origin
const gridHelper = new THREE.GridHelper(1000, 20, 0x333355, 0x222244);
scene.add(gridHelper);

// Origin axes (X=red, Y=green, Z=blue) 100mm each
function addOriginAxes() {{
    const len = 100;
    const axes = [
        [new THREE.Vector3(len,0,0), 0xff4444, 'X'],
        [new THREE.Vector3(0,len,0), 0x44ff44, 'Y'],
        [new THREE.Vector3(0,0,len), 0x4444ff, 'Z']
    ];
    axes.forEach(([dir, color, label]) => {{
        const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0,0,0), dir]);
        const mat = new THREE.LineBasicMaterial({{ color, linewidth: 2 }});
        originGroup.add(new THREE.Line(geo, mat));
    }});
}}
addOriginAxes();

// Clickable objects for raycasting
const clickables = [];

// Create link spheres
links.forEach(l => {{
    const radius = Math.max(8, Math.min(30, Math.log10(Math.max(l.mass_g, 1)) * 10));
    const color = new THREE.Color(asmColors[l.assembly] || '#888888');
    
    const geo = new THREE.SphereGeometry(radius, 16, 12);
    const mat = new THREE.MeshPhongMaterial({{ color, transparent: true, opacity: 0.85 }});
    const mesh = new THREE.Mesh(geo, mat);
    // Three.js uses Y-up; Fusion uses Z-up
    mesh.position.set(l.x, l.z, -l.y);
    mesh.userData = {{ type: 'link', data: l }};
    linkGroup.add(mesh);
    clickables.push(mesh);
    
    // CoM marker
    const comGeo = new THREE.SphereGeometry(4, 8, 6);
    const comMat = new THREE.MeshBasicMaterial({{ color: 0xffff00 }});
    const comMesh = new THREE.Mesh(comGeo, comMat);
    comMesh.position.set(l.com_global[0], l.com_global[2], -l.com_global[1]);
    comGroup.add(comMesh);
}});

// Create joint lines and axis arrows
joints.forEach(j => {{
    if (j.parent_pos && j.child_pos) {{
        // Connection line: parent → child
        const p = new THREE.Vector3(j.parent_pos[0], j.parent_pos[2], -j.parent_pos[1]);
        const c = new THREE.Vector3(j.child_pos[0], j.child_pos[2], -j.child_pos[1]);
        
        const lineColor = j.type === 'revolute' ? 0xff8800 :
                          j.type === 'slider' ? 0x00ff88 :
                          j.type === 'rigid' ? 0x888888 : 0xffffff;
        
        const geo = new THREE.BufferGeometry().setFromPoints([p, c]);
        const mat = new THREE.LineBasicMaterial({{ color: lineColor, linewidth: 1 }});
        const line = new THREE.Line(geo, mat);
        line.userData = {{ type: 'joint', data: j }};
        jointGroup.add(line);
        
        // Joint origin marker
        const o = new THREE.Vector3(j.origin[0], j.origin[2], -j.origin[1]);
        const markerGeo = new THREE.OctahedronGeometry(5);
        const markerMat = new THREE.MeshBasicMaterial({{ color: lineColor }});
        const marker = new THREE.Mesh(markerGeo, markerMat);
        marker.position.copy(o);
        marker.userData = {{ type: 'joint', data: j }};
        jointGroup.add(marker);
        clickables.push(marker);
        
        // Axis arrow (for revolute/prismatic)
        if (j.type !== 'rigid') {{
            const axisLen = 40;
            const axisStart = o.clone();
            const axisEnd = new THREE.Vector3(
                j.origin[0] + j.axis[0] * axisLen,
                j.origin[2] + j.axis[2] * axisLen,
                -(j.origin[1] + j.axis[1] * axisLen)
            );
            const axGeo = new THREE.BufferGeometry().setFromPoints([axisStart, axisEnd]);
            const axMat = new THREE.LineBasicMaterial({{ color: 0xff00ff, linewidth: 2 }});
            axisGroup.add(new THREE.Line(axGeo, axMat));
            
            // Arrowhead
            const coneGeo = new THREE.ConeGeometry(3, 10, 6);
            const coneMat = new THREE.MeshBasicMaterial({{ color: 0xff00ff }});
            const cone = new THREE.Mesh(coneGeo, coneMat);
            cone.position.copy(axisEnd);
            cone.lookAt(axisStart);
            cone.rotateX(Math.PI / 2);
            axisGroup.add(cone);
        }}
    }}
}});

// Legend
const legendDiv = document.getElementById('legend');
const allAsms = [...new Set(links.map(l => l.assembly))].sort();
allAsms.forEach(asm => {{
    const color = asmColors[asm] || '#888';
    const count = links.filter(l => l.assembly === asm).length;
    const item = document.createElement('div');
    item.className = 'legend-item';
    item.innerHTML = `<span class="legend-dot" style="background:${{color}}"></span>${{asm}} (${{count}} links)`;
    legendDiv.appendChild(item);
}});
// Joint type legend
['revolute:#ff8800', 'prismatic:#00ff88', 'fixed:#888888'].forEach(entry => {{
    const [type, color] = entry.split(':');
    const item = document.createElement('div');
    item.className = 'legend-item';
    item.innerHTML = `<span class="legend-dot" style="background:${{color}};border-radius:2px"></span>${{type}} joint`;
    legendDiv.appendChild(item);
}});

// Camera controls (simple orbit)
let isDragging = false, prevMouse = {{ x: 0, y: 0 }};
let spherical = {{ r: 1200, theta: Math.PI / 4, phi: Math.PI / 4 }};
let target = new THREE.Vector3(100, 250, 0);

function updateCamera() {{
    camera.position.set(
        target.x + spherical.r * Math.sin(spherical.theta) * Math.cos(spherical.phi),
        target.y + spherical.r * Math.cos(spherical.theta),
        target.z + spherical.r * Math.sin(spherical.theta) * Math.sin(spherical.phi)
    );
    camera.lookAt(target);
}}
updateCamera();

renderer.domElement.addEventListener('mousedown', e => {{
    isDragging = true;
    prevMouse = {{ x: e.clientX, y: e.clientY }};
}});
renderer.domElement.addEventListener('mousemove', e => {{
    if (!isDragging) return;
    const dx = e.clientX - prevMouse.x;
    const dy = e.clientY - prevMouse.y;
    spherical.phi -= dx * 0.005;
    spherical.theta = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.theta + dy * 0.005));
    prevMouse = {{ x: e.clientX, y: e.clientY }};
    updateCamera();
}});
renderer.domElement.addEventListener('mouseup', () => isDragging = false);
renderer.domElement.addEventListener('wheel', e => {{
    spherical.r = Math.max(100, spherical.r + e.deltaY * 0.5);
    updateCamera();
}});

// Click for details
const tooltip = document.getElementById('tooltip');
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

renderer.domElement.addEventListener('click', e => {{
    mouse.x = (e.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(e.clientY / window.innerHeight) * 2 + 1;
    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects(clickables);
    
    if (intersects.length > 0) {{
        const obj = intersects[0].object;
        const ud = obj.userData;
        
        if (ud.type === 'link') {{
            const d = ud.data;
            tooltip.innerHTML = `<strong>${{d.name}}</strong> [${{d.assembly}}]\\n` +
                `Position: (${{d.x.toFixed(1)}}, ${{d.y.toFixed(1)}}, ${{d.z.toFixed(1)}}) mm\\n` +
                `Mass: ${{d.mass_g.toFixed(1)}} g\\n` +
                `Material: ${{d.material}}\\n` +
                `CoM: (${{d.com_global[0].toFixed(1)}}, ${{d.com_global[1].toFixed(1)}}, ${{d.com_global[2].toFixed(1)}}) mm`;
        }} else if (ud.type === 'joint') {{
            const d = ud.data;
            tooltip.innerHTML = `<strong>${{d.name}}</strong> [${{d.type}}]\\n` +
                `${{d.parent_name}} → ${{d.child_name}}\\n` +
                `Origin: (${{d.origin[0].toFixed(1)}}, ${{d.origin[1].toFixed(1)}}, ${{d.origin[2].toFixed(1)}}) mm\\n` +
                `Axis: (${{d.axis[0].toFixed(2)}}, ${{d.axis[1].toFixed(2)}}, ${{d.axis[2].toFixed(2)}})\\n` +
                `Source: ${{d.source}} (in ${{d.defining}})`;
        }}
        tooltip.style.display = 'block';
        tooltip.style.left = (e.clientX + 15) + 'px';
        tooltip.style.top = (e.clientY + 15) + 'px';
    }} else {{
        tooltip.style.display = 'none';
    }}
}});

// Toggle controls
document.getElementById('show-links').onchange = e => linkGroup.visible = e.target.checked;
document.getElementById('show-joints').onchange = e => jointGroup.visible = e.target.checked;
document.getElementById('show-axes').onchange = e => axisGroup.visible = e.target.checked;
document.getElementById('show-labels').onchange = e => labelGroup.visible = e.target.checked;
document.getElementById('show-com').onchange = e => comGroup.visible = e.target.checked;
document.getElementById('show-origin').onchange = e => originGroup.visible = e.target.checked;
comGroup.visible = false;  // Off by default

// Render loop
function animate() {{
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}}
animate();

// Resize
window.addEventListener('resize', () => {{
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}});
</script>
</body>
</html>"""


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python visualize.py <snapshot.json> [-o output.html]")
        sys.exit(1)
    
    snap_path = sys.argv[1]
    out_path = None
    if '-o' in sys.argv:
        idx = sys.argv.index('-o')
        if idx + 1 < len(sys.argv):
            out_path = sys.argv[idx + 1]
    
    generate_visualization(snap_path, out_path)
