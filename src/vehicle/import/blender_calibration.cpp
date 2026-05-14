// Copyright (C) 2026 Abdul, Hashim.
//
// This file is part of CARLA Studio.
// Licensed under the GNU Affero General Public License v3 or later -
// see <LICENSE> at the project root or
// <https://www.gnu.org/licenses/agpl-3.0.html> for the full text.
// SPDX-License-Identifier: AGPL-3.0-or-later

#include "vehicle/import/blender_calibration.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QStandardPaths>
#include <QTextStream>

namespace carla_studio::vehicle_import {

// ---------------------------------------------------------------------------
// Embedded Blender Python script
//
// Receives arguments after "--":
//   mesh_path ext scale_cm wheel_fl_x,y,z wheel_fr_x,y,z wheel_rl_x,y,z wheel_rr_x,y,z out_dir
// ---------------------------------------------------------------------------
static const char kBlenderScript[] = R"PY(
import bpy, math, os, sys

def parse_vec3(s):
    return [float(v) for v in s.split(',')]

argv = sys.argv[sys.argv.index('--') + 1:]
mesh_path   = argv[0]
ext         = argv[1].lower()
scale_cm    = float(argv[2])   # cm per model unit → multiply vertex by scale_cm to get cm
has_wheels  = argv[3] == '1'
if has_wheels:
    wfl = parse_vec3(argv[4])
    wfr = parse_vec3(argv[5])
    wrl = parse_vec3(argv[6])
    wrr = parse_vec3(argv[7])
    out_dir = argv[8]
else:
    out_dir = argv[4]

os.makedirs(out_dir, exist_ok=True)

# ── Clear scene ──────────────────────────────────────────────────────────────
bpy.ops.wm.read_factory_settings(use_empty=True)

# ── Import mesh ──────────────────────────────────────────────────────────────
try:
    if ext == 'fbx':
        bpy.ops.import_scene.fbx(filepath=mesh_path, use_manual_orientation=False)
    elif ext in ('glb', 'gltf'):
        bpy.ops.import_scene.gltf(filepath=mesh_path)
    elif ext == 'dae':
        bpy.ops.wm.collada_import(filepath=mesh_path)
    else:  # obj - try new API first (Blender 4+), fall back to old
        try:
            bpy.ops.wm.obj_import(filepath=mesh_path)
        except AttributeError:
            bpy.ops.import_scene.obj(filepath=mesh_path)
except Exception as e:
    print(f'[blender-calib] import failed: {e}', flush=True)
    sys.exit(1)

# ── Scale all objects to metric cm then normalise to meters ─────────────────
# Vertices are already in model units. scale_cm converts 1 model unit → cm.
# Blender uses meters, so we scale by scale_cm / 100.
scale_m = scale_cm / 100.0
bpy.ops.object.select_all(action='SELECT')
bpy.ops.transform.resize(value=(scale_m, scale_m, scale_m))
bpy.ops.object.transform_apply(scale=True)

# ── Ground plane grid ────────────────────────────────────────────────────────
bpy.ops.mesh.primitive_grid_add(x_subdivisions=20, y_subdivisions=20,
                                 size=20.0, location=(0, 0, 0))
grid = bpy.context.active_object
grid.name = 'CalibGrid'
mat_grid = bpy.data.materials.new('GridMat')
mat_grid.use_nodes = True
nodes = mat_grid.node_tree.nodes
nodes.clear()
emit = nodes.new('ShaderNodeEmission')
emit.inputs['Color'].default_value = (0.15, 0.15, 0.15, 1.0)
emit.inputs['Strength'].default_value = 0.3
out = nodes.new('ShaderNodeOutputMaterial')
mat_grid.node_tree.links.new(emit.outputs[0], out.inputs[0])
grid.data.materials.append(mat_grid)
grid.display_type = 'WIRE'

# ── Wheel markers ────────────────────────────────────────────────────────────
WHEEL_COLORS = [(1,0,0,1),(0,1,0,1),(0,0,1,1),(1,1,0,1)]  # FL FR RL RR
WHEEL_LABELS = ['FL','FR','RL','RR']
if has_wheels:
    for idx, (xyz, color, label) in enumerate(
            zip([wfl, wfr, wrl, wrr], WHEEL_COLORS, WHEEL_LABELS)):
        x_m, y_m, z_m = xyz[0]/100.0, xyz[1]/100.0, xyz[2]/100.0
        bpy.ops.mesh.primitive_uv_sphere_add(radius=0.08,
                                              location=(x_m, y_m, z_m))
        sphere = bpy.context.active_object
        sphere.name = f'Wheel_{label}'
        mat = bpy.data.materials.new(f'WheelMat_{label}')
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        nodes.clear()
        em = nodes.new('ShaderNodeEmission')
        em.inputs['Color'].default_value = color
        em.inputs['Strength'].default_value = 2.0
        outn = nodes.new('ShaderNodeOutputMaterial')
        mat.node_tree.links.new(em.outputs[0], outn.inputs[0])
        sphere.data.materials.append(mat)

# ── Lighting ─────────────────────────────────────────────────────────────────
bpy.ops.object.light_add(type='SUN', location=(5, 5, 10))
sun = bpy.context.active_object
sun.data.energy = 2.0
bpy.ops.object.light_add(type='SUN', location=(-5, -5, 8))
sun2 = bpy.context.active_object
sun2.data.energy = 0.8
sun2.data.angle = math.radians(60)

# ── World background ─────────────────────────────────────────────────────────
world = bpy.data.worlds.new('World')
bpy.context.scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes['Background']
bg.inputs['Color'].default_value = (0.05, 0.05, 0.05, 1.0)
bg.inputs['Strength'].default_value = 0.5

# ── Render settings ──────────────────────────────────────────────────────────
scene = bpy.context.scene
scene.render.engine           = 'BLENDER_EEVEE_NEXT' if hasattr(bpy.types, 'EEVEE_NEXT_RenderSettings') else 'BLENDER_EEVEE'
scene.render.resolution_x     = 720
scene.render.resolution_y     = 720
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.render.film_transparent  = False

# EEVEE settings for speed
if hasattr(scene, 'eevee'):
    scene.eevee.taa_render_samples = 8
    scene.eevee.use_gtao            = True
    scene.eevee.gtao_distance       = 0.2

# ── Camera helper ────────────────────────────────────────────────────────────
cam_data = bpy.data.cameras.new('CalibCam')
cam_obj  = bpy.data.objects.new('CalibCam', cam_data)
bpy.context.collection.objects.link(cam_obj)
scene.camera = cam_obj

def render_view(name, cam_type, location, rotation_deg, ortho_scale=None):
    cam_data.type = cam_type
    cam_obj.location = location
    cam_obj.rotation_euler = [math.radians(r) for r in rotation_deg]
    if ortho_scale is not None:
        cam_data.ortho_scale = ortho_scale
    scene.render.filepath = os.path.join(out_dir, name + '.png')
    bpy.ops.render.render(write_still=True)
    print(f'[blender-calib] rendered {name}.png', flush=True)

# Compute bounding box to auto-fit camera
all_verts = []
for obj in bpy.context.scene.objects:
    if obj.type == 'MESH' and obj.name not in ('CalibGrid',) \
            and not obj.name.startswith('Wheel_'):
        mat = obj.matrix_world
        for v in obj.data.vertices:
            co = mat @ v.co
            all_verts.append((co.x, co.y, co.z))

if all_verts:
    xs = [v[0] for v in all_verts]
    ys = [v[1] for v in all_verts]
    zs = [v[2] for v in all_verts]
    cx = (min(xs)+max(xs))/2; cy = (min(ys)+max(ys))/2
    sz = max(max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs))
    dist = sz * 1.4
    scale_o = sz * 1.2
    top_z   = max(zs) + dist
else:
    cx = cy = 0.0; dist = 8.0; scale_o = 8.0; top_z = dist

# Top (orthographic, looking down)
render_view('top', 'ORTHO', (cx, cy, top_z), (0, 0, 0), scale_o)
# Side (orthographic, looking in -X)
render_view('side', 'ORTHO', (cx + dist, cy, 1.0), (90, 0, 90), scale_o)
# Front (orthographic, looking in +Y)
render_view('front', 'ORTHO', (cx, cy - dist, 1.0), (90, 0, 0), scale_o)
# 3D perspective
render_view('3d', 'PERSP', (cx + dist*0.7, cy - dist*0.7, dist*0.6),
            (60, 0, 45))

print('[blender-calib] all views rendered', flush=True)
)PY";

// ---------------------------------------------------------------------------

static QString findBlender() {
  // Check env override first, then PATH search.
  const QString envBlender = qEnvironmentVariable("CARLA_STUDIO_BLENDER");
  if (!envBlender.isEmpty() && QFileInfo(envBlender).isExecutable()) return envBlender;
  return QStandardPaths::findExecutable("blender");
}

BlenderCalibrationResult render_calibration_with_blender(
    const QString &mesh_path,
    const std::array<float, 12> &wheel_xyz_cm,
    const QString &out_dir)
{
  BlenderCalibrationResult res;

  const QString blender = findBlender();
  if (blender.isEmpty()) {
    res.log = "blender not found in PATH (set CARLA_STUDIO_BLENDER to override)";
    return res;
  }
  if (!QFileInfo::exists(mesh_path)) {
    res.log = "mesh not found: " + mesh_path;
    return res;
  }

  QDir().mkpath(out_dir);

  // Write the embedded Python script to a temp file.
  const QString scriptPath = QDir(out_dir).filePath("__calib_render.py");
  {
    QFile f(scriptPath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
      res.log = "cannot write script to " + scriptPath;
      return res;
    }
    f.write(kBlenderScript);
  }

  // Resolve symlinks so Blender can find sibling MTL/texture files.
  const QString resolved = QFileInfo(mesh_path).canonicalFilePath();
  const QString actual   = resolved.isEmpty() ? mesh_path : resolved;
  const QString ext = QFileInfo(actual).suffix();

  // Wheel args: scale_cm is 1 (mesh already in cm for carla-studio)
  QStringList args = {
    "--background",
    "--python", scriptPath,
    "--",
    actual,
    ext,
    "1",  // scale_cm: carla-studio verts are already in cm
    "1",  // has_wheels = true
    QString("%1,%2,%3").arg(wheel_xyz_cm[0]).arg(wheel_xyz_cm[1]).arg(wheel_xyz_cm[2]),
    QString("%1,%2,%3").arg(wheel_xyz_cm[3]).arg(wheel_xyz_cm[4]).arg(wheel_xyz_cm[5]),
    QString("%1,%2,%3").arg(wheel_xyz_cm[6]).arg(wheel_xyz_cm[7]).arg(wheel_xyz_cm[8]),
    QString("%1,%2,%3").arg(wheel_xyz_cm[9]).arg(wheel_xyz_cm[10]).arg(wheel_xyz_cm[11]),
    out_dir,
  };

  QProcess p;
  p.setProcessChannelMode(QProcess::MergedChannels);
  p.start(blender, args);
  if (!p.waitForStarted(10000)) {
    res.log = "blender failed to start";
    return res;
  }
  p.waitForFinished(300000);  // 5 min max
  res.log = QString::fromLocal8Bit(p.readAll()).right(2000);

  res.top_png         = QDir(out_dir).filePath("top.png");
  res.side_png        = QDir(out_dir).filePath("side.png");
  res.front_png       = QDir(out_dir).filePath("front.png");
  res.perspective_png = QDir(out_dir).filePath("3d.png");

  res.ok = QFileInfo(res.top_png).isFile()
        && QFileInfo(res.side_png).isFile()
        && QFileInfo(res.front_png).isFile();
  return res;
}

BlenderCalibrationResult render_calibration_with_blender(
    const QString &mesh_path,
    const QString &out_dir)
{
  BlenderCalibrationResult res;

  const QString blender = findBlender();
  if (blender.isEmpty()) {
    res.log = "blender not found in PATH";
    return res;
  }
  if (!QFileInfo::exists(mesh_path)) {
    res.log = "mesh not found: " + mesh_path;
    return res;
  }

  QDir().mkpath(out_dir);

  const QString scriptPath = QDir(out_dir).filePath("__calib_render.py");
  {
    QFile f(scriptPath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
      res.log = "cannot write script";
      return res;
    }
    f.write(kBlenderScript);
  }

  const QString resolved2 = QFileInfo(mesh_path).canonicalFilePath();
  const QString actual2   = resolved2.isEmpty() ? mesh_path : resolved2;
  const QString ext = QFileInfo(actual2).suffix();

  QStringList args = {
    "--background",
    "--python", scriptPath,
    "--",
    actual2,
    ext,
    "1",  // scale_cm
    "0",  // has_wheels = false
    out_dir,
  };

  QProcess p;
  p.setProcessChannelMode(QProcess::MergedChannels);
  p.start(blender, args);
  if (!p.waitForStarted(10000)) {
    res.log = "blender failed to start";
    return res;
  }
  p.waitForFinished(300000);
  res.log = QString::fromLocal8Bit(p.readAll()).right(2000);

  res.top_png         = QDir(out_dir).filePath("top.png");
  res.side_png        = QDir(out_dir).filePath("side.png");
  res.front_png       = QDir(out_dir).filePath("front.png");
  res.perspective_png = QDir(out_dir).filePath("3d.png");

  res.ok = QFileInfo(res.top_png).isFile()
        && QFileInfo(res.side_png).isFile()
        && QFileInfo(res.front_png).isFile();
  return res;
}

}
