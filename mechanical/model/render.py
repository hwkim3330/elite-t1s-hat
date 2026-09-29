"""Render README previews of the STL models with Blender, headless:

    blender -b --python mechanical/model/render.py

writes render-hat.png and render-stack.png next to this file. Plain studio lighting on a
white ground: the models carry no colour, so the render gives every part one neutral
material and lets shape and shadow do the work.
"""
import math
import os

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.samples = 64
    sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = 1600, 1000
    sc.render.film_transparent = False
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (1, 1, 1, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.35
    sc.view_settings.view_transform = "Filmic" if "Filmic" in [v.identifier for v in type(sc.view_settings).bl_rna.properties["view_transform"].enum_items] else "AgX"
    sc.world = world
    return sc


def material(name, rgb, rough=0.5, metal=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*rgb, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    return m


def load(path, mat):
    bpy.ops.wm.stl_import(filepath=path) if hasattr(bpy.ops.wm, "stl_import") else bpy.ops.import_mesh.stl(filepath=path)
    ob = bpy.context.selected_objects[0]
    ob.data.materials.append(mat)
    return ob


def ground(z):
    bpy.ops.mesh.primitive_plane_add(size=400, location=(33, 25, z))
    bpy.context.object.data.materials.append(material("ground", (0.92, 0.92, 0.93), rough=0.95))


def camera(target, dist, elev, azim, lens=60):
    t = target
    x = t[0] + dist * math.cos(math.radians(elev)) * math.cos(math.radians(azim))
    y = t[1] + dist * math.cos(math.radians(elev)) * math.sin(math.radians(azim))
    z = t[2] + dist * math.sin(math.radians(elev))
    bpy.ops.object.camera_add(location=(x, y, z))
    cam = bpy.context.object
    cam.data.lens = lens
    d = (t[0] - x, t[1] - y, t[2] - z)
    cam.rotation_euler = (math.atan2(math.hypot(d[0], d[1]), -d[2]), 0, math.atan2(d[1], d[0]) - math.pi / 2)
    bpy.context.scene.camera = cam


def lights(target):
    for loc, e in (((target[0] - 60, target[1] - 80, 120), 9e4), ((target[0] + 90, target[1] + 40, 80), 3e4)):
        bpy.ops.object.light_add(type="AREA", location=loc)
        L = bpy.context.object
        L.data.energy, L.data.size = e, 80
        d = (target[0] - loc[0], target[1] - loc[1], target[2] - loc[2])
        L.rotation_euler = (math.atan2(math.hypot(d[0], d[1]), -d[2]), 0, math.atan2(d[1], d[0]) - math.pi / 2)


def shot(out):
    bpy.context.scene.render.filepath = out
    bpy.ops.render.render(write_still=True)


# the HAT alone
reset()
load(os.path.join(HERE, "_parts", "hat_board.stl"), material("pcb", (0.05, 0.22, 0.11), rough=0.4))
load(os.path.join(HERE, "_parts", "j1_socket.stl"), material("black", (0.02, 0.02, 0.02), rough=0.35))
ground(-8.6)
lights((33, 25, 0))
camera((33, 23, -1), 175, 38, -65)
shot(os.path.join(HERE, "render-hat.png"))

# the stack on the Elite
reset()
P = os.path.join(HERE, "_parts")
load(os.path.join(P, "elite.stl"), material("elite", (0.06, 0.06, 0.07), rough=0.45))
load(os.path.join(P, "hat.stl"), material("hat", (0.05, 0.22, 0.11), rough=0.4))
load(os.path.join(P, "connectors.stl"), material("black", (0.02, 0.02, 0.02), rough=0.35))
load(os.path.join(P, "standoffs.stl"), material("brass", (0.78, 0.6, 0.3), rough=0.3, metal=1.0))
ground(-2.9)
lights((33, 25, 12))
camera((32, 24, 10), 225, 22, -128)
shot(os.path.join(HERE, "render-stack.png"))
