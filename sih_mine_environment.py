"""
SIH Mine Environment Generator
================================================================
Smart India Hackathon prototype:
"Safe and Efficient Operation of Mine Vehicles in Fog and
Low-Visibility Conditions in Open Cast Iron Ore Mines."

Target stack : Blender 5.2.1 LTS  ->  export (OBJ/DAE/GLB)  ->  Gazebo Sim
Purpose      : Lightweight, low-poly, simulation-friendly open-pit mine
               terrain, haul roads/ramps, navigation markers and a
               small set of safety props.

LAYOUT LOGIC (important - read before tweaking constants):
  Each bench is modelled as a square "picture-frame" ring (outer square
  minus a smaller inner square = a hole, through which the next, lower
  bench is visible). Because the middle of a ring is an open hole, any
  road/ramp path must stay on solid rectangular strips of a ring and
  must not cut straight across the hole. This script keeps the entire
  spawn -> ramp -> ramp -> loading route on the same (+Y / "north") side
  of the pit the whole way down, which is what keeps every road segment
  resting on solid ground instead of floating over the hole below it.

Run:
  Blender 5.2.1 -> Scripting workspace -> open this file -> Run Script (Alt+P)
  Safe to run repeatedly: all previously generated SIH objects/collections
  are deleted at the start of every run.

NOTE: No trucks, no sensors, no fog, no ROS 2 code, no dashboard here.
      This script ONLY builds the static mine environment.
================================================================
"""

import bpy
import math
import random

random.seed(42)

# ============================================================
# CONFIG  (all tunable - the layout below is derived from these)
# ============================================================
PREFIX = "SIH_"

COLLECTION_NAMES = {
    "ROOT": "SIH_MINE",
    "TERRAIN": "TERRAIN",
    "ROADS": "ROADS",
    "MARKERS": "MARKERS",
    "SAFETY": "SAFETY_OBJECTS",
}

# --- Bench / pit geometry -----------------------------------
# Three descending square "ring" benches + a flat floor filling the
# innermost hole. Ring widths are uneven on purpose: the top ring only
# needs to hold the spawn apron, while the lower two rings are widened
# so the ramps that cross them get a gentler slope.
OUTER0 = 95.0                      # outer half-extent of the top bench (m)
RING_WIDTHS = [15.0, 32.0, 33.0]   # width of ring0, ring1, ring2 bands (m)
BENCH_HEIGHT = 7.0                 # vertical drop per bench step (m)

OUTER = [OUTER0,
         OUTER0 - RING_WIDTHS[0],
         OUTER0 - RING_WIDTHS[0] - RING_WIDTHS[1]]
INNER = [OUTER[0] - RING_WIDTHS[0],
         OUTER[1] - RING_WIDTHS[1],
         OUTER[2] - RING_WIDTHS[2]]
Z_LEVELS = [0.0, -BENCH_HEIGHT, -2 * BENCH_HEIGHT]
FLOOR_HALF = INNER[2]              # 15 -> 30m x 30m flat mine floor
FLOOR_Z = Z_LEVELS[2]              # floor is flush with ring2 (no extra step)

# --- Roads / ramps -----------------------------------------
ROAD_WIDTH = 14.0                  # wide enough for two-lane heavy-truck traffic
ROAD_THICK = 0.3
RAMP_RUN_1 = 26.0                  # horizontal run of ramp 1 (< RING_WIDTHS[1])
RAMP_RUN_2 = 28.0                  # horizontal run of ramp 2 (< RING_WIDTHS[2])
# NOTE: resulting grades are ~25-27%, steeper than a real haul road (~8-10%).
# This is an intentional simplification to fit switch-back-free ramps inside
# a 200m x 200m envelope within a 20-hour build budget. Widen RING_WIDTHS /
# lower BENCH_HEIGHT / raise RAMP_RUN_* if gentler ramps are needed later.

# ============================================================
# CLEANUP  (run-repeatedly safety)
# ============================================================
def clear_existing():
    if bpy.context.object and bpy.context.object.mode != 'OBJECT':
        try:
            bpy.ops.object.mode_set(mode='OBJECT')
        except RuntimeError:
            pass

    names = [
        COLLECTION_NAMES["TERRAIN"],
        COLLECTION_NAMES["ROADS"],
        COLLECTION_NAMES["MARKERS"],
        COLLECTION_NAMES["SAFETY"],
        COLLECTION_NAMES["ROOT"],
    ]
    for name in names:
        col = bpy.data.collections.get(name)
        if col is None:
            continue
        for obj in list(col.objects):
            mesh = obj.data if obj.type == 'MESH' else None
            bpy.data.objects.remove(obj, do_unlink=True)
            if mesh is not None and mesh.users == 0:
                bpy.data.meshes.remove(mesh)
        for parent in list(bpy.data.collections):
            if col.name in [c.name for c in parent.children]:
                parent.children.unlink(col)
        if col.name in [c.name for c in bpy.context.scene.collection.children]:
            bpy.context.scene.collection.children.unlink(col)
        bpy.data.collections.remove(col)


def setup_collections():
    root = bpy.data.collections.new(COLLECTION_NAMES["ROOT"])
    bpy.context.scene.collection.children.link(root)
    subs = {}
    for key in ["TERRAIN", "ROADS", "MARKERS", "SAFETY"]:
        col = bpy.data.collections.new(COLLECTION_NAMES[key])
        root.children.link(col)
        subs[key] = col
    return root, subs


# ============================================================
# MATERIALS  (simple procedural Principled BSDF, no textures)
# ============================================================
def get_or_create_material(name, color, roughness=0.9, emission_strength=0.0):
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name)
        mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf is not None:
        bsdf.inputs["Base Color"].default_value = (*color, 1.0)
        bsdf.inputs["Roughness"].default_value = roughness
        if emission_strength > 0.0 and "Emission Color" in bsdf.inputs:
            bsdf.inputs["Emission Color"].default_value = (*color, 1.0)
            bsdf.inputs["Emission Strength"].default_value = emission_strength
    mat.diffuse_color = (*color, 1.0)
    return mat


def build_materials():
    return {
        "rock": get_or_create_material(f"{PREFIX}Mat_Rock", (0.42, 0.32, 0.24)),
        "road": get_or_create_material(f"{PREFIX}Mat_Road", (0.55, 0.55, 0.55), roughness=0.95),
        "safety": get_or_create_material(f"{PREFIX}Mat_Safety", (0.95, 0.55, 0.05), roughness=0.6),
        "marker": get_or_create_material(f"{PREFIX}Mat_Marker", (0.0, 0.9, 1.0), roughness=0.3,
                                          emission_strength=1.5),
    }


# ============================================================
# LOW-LEVEL MESH HELPERS
# ============================================================
def _link(mesh, name, collection, material=None, location=(0.0, 0.0, 0.0)):
    obj = bpy.data.objects.new(name, mesh)
    obj.location = location
    collection.objects.link(obj)
    if material is not None:
        obj.data.materials.append(material)
    return obj


def make_rect_ring(name, outer_half, inner_half, z, collection, material):
    """
    Square 'picture-frame' bench built from 4 AXIS-ALIGNED rectangular
    strips (not mitred/trapezoid corners). This keeps every strip a
    simple, predictable rectangle so road placement math stays exact:
      North: x in [-O,O], y in [ I, O]
      South: x in [-O,O], y in [-O,-I]
      East : x in [ I,O], y in [-I, I]
      West : x in [-O,-I], y in [-I, I]
    8 quads total - still very low-poly.
    """
    o, i = outer_half, inner_half
    verts = [
        # North strip (0-3)
        (-o, i, z), (o, i, z), (o, o, z), (-o, o, z),
        # South strip (4-7)
        (-o, -o, z), (o, -o, z), (o, -i, z), (-o, -i, z),
        # East strip (8-11)
        (i, -i, z), (o, -i, z), (o, i, z), (i, i, z),
        # West strip (12-15)
        (-o, -i, z), (-i, -i, z), (-i, i, z), (-o, i, z),
    ]
    faces = [
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (8, 9, 10, 11),
        (12, 13, 14, 15),
    ]
    mesh = bpy.data.meshes.new(name + "_Mesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    return _link(mesh, name, collection, material)


def make_wall(name, half, z_top, z_bottom, collection, material):
    """Vertical cliff face (constant footprint) connecting two bench levels."""
    h = half
    top_v = [(-h, -h, z_top), (h, -h, z_top), (h, h, z_top), (-h, h, z_top)]
    bot_v = [(-h, -h, z_bottom), (h, -h, z_bottom), (h, h, z_bottom), (-h, h, z_bottom)]
    verts = top_v + bot_v
    faces = [(0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    mesh = bpy.data.meshes.new(name + "_Mesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    return _link(mesh, name, collection, material)


def make_floor(name, half, z, collection, material):
    h = half
    verts = [(-h, -h, z), (h, -h, z), (h, h, z), (-h, h, z)]
    faces = [(0, 1, 2, 3)]
    mesh = bpy.data.meshes.new(name + "_Mesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    return _link(mesh, name, collection, material)


def relink(obj, collection):
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    collection.objects.link(obj)


def add_box(name, location, dims, collection, material=None, rot_x=0.0, rot_z=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    obj = bpy.context.active_object
    obj.name = name
    obj.dimensions = dims
    obj.rotation_euler[0] = rot_x
    obj.rotation_euler[2] = rot_z
    relink(obj, collection)
    if material is not None:
        obj.data.materials.append(material)
    return obj


def add_ramp(name, y_start, z_start, y_end, z_end, width, collection, material):
    """Sloped haul-road segment between two bench levels (x=0 centreline)."""
    dy = y_end - y_start
    dz = z_end - z_start
    length = math.sqrt(dy * dy + dz * dz)
    mid_y = (y_start + y_end) / 2.0
    mid_z = (z_start + z_end) / 2.0
    angle = math.atan2(dz, dy)
    obj = add_box(name, (0.0, mid_y, mid_z), (width, length, ROAD_THICK), collection, material)
    obj.rotation_euler[0] = angle
    return obj


def add_flat_road(name, x, y, z_surface, length_x, length_y, collection, material):
    """Straight road/pad resting ON a bench surface (never floating/buried)."""
    z = z_surface + ROAD_THICK / 2.0
    return add_box(name, (x, y, z), (length_x, length_y, ROAD_THICK), collection, material)


def add_cone_marker(name, location, collection, material, radius=2.2, depth=4.0):
    bpy.ops.mesh.primitive_cone_add(radius1=radius, radius2=0.0, depth=depth, location=location)
    obj = bpy.context.active_object
    obj.name = name
    relink(obj, collection)
    if material is not None:
        obj.data.materials.append(material)
    return obj


def add_barrier(name, location, collection, material, rot_z=0.0, length=6.0):
    return add_box(name, location, (length, 0.4, 1.0), collection, material, rot_z=rot_z)


def add_sign(name, location, collection, material):
    return add_box(name, location, (1.2, 0.15, 2.2), collection, material)


def add_rock(name, location, collection, material, radius=1.6):
    bpy.ops.mesh.primitive_ico_sphere_add(radius=radius, subdivisions=1, location=location)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = (1.0, random.uniform(0.7, 1.1), random.uniform(0.6, 0.9))
    relink(obj, collection)
    if material is not None:
        obj.data.materials.append(material)
    return obj


# ============================================================
# BUILD: TERRAIN
# ============================================================
def build_terrain(col, mat):
    for idx in range(3):
        make_rect_ring(f"{PREFIX}Bench_Level_{idx + 1}", OUTER[idx], INNER[idx],
                        Z_LEVELS[idx], col, mat["rock"])
    for idx in range(2):
        half = INNER[idx]  # == OUTER[idx + 1]
        make_wall(f"{PREFIX}Bench_Wall_{idx + 1}_to_{idx + 2}", half,
                   Z_LEVELS[idx], Z_LEVELS[idx + 1], col, mat["rock"])
    make_floor(f"{PREFIX}Mine_Floor", FLOOR_HALF, FLOOR_Z, col, mat["rock"])


# ============================================================
# BUILD: ROADS, RAMPS & TURNING AREAS
# Route stays on the +Y ("north") side of the pit the whole way down,
# so every segment sits on a solid ring strip - never over the hole.
# ============================================================
def build_roads(col, mat):
    band0 = (INNER[0], OUTER[0])   # (80, 95)
    band1 = (INNER[1], OUTER[1])   # (48, 80)
    band2 = (INNER[2], OUTER[2])   # (15, 48)

    apron_y = (band0[0] + band0[1]) / 2.0          # 87.5
    apron_h = (band0[1] - band0[0]) - 2.0           # 13 (2m margin inside the ring)

    # --- Spawn apron / turning area on the top bench ---
    add_flat_road(f"{PREFIX}Road_Spawn_Apron", 0.0, apron_y, Z_LEVELS[0],
                   170.0, apron_h, col, mat["road"])
    add_flat_road(f"{PREFIX}Turning_Area_Spawn", 0.0, apron_y, Z_LEVELS[0],
                   40.0, apron_h, col, mat["road"])

    # --- Apron -> Ramp 1 entrance (short link, still inside ring0 band) ---
    ramp1_y_start = band0[0]  # 80
    add_flat_road(f"{PREFIX}Road_Approach_Ramp01", 0.0, (apron_y - apron_h / 2 + ramp1_y_start) / 2.0,
                   Z_LEVELS[0], ROAD_WIDTH, abs((apron_y - apron_h / 2) - ramp1_y_start), col, mat["road"])

    # --- Ramp 1: Bench 1 -> Bench 2 ---
    ramp1_y_end = ramp1_y_start - RAMP_RUN_1
    add_ramp(f"{PREFIX}Ramp_01_Bench1_to_Bench2", ramp1_y_start, Z_LEVELS[0],
              ramp1_y_end, Z_LEVELS[1], ROAD_WIDTH, col, mat["road"])

    # --- Short connector on Bench 2 between ramp 1 exit and ramp 2 entrance ---
    ramp2_y_start = band1[0]  # 48
    add_flat_road(f"{PREFIX}Road_Level2_Connector", 0.0, (ramp1_y_end + ramp2_y_start) / 2.0,
                   Z_LEVELS[1], ROAD_WIDTH, abs(ramp1_y_end - ramp2_y_start), col, mat["road"])

    # --- Ramp 2: Bench 2 -> Bench 3 / Mine Floor ---
    ramp2_y_end = ramp2_y_start - RAMP_RUN_2
    add_ramp(f"{PREFIX}Ramp_02_Bench2_to_Floor", ramp2_y_start, Z_LEVELS[1],
              ramp2_y_end, FLOOR_Z, ROAD_WIDTH, col, mat["road"])

    # --- Bench 3 / floor: ramp 2 exit -> Loading Zone (all coplanar, flat) ---
    loading_y = 0.0
    add_flat_road(f"{PREFIX}Road_Floor_to_Loading", 0.0, (ramp2_y_end + loading_y) / 2.0,
                   FLOOR_Z, ROAD_WIDTH, abs(ramp2_y_end - loading_y), col, mat["road"])

    # --- Turning area at the loading zone (mine floor) ---
    add_flat_road(f"{PREFIX}Turning_Area_Loading", 0.0, loading_y, FLOOR_Z,
                   26.0, 26.0, col, mat["road"])

    return {
        "apron_y": apron_y, "apron_h": apron_h,
        "ramp1_y_start": ramp1_y_start, "ramp1_y_end": ramp1_y_end,
        "ramp2_y_start": ramp2_y_start, "ramp2_y_end": ramp2_y_end,
        "loading_y": loading_y,
    }


# ============================================================
# BUILD: NAVIGATION MARKERS
# ============================================================
def build_markers(col, mat, route):
    m = mat["marker"]
    apron_y = route["apron_y"]

    add_cone_marker(f"{PREFIX}Truck_Spawn_01", (-60.0, apron_y, Z_LEVELS[0] + 2.0), col, m)
    add_cone_marker(f"{PREFIX}Truck_Spawn_02", (60.0, apron_y, Z_LEVELS[0] + 2.0), col, m)
    add_cone_marker(f"{PREFIX}Dumping_Zone", (30.0, apron_y, Z_LEVELS[0] + 2.0), col, m)
    add_cone_marker(f"{PREFIX}Loading_Zone", (0.0, route["loading_y"] - 8.0, FLOOR_Z + 2.0), col, m)

    add_cone_marker(f"{PREFIX}Waypoint_01", (0.0, apron_y, Z_LEVELS[0] + 2.0), col, m, radius=1.4, depth=2.6)
    add_cone_marker(f"{PREFIX}Waypoint_02", (0.0, route["ramp1_y_start"], Z_LEVELS[0] + 2.0), col, m, radius=1.4, depth=2.6)
    add_cone_marker(f"{PREFIX}Waypoint_03", (0.0, route["ramp2_y_start"], Z_LEVELS[1] + 2.0), col, m, radius=1.4, depth=2.6)
    add_cone_marker(f"{PREFIX}Waypoint_04", (0.0, route["ramp2_y_end"], FLOOR_Z + 2.0), col, m, radius=1.4, depth=2.6)
    add_cone_marker(f"{PREFIX}Waypoint_05", (0.0, route["loading_y"], FLOOR_Z + 2.0), col, m, radius=1.4, depth=2.6)


# ============================================================
# BUILD: SAFETY / ENVIRONMENT PROPS  (small number only)
# ============================================================
def build_safety_objects(col, mat, route):
    s = mat["safety"]

    # Barriers flanking the two ramp entrances (4 total), sitting on flat
    # ground just before each slope starts.
    add_barrier(f"{PREFIX}Barrier_Ramp01_Left", (-ROAD_WIDTH / 2 - 1.5, route["ramp1_y_start"] + 2.0, Z_LEVELS[0] + 0.5),
                col, s, rot_z=math.radians(90))
    add_barrier(f"{PREFIX}Barrier_Ramp01_Right", (ROAD_WIDTH / 2 + 1.5, route["ramp1_y_start"] + 2.0, Z_LEVELS[0] + 0.5),
                col, s, rot_z=math.radians(90))
    add_barrier(f"{PREFIX}Barrier_Ramp02_Left", (-ROAD_WIDTH / 2 - 1.5, route["ramp2_y_start"] + 2.0, Z_LEVELS[1] + 0.5),
                col, s, rot_z=math.radians(90))
    add_barrier(f"{PREFIX}Barrier_Ramp02_Right", (ROAD_WIDTH / 2 + 1.5, route["ramp2_y_start"] + 2.0, Z_LEVELS[1] + 0.5),
                col, s, rot_z=math.radians(90))

    # Warning signs near each ramp entrance and the loading zone (3 total)
    add_sign(f"{PREFIX}Warning_Sign_Ramp01", (12.0, route["ramp1_y_start"] + 3.0, Z_LEVELS[0] + 1.1), col, s)
    add_sign(f"{PREFIX}Warning_Sign_Ramp02", (12.0, route["ramp2_y_start"] + 3.0, Z_LEVELS[1] + 1.1), col, s)
    add_sign(f"{PREFIX}Warning_Sign_Loading", (10.0, route["loading_y"] - 4.0, FLOOR_Z + 1.1), col, s)

    # A handful of rock obstacles scattered on unused parts of the benches
    # (south/east/west strips + mine floor) - 4 total, never on the road.
    rock_spots = [
        (55.0, -(OUTER[0] + INNER[0]) / 2.0, Z_LEVELS[0]),   # ring0 south strip
        (-(OUTER[0] + INNER[0]) / 2.0, 10.0, Z_LEVELS[0]),   # ring0 west strip
        ((OUTER[1] + INNER[1]) / 2.0, 15.0, Z_LEVELS[1]),    # ring1 east strip
        (10.0, -10.0, FLOOR_Z),                               # mine floor
    ]
    for idx, (rx, ry, rz) in enumerate(rock_spots, start=1):
        radius = random.uniform(1.2, 2.0)
        add_rock(f"{PREFIX}Rock_{idx:02d}", (rx, ry, rz + radius * 0.5), col, s, radius=radius)


# ============================================================
# MAIN
# ============================================================
def main():
    clear_existing()
    root, subs = setup_collections()
    mat = build_materials()

    build_terrain(subs["TERRAIN"], mat)
    route = build_roads(subs["ROADS"], mat)
    build_markers(subs["MARKERS"], mat, route)
    build_safety_objects(subs["SAFETY"], mat, route)

    counts = {name: len(col.objects) for name, col in subs.items()}
    total = sum(counts.values())

    print("=" * 60)
    print("SIH OPEN-CAST MINE ENVIRONMENT GENERATED")
    print("=" * 60)
    print(f"Collection root      : {COLLECTION_NAMES['ROOT']}")
    print(f"Terrain objects      : {counts['TERRAIN']} (3 bench rings, 2 walls, 1 floor)")
    print(f"Road/ramp objects    : {counts['ROADS']} (2 ramps + flat roads + turning pads)")
    print(f"Marker objects       : {counts['MARKERS']} (2 spawns, loading, dumping, 5 waypoints)")
    print(f"Safety/prop objects  : {counts['SAFETY']} (4 barriers, 3 signs, 4 rocks)")
    print(f"Total objects        : {total}")
    print(f"Mine footprint       : {2 * OUTER[0]:.0f}m x {2 * OUTER[0]:.0f}m (within 200m x 200m limit)")
    print(f"Total depth          : {abs(FLOOR_Z):.0f}m across 2 ramps / 3 bench levels")
    print("=" * 60)


if __name__ == "__main__":
    main()
