import math

import bmesh
import bpy
import numpy as np
from mathutils import Matrix

from .scoped_timer import scoped_timer

DEBUG = False

CUBE_FACE_INDICES = (
    (0, 1, 3, 2),
    (2, 3, 7, 6),
    (6, 7, 5, 4),
    (4, 5, 1, 0),
    (2, 6, 4, 0),
    (7, 3, 1, 5),
)


def gen_cube_verts():
    for x in range(-1, 2, 2):
        for y in range(-1, 2, 2):
            for z in range(-1, 2, 2):
                yield x, y, z


def rotating_calipers(hull_points: np.ndarray, bases):
    min_bb_basis = None
    min_bb_min = None
    min_bb_max = None
    min_vol = math.inf
    for basis in bases:
        rot_points = hull_points.dot(np.linalg.inv(basis))
        # Should be equivalent to: rot_points = hull_points.dot(np.linalg.inv(np.transpose(basis)).T)

        bb_min = rot_points.min(axis=0)
        bb_max = rot_points.max(axis=0)
        volume = (bb_max - bb_min).prod()
        if volume < min_vol:
            min_bb_basis = basis
            min_vol = volume

            min_bb_min = bb_min
            min_bb_max = bb_max

    return np.array(min_bb_basis), min_bb_max, min_bb_min


def obj_rotating_calipers(obj):
    bm = bmesh.new()
    dg = bpy.context.evaluated_depsgraph_get()
    bm.from_object(obj, dg)

    with scoped_timer("Calculating Convex-Hull"):
        chull_out = bmesh.ops.convex_hull(bm, input=bm.verts, use_existing_faces=False)

    chull_geom = chull_out["geom"]
    chull_points = np.array(
        [bmelem.co for bmelem in chull_geom if isinstance(bmelem, bmesh.types.BMVert)]
    )

    # Create object from Convex-Hull (for debugging)
    if DEBUG:
        with scoped_timer("Deleting non Convex-Hull edges and faces"):
            for face in set(bm.faces) - set(chull_geom):
                bm.faces.remove(face)
            for edge in set(bm.edges) - set(chull_geom):
                bm.edges.remove(edge)

        chull_mesh = bpy.data.meshes.new(obj.name + "_convex_hull")
        bm.to_mesh(chull_mesh)
        chull_obj = bpy.data.objects.new(chull_mesh.name, chull_mesh)
        chull_obj.matrix_world = obj.matrix_world
        bpy.context.scene.collection.objects.link(chull_obj)

    bases = []

    with scoped_timer("Building list of bases"):
        for elem in chull_geom:
            if not isinstance(elem, bmesh.types.BMFace):
                continue
            if len(elem.verts) != 3:
                continue

            face_normal = elem.normal
            if np.allclose(face_normal, 0, atol=0.00001):
                continue

            for e in elem.edges:
                v0, v1 = e.verts
                edge_vec = (v0.co - v1.co).normalized()
                co_tangent = face_normal.cross(edge_vec)
                basis = (edge_vec, co_tangent, face_normal)
                bases.append(basis)

    with scoped_timer("Finding minimum volume basis"):
        bb_basis, bb_max, bb_min = rotating_calipers(chull_points, bases)

    bm.free()

    bb_basis_mat = bb_basis.T

    bb_dim = bb_max - bb_min
    bb_center = (bb_max + bb_min) / 2

    mat = (
        Matrix.Translation(bb_center.dot(bb_basis))
        @ Matrix(bb_basis_mat).to_4x4()
        @ Matrix(np.identity(3) * bb_dim / 2).to_4x4()
    )

    bb_mesh = bpy.data.meshes.new(obj.name + "_minimum_bounding_box")
    bb_mesh.from_pydata(
        vertices=list(gen_cube_verts()), edges=[], faces=CUBE_FACE_INDICES
    )
    bb_mesh.validate()
    bb_mesh.transform(mat)
    bb_mesh.update()
    bb_obj = bpy.data.objects.new(bb_mesh.name, bb_mesh)
    bb_obj.display_type = "WIRE"
    bb_obj.matrix_world = obj.matrix_world
    bpy.context.scene.collection.objects.link(bb_obj)


obj_rotating_calipers(bpy.context.object)
