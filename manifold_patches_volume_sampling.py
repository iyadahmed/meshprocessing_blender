from typing import List

import bmesh
import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree


def _bm_grow_untagged_manifold(seed_face: bmesh.types.BMFace):
    """Flood fill untagged and linked faces that are connected via manifold edges starting from a seed face, tags those faces and returns them"""
    faces = [seed_face]
    e: bmesh.types.BMEdge
    lf: bmesh.types.BMFace
    for f in faces:
        for e in f.edges:
            if e.is_manifold:
                for lf in e.link_faces:
                    if lf != f:
                        if not lf.tag:
                            faces.append(lf)
                            lf.tag = True
    return faces


def get_manifold_patches(bm: bmesh.types.BMesh):
    manifold_patches: List[List[bmesh.types.BMFace]] = []
    for f in bm.faces:
        f.tag = False
    for f in bm.faces:
        if not f.tag:
            # We could yield instead
            # but tag could be modifed after yield
            # so better to store results in a list
            manifold_patches.append(_bm_grow_untagged_manifold(f))
    return manifold_patches


if __name__ == "__main__":
    obj = bpy.context.object
    assert obj.type == "MESH"

    dg = bpy.context.evaluated_depsgraph_get()
    bm = bmesh.new()
    bm.from_object(obj, dg)
    # bm.transform(obj.matrix_world)

    # non_manifold_verts = [v for v in bm.verts if not v.is_manifold]
    # for v in non_manifold_verts:
    #     bmesh.utils.vert_separate(v, v.link_edges)

    # new_mesh = bpy.data.meshes.new("")
    # bm.to_mesh(new_mesh)
    # new_obj = bpy.data.objects.new("", new_mesh)
    # bpy.context.scene.collection.objects.link(new_obj)

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)

    trees: List[BVHTree] = []
    bm.verts.ensure_lookup_table()
    verts = [v.co.copy() for v in bm.verts]

    for faces in get_manifold_patches(bm):
        polys = [[v.index for v in f.verts] for f in faces]
        bvh = BVHTree.FromPolygons(verts, polys)
        trees.append(bvh)

    # f: bmesh.types.BMFace
    # for f in bm.faces:
    #     verts = [v.co for v in f.verts]
    #     v1 = verts[1] - verts[0]
    #     v2 = verts[2] - verts[0]
    #     n = v1.cross(v2)
    #     assert n.dot(f.normal) > 0

    bm.free()

    def is_inside(p: Vector):
        for t in trees:
            location, normal, polygon_index, distance = t.find_nearest(p)
            if location:
                if (location - p).dot(normal) > 0:
                    return True
        return False

    # Generate points inside bounding box
    min_bb = np.min(obj.bound_box, axis=0)
    max_bb = np.max(obj.bound_box, axis=0)
    rng = np.random.default_rng()
    points = rng.uniform(low=min_bb, high=max_bb, size=(1000, 3))

    # Filter points
    inside_points = [p for p in points if is_inside(Vector(p))]

    # Create point cloud
    points_mesh = bpy.data.meshes.new("")
    points_mesh.from_pydata(inside_points, [], [])
    points_obj = bpy.data.objects.new("", points_mesh)
    bpy.context.scene.collection.objects.link(points_obj)
