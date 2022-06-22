from typing import List

import bmesh
import bpy
from mathutils.bvhtree import BVHTree


def _bm_grow_tagged(seed_face: bmesh.types.BMFace):
    """Flood fill untagged linked faces starting from a faces, tags and returns them"""
    faces = [seed_face]
    e: bmesh.types.BMEdge
    lf: bmesh.types.BMFace
    for f in faces:
        for e in f.edges:
            for lf in e.link_faces:
                if lf != f:
                    if not lf.tag:
                        faces.append(lf)
                        lf.tag = True
    return faces


def get_loose_parts(bm: bmesh.types.BMesh):
    loose_parts: List[List[bmesh.types.BMFace]] = []
    for f in bm.faces:
        f.tag = False
    for f in bm.faces:
        if not f.tag:
            # We could yield instead
            # but tag could be modifed after yield
            # so better to store results in a list
            loose_parts.append(_bm_grow_tagged(f))
    return loose_parts


obj = bpy.context.object

dg = bpy.context.evaluated_depsgraph_get()
bm = bmesh.new()
bm.from_object(obj, dg)
bm.transform(obj.matrix_world)

# non_manifold_verts = [v for v in bm.verts if not v.is_manifold]
# for v in non_manifold_verts:
#     bmesh.utils.vert_separate(v, v.link_edges)

# bmesh.ops.recalc_face_normals(bm, faces=bm.faces)


# new_mesh = bpy.data.meshes.new("")
# bm.to_mesh(new_mesh)
# new_obj = bpy.data.objects.new("", new_mesh)
# bpy.context.scene.collection.objects.link(new_obj)

# bvh = BVHTree.FromBMesh(bm)

print(len(get_loose_parts(bm)))

bm.free()
