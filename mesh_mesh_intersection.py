from itertools import chain

import bmesh
import bpy
from bmesh.types import BMesh, BMEdge, BMFace
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.geometry import (
    delaunay_2d_cdt,
    intersect_line_plane,
    intersect_point_tri,
)


def bm_edge_vec(e: BMEdge) -> Vector:
    return e.verts[1].co - e.verts[0].co


def is_point_on_bm_edge(p: Vector, e: BMEdge):
    edge_length = bm_edge_vec(e).length
    return (p - e.verts[0].co).length <= edge_length


def is_point_on_bm_face_tri(p: Vector, f: BMFace):
    assert len(f.verts) == 3
    return bool(intersect_point_tri(p, *(v.co for v in f.verts)))


def intersect_bm_edge_face_tri(e: BMEdge, f: BMFace):
    assert len(f.verts) == 3
    return intersect_line_plane(e.verts[0].co, e.verts[1].co, f.verts[0].co, f.normal)


def bm_tri_tri_intersect(bm_face1, bm_face2):
    points = []
    for e in bm_face1.edges:
        p = intersect_bm_edge_face_tri(e, bm_face2)
        if p:
            if is_point_on_bm_edge(p, e) and is_point_on_bm_face_tri(p, bm_face2):
                points.append(p)
    return points


def bm_intersect(bm1: BMesh, bm2: BMesh):
    bmesh.ops.triangulate(bm1, faces=bm1.faces)
    bmesh.ops.triangulate(bm2, faces=bm2.faces)
    bm1.faces.ensure_lookup_table()
    bm1.normal_update()
    bm2.faces.ensure_lookup_table()
    bm2.normal_update()

    bvh1 = BVHTree.FromBMesh(bm1)
    bvh2 = BVHTree.FromBMesh(bm2)

    bm_out = bmesh.new()

    # NOTE: we need to initlize the dict for all faces in order for them to be present in the final mesh,
    # and not just triangles that have intersections
    intersection_points_map = {f: [] for f in chain(bm1.faces, bm2.faces)}

    for i1, i2 in bvh1.overlap(bvh2):
        bm_face1 = bm1.faces[i1]
        bm_face2 = bm2.faces[i2]

        points = bm_tri_tri_intersect(bm_face1, bm_face2)
        intersection_points_map[bm_face1].extend(points)
        intersection_points_map[bm_face2].extend(points)

        points = bm_tri_tri_intersect(bm_face2, bm_face1)
        intersection_points_map[bm_face1].extend(points)
        intersection_points_map[bm_face2].extend(points)

    for tri, points in intersection_points_map.items():
        points = list(set((p[:] for p in points)))
        cdt_points_input_3d = [v.co for v in tri.verts] + points
        bm_verts = [bm_out.verts.new(p) for p in cdt_points_input_3d]

        rot_mat = tri.normal.rotation_difference((0, 0, 1)).to_matrix()
        cdt_points_input_2d = [
            (rot_mat @ Vector(p)).to_2d() for p in cdt_points_input_3d
        ]
        # NOTE: input points to delaunay_2d_cdt must be unique
        # otherwise there will be errors in the result
        (
            delaunay_verts_co,
            delaunay_edges,
            delaunay_faces,
            delaunay_orig_verts,
            delaunay_orig_edges,
            delaunay_orig_faces,
        ) = delaunay_2d_cdt(cdt_points_input_2d, [], [], 0, 0.00001, False)
        for f in delaunay_faces:
            bm_out.faces.new([bm_verts[i] for i in f])

    bmesh.ops.remove_doubles(bm_out, verts=bm_out.verts, dist=0.00001)
    bmesh.ops.dissolve_degenerate(bm_out, dist=0.00001, edges=bm_out.edges)

    return bm_out


def obj_obj_intersect(obj1, obj2):
    """Intersects two mesh objects and creates a new object from the result"""
    dg = bpy.context.evaluated_depsgraph_get()
    bm1 = bmesh.new()
    bm1.from_object(obj1, dg)
    bm1.transform(obj1.matrix_world)

    bm2 = bmesh.new()
    bm2.from_object(obj2, dg)
    bm2.transform(obj2.matrix_world)

    bm_out = bm_intersect(bm1, bm2)
    mesh_out = bpy.data.meshes.new("")
    bm_out.to_mesh(mesh_out)
    obj_out = bpy.data.objects.new("", mesh_out)
    bpy.context.scene.collection.objects.link(obj_out)
    bm_out.free()
    bm1.free()
    bm2.free()
