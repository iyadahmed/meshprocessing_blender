from math import atan
import math

import bpy
import numpy as np
from mathutils import Vector


# https://en.wikipedia.org/wiki/Solid_angle#Tetrahedron
def tet_solid_angle(O: Vector, A: Vector, B: Vector, C: Vector):
    av: Vector = A - O
    bv: Vector = B - O
    cv: Vector = C - O

    al = av.length
    bl = bv.length
    cl = cv.length

    numerator = av.dot(bv.cross(cv))
    denominator = al * bl * cl + av.dot(bv) * cl + av.dot(cv) * bl + bv.dot(cv) * al
    return atan(2 * (numerator / denominator))


def calc_winding_number(point: Vector, mesh: bpy.types.Mesh):
    w = 0.0
    mesh.calc_loop_triangles()
    tri: bpy.types.MeshLoopTriangle
    for tri in mesh.loop_triangles:
        a, b, c = (mesh.vertices[i].co for i in tri.vertices)
        w += tet_solid_angle(point, a, b, c) / (4 * math.pi)
    return w


def is_inside(point: Vector, mesh: bpy.types.Mesh, isovalue=0.5):
    """Checks if point is inside mesh,
    assumes mesh already has consistent normals with positive volume everywhere inside"""
    return calc_winding_number(point, mesh) > isovalue


if __name__ == "__main__":
    obj = bpy.context.object
    assert obj.type == "MESH"

    # Generate poitns inside bounding box
    min_bb = np.min(obj.bound_box, axis=0)
    max_bb = np.max(obj.bound_box, axis=0)
    rng = np.random.default_rng()
    points = rng.uniform(low=min_bb, high=max_bb, size=(1000, 3))

    # Filter points
    inside_points = [p for p in points if is_inside(Vector(p), obj.data)]

    # Create point cloud
    points_mesh = bpy.data.meshes.new("")
    points_mesh.from_pydata(inside_points, [], [])
    points_obj = bpy.data.objects.new("", points_mesh)
    bpy.context.scene.collection.objects.link(points_obj)
