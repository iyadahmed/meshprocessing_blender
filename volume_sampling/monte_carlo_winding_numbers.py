import math
from contextlib import contextmanager
from math import atan2
from timeit import default_timer

import bmesh
import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

RNG = np.random.default_rng()


def tet_solid_angle(O: Vector, A: Vector, B: Vector, C: Vector):
    av: Vector = A - O
    bv: Vector = B - O
    cv: Vector = C - O

    al = av.length
    bl = bv.length
    cl = cv.length

    numerator = av.dot(bv.cross(cv))
    denominator = al * bl * cl + av.dot(bv) * cl + av.dot(cv) * bl + bv.dot(cv) * al
    return atan2(numerator, denominator)


@contextmanager
def scoped_timer(msg: str):
    t0 = default_timer()
    yield
    t1 = default_timer()
    print(f"{msg} finished in {t1 - t0:.2f} seconds.")


# https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#UniformlySamplingaHemisphere
def uniform_sample_sphere(num_samples: int):
    u = RNG.uniform(0, 1, (num_samples, 2))
    z = 1 - 2 * u[:, 0]
    r = np.sqrt(1 - z * z)
    phi = 2 * np.pi * u[:, 1]
    return np.c_[r * np.cos(phi), r * np.sin(phi), z]


SPHERE_SAMPLES = uniform_sample_sphere(100)


if __name__ == "__main__":
    obj = bpy.context.object
    assert obj.type == "MESH"

    dg = bpy.context.evaluated_depsgraph_get()
    bm = bmesh.new()
    bm.from_object(obj, dg)
    bm.transform(obj.matrix_world)

    bmesh.ops.triangulate(bm, faces=bm.faces)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.faces.ensure_lookup_table()

    bvh: BVHTree = BVHTree.FromBMesh(bm)
    print(f"Number of mesh triangles = {len(bm.faces)}")

    def is_inside(query_point: Vector):
        w = 0
        for direction in SPHERE_SAMPLES:
            polygon_index = bvh.ray_cast(query_point, direction)[2]
            if polygon_index:
                face = bm.faces[polygon_index]
                a, b, c = (v.co for v in face.verts)
                w += tet_solid_angle(query_point, a, b, c)

        return w >= (2 * math.pi)

    # Generate points inside bounding box
    min_bb = np.min(obj.bound_box, axis=0)
    max_bb = np.max(obj.bound_box, axis=0)
    query_points = RNG.uniform(low=min_bb, high=max_bb, size=(50000, 3))

    with scoped_timer(
        f"Filtering {len(query_points)} points using Monte Carlo Winding Numbers Integration"
    ):
        filtered_points = [p for p in query_points if is_inside(Vector(p))]

    bm.free()

    # Create point cloud
    points_mesh = bpy.data.meshes.new("")
    points_mesh.from_pydata(filtered_points, [], [])
    points_obj = bpy.data.objects.new("", points_mesh)
    bpy.context.scene.collection.objects.link(points_obj)
