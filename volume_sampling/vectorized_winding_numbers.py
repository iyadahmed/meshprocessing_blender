import math
from contextlib import contextmanager
from timeit import default_timer

import bpy
import numpy as np
import numpy.typing as npt

# Vectorized to speed up query for one point (mesh triangles are vectorized)

# References
# https://github.com/marmakoide/inside-3d-mesh/blob/master/is_inside_mesh.py
# https://en.wikipedia.org/wiki/Solid_angle#Tetrahedron
# https://igl.ethz.ch/projects/winding-number/robust-inside-outside-segmentation-using-generalized-winding-numbers-siggraph-2013-compressed-jacobson-et-al.pdf


@contextmanager
def scoped_timer(msg: str):
    t0 = default_timer()
    yield
    t1 = default_timer()
    print(f"{msg} finished in {t1 - t0:.2f} seconds.")


def calc_winding_number_vectorized(query_point: npt.DTypeLike, tris: npt.DTypeLike):
    """Origin is expected to be a 3 component vector (1D NumPy array),
    tris is exepcted to be a 2D NumPy array of vertex location of each triangle with shape (num_tris, 3)"""

    assert query_point.shape == (3,)
    assert len(tris.shape) == 2
    assert tris.shape[1] == 3
    assert (tris.shape[0] % 3) == 0

    tris_shifted = tris - query_point

    # Compute norm once so that it is vectorized by NumPy
    norm = np.linalg.norm(tris_shifted, axis=1, ord=2)

    v1_norm = norm[0::3]
    v2_norm = norm[1::3]
    v3_norm = norm[2::3]

    v1 = tris_shifted[0::3]
    v2 = tris_shifted[1::3]
    v3 = tris_shifted[2::3]

    denominator = (
        v1_norm * v2_norm * v3_norm
        + (v1 * v2).sum(axis=1) * v3_norm
        + (v1 * v3).sum(axis=1) * v2_norm
        + (v2 * v3).sum(axis=1) * v1_norm
    )
    numerator = np.linalg.det(tris_shifted.reshape(-1, 3, 3))

    return np.arctan2(numerator, denominator).sum()


def is_inside(query_point: npt.DTypeLike, tris: npt.DTypeLike):
    """Checks if point is inside mesh,
    assumes mesh already has consistent normals with positive volume everywhere inside"""
    return calc_winding_number_vectorized(query_point, tris) >= (2.0 * np.pi)


if __name__ == "__main__":
    obj = bpy.context.object
    assert obj.type == "MESH"

    # Generate poitns inside bounding box
    min_bb = np.min(obj.bound_box, axis=0)
    max_bb = np.max(obj.bound_box, axis=0)
    rng = np.random.default_rng()
    query_points = rng.uniform(low=min_bb, high=max_bb, size=(100000, 3))

    mesh: bpy.types.Mesh = obj.data
    mesh.calc_loop_triangles()
    tris = np.array(
        [mesh.vertices[i].co for tri in mesh.loop_triangles for i in tri.vertices]
    )
    print(f"Number of mesh triangles = {len(mesh.loop_triangles)}")
    with scoped_timer(
        f"Filtering {len(query_points)} points using Generalized Winding Numbers"
    ):
        filtered_points = [p for p in query_points if is_inside(p, tris)]

    # Create point cloud
    points_mesh = bpy.data.meshes.new("")
    points_mesh.from_pydata(filtered_points, [], [])
    points_obj = bpy.data.objects.new("", points_mesh)
    bpy.context.scene.collection.objects.link(points_obj)
