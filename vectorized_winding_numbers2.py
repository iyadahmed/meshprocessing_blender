from contextlib import contextmanager
from timeit import default_timer

import bpy
import numpy as np

# This is a vectorized version for many points
# This currently might have memory/perf issues with large number of points (e.g. 1000 00)


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


if __name__ == "__main__":
    obj = bpy.context.object
    assert obj.type == "MESH"

    # Generate poitns inside bounding box
    min_bb = np.min(obj.bound_box, axis=0)
    max_bb = np.max(obj.bound_box, axis=0)
    rng = np.random.default_rng(seed=1234)

    NUM_QUERY_POINTS = 1000

    query_points = rng.uniform(low=min_bb, high=max_bb, size=(NUM_QUERY_POINTS, 3))

    mesh: bpy.types.Mesh = obj.data
    mesh.calc_loop_triangles()
    tris = np.array(
        [mesh.vertices[i].co for tri in mesh.loop_triangles for i in tri.vertices]
    )

    tris_shifted = tris - query_points[:, np.newaxis]

    with scoped_timer("Calculating norm"):
        norm = np.linalg.norm(tris_shifted, axis=2, ord=2)

    v1_norm = norm[:, 0::3]
    v2_norm = norm[:, 1::3]
    v3_norm = norm[:, 2::3]

    v1 = tris_shifted[:, 0::3]
    v2 = tris_shifted[:, 1::3]
    v3 = tris_shifted[:, 2::3]

    with scoped_timer("Calculating denominator"):
        denominator = (
            v1_norm * v2_norm * v3_norm
            + (v1 * v2).sum(axis=2) * v3_norm
            + (v1 * v3).sum(axis=2) * v2_norm
            + (v2 * v3).sum(axis=2) * v1_norm
        )

    with scoped_timer("Reshaping"):
        tris_reshaped = tris_shifted.reshape(NUM_QUERY_POINTS, -1, 3, 3)

    with scoped_timer("Calculating determinant"):
        numerator = np.linalg.det(tris_reshaped)

    with scoped_timer("Calculating arctangent"):
        w = np.arctan2(numerator, denominator).sum(axis=1)

    with scoped_timer("Filtering points"):
        is_inside = w >= (2.0 * np.pi)
        filtered_points = query_points[is_inside]

    # Create point cloud
    with scoped_timer("Creating point cloud"):
        points_mesh = bpy.data.meshes.new("")
        points_mesh.from_pydata(filtered_points, [], [])
        points_obj = bpy.data.objects.new("", points_mesh)
        bpy.context.scene.collection.objects.link(points_obj)
