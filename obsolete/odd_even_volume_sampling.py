import bmesh
import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

if __name__ == "__main__":
    obj = bpy.context.object
    assert obj.type == "MESH"

    dg = bpy.context.evaluated_depsgraph_get()
    bm = bmesh.new()
    bm.from_object(obj, dg)
    bm.transform(obj.matrix_world)
    bvh: BVHTree = BVHTree.FromBMesh(bm)
    bm.free()

    def is_inside(query_point: Vector):
        closest_point, *_ = bvh.find_nearest(query_point)
        if closest_point is None:
            return False
        result = True
        direction: Vector = (closest_point - query_point).normalized()
        hit_point = closest_point
        while True:
            hit_point, *_ = bvh.ray_cast(
                hit_point + 0.00001 * direction,
                direction,
            )
            if hit_point is None:
                break
            result = not result

        return result

    # Generate points inside bounding box
    min_bb = np.min(obj.bound_box, axis=0)
    max_bb = np.max(obj.bound_box, axis=0)
    rng = np.random.default_rng()
    query_points = rng.uniform(low=min_bb, high=max_bb, size=(1000, 3))

    filtered_points = [p for p in query_points if is_inside(Vector(p))]

    # Create point cloud
    points_mesh = bpy.data.meshes.new("")
    points_mesh.from_pydata(filtered_points, [], [])
    points_obj = bpy.data.objects.new("", points_mesh)
    bpy.context.scene.collection.objects.link(points_obj)
