############################
# NOT WORKING Yet
############################


import bpy
import numpy as np
from mathutils import Vector
from mathutils.kdtree import KDTree

NUM_NEAREST_TRIANGLES_PER_POINT = 4

obj = bpy.context.object

min_bb = np.min(obj.bound_box, axis=0)
max_bb = np.max(obj.bound_box, axis=0)


rng = np.random.default_rng()
# Generate poitns inside bounding box
points = rng.uniform(low=min_bb, high=max_bb, size=(100000, 3))

kd = KDTree(len(obj.data.polygons))
for i, p in enumerate(obj.data.polygons):
    kd.insert(p.center, i)
kd.balance()


inside_points = []
for p in points:
    a = 0
    for co, index, dist in kd.find_n(p, NUM_NEAREST_TRIANGLES_PER_POINT):
        n = obj.data.polygons[index].normal
        a += (co - Vector(p)).dot(n)
    if a > (NUM_NEAREST_TRIANGLES_PER_POINT // 2):
        inside_points.append(p)


points_mesh = bpy.data.meshes.new("")
points_mesh.from_pydata(inside_points, [], [])
points_obj = bpy.data.objects.new("", points_mesh)
bpy.context.scene.collection.objects.link(points_obj)
