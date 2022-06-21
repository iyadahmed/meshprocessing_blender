import bpy
from mathutils import Vector
from math import atan


# https://en.wikipedia.org/wiki/Solid_angle#Tetrahedron
def solid_angle(O: Vector, A: Vector, B: Vector, C: Vector):
    av: Vector = A - O
    bv: Vector = B - O
    cv: Vector = C - O

    al = av.length
    bl = bv.length
    cl = cv.length

    numerator = av.dot(bv.cross(cv))
    denominator = al * bl * cl + av.dot(bv) * cl + av.dot(cv) * bl + bv.dot(cv) * al
    return atan(2 * (numerator / denominator))


obj = bpy.context.object

mesh: bpy.types.Mesh = obj.data
mesh.calc_loop_triangles()

tri: bpy.types.MeshLoopTriangle
for tri in mesh.loop_triangles:
    a, b, c = (mesh.vertices[i].co for i in tri.vertices)
    print(a, b, c)
