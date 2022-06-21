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


p = Vector()
tri: bpy.types.MeshLoopTriangle

w = 0
for tri in mesh.loop_triangles:
    a, b, c = (mesh.vertices[i].co for i in tri.vertices)
    w += solid_angle(p, a, b, c)

print(w)
