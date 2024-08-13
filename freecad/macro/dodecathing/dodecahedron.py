# import FreeCAD as App
from FreeCAD import Vector

# , Units, Console, Rotation, Matrix, Sketcher
import Part
from itertools import product, combinations
import numpy as np


def make_dodecahedron_vertices(radius: float = 1):
    """Create the set of 20 vertices of a dodecahedron that is inscribed in a sphere of the given radius

    Args:
        radius (float): the radius of the sphere that the dodecahedron is inscribed in

    Return:
        an array of vertices
    """
    size = radius / (3**0.5)

    phi = (1 + 5**0.5) / 2
    inverse_phi = 1 / phi
    p2 = list(product([1, -1], repeat=2))
    pts = np.zeros((20, 3))

    pts[0:8] = np.array(list(product([1, -1], repeat=3))) * size
    pts[8:12] = np.array([(0, y, z) for y, z in p2]) * np.array([0, phi, inverse_phi]) * size
    pts[12:16] = np.array([(x, 0, z) for x, z in p2]) * np.array([inverse_phi, 0, phi]) * size
    pts[16:20] = np.array([(x, y, 0) for x, y in p2]) * np.array([phi, inverse_phi, 0]) * size
    return pts


def make_dodecahedron_edges():
    """Create the set of edges of the dodecahedron"""
    pts = make_dodecahedron_vertices()
    edges = set()
    for n, p in enumerate(pts):
        dists = sorted([(np.linalg.norm(p - p2), d) for d, p2 in enumerate(pts)])
        for _, y in dists[1:4]:
            if not (y, n) in edges:
                edges.add((n, y))
    return sorted(list(edges))


"""Table to convert the output of make_dodecahedron_vertices into a dodecahedron.

There's probably a very clever way to generate this table
"""
# fmt: off
dodecahedron_faces = [
          [ 0, 8, 9, 1,16],
          [ 0,12,14, 4, 8],
          [ 0,16,17, 2,12],
          [ 1, 9, 5,15,13],
          [ 1,13, 3,17,16],
          [ 2,17, 3,11,10],
          [ 2,10, 6,14,12],
          [ 3,13,15, 7,11],
          [ 4,14, 6,19,18],
          [ 4,18, 5, 9, 8],
          [ 5,18,19, 7,15],
          [ 6,10,11, 7,19]]
# fmt: on


def get_inscribed_sphere_radius(circumscribed_radius: float = 1):
    """Given a dodecahedron inscribed in a sphere of radius circumscribed_radius return the radius the sphere inscribed
    in the dodecahedron
    """
    return circumscribed_radius * 0.794654472291766


def get_circumscribed_sphere_radius(inscribed_radius: float = 1):
    """Given a dodecahedron circumscribed around a sphere of radius inscribed_radius return the radius of the sphere
    circumscribed about the dodecahedron
    """
    return inscribed_radius / 0.794654472291766


def make_dodecahedron_faces(radius: float = 1, is_circumscribed: bool = False):
    """Make the faces of a dodecahedron

    Args:
        is_circumscribed (bool): if True the sphere's radius is circumscribed about the dodecahedron
            if False the sphere is inscribed in the dodecahedron
    """

    # the vertex generation uses an circumscribed sphere. Convert if needed
    dodecahedron_radius = radius if is_circumscribed else get_circumscribed_sphere_radius(radius)

    pts = [Vector(x, y, z) for x, y, z in make_dodecahedron_vertices(dodecahedron_radius)]
    faces = []
    for f in dodecahedron_faces:
        # pass True to close the polygon. Why isn't that the default behavior?
        faces.append(Part.Face(Part.makePolygon([pts[n] for n in f], True)))
    return faces


def get_opposing_faces(dodecahedron_faces):
    """Given the output of make_dodecahedron_faces produce the list of faces that are opposing but not directly
    opposite each face.

    i.e. in a dodecahedron face 0 has 5 adjacent faces, 1 opposite face, and 5 faces that are facing away from face 0

    Returns: a list of pairs of opposing faces. Each pair of elements occurs once
        so if (0, 5) is in the list then (5, 0) will not also be in the list
    """
    found = set()
    for m, n in combinations(range(len(dodecahedron_faces)), 2):
        normDot = dodecahedron_faces[m].normalAt(0, 0).dot(dodecahedron_faces[n].normalAt(0, 0))
        # want faces that are facing in an opposing direction but not directly opposite
        if normDot < 0 and normDot > -1:
            found.add((m, n))
    return sorted(list(found))
