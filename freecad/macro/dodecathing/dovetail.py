import FreeCAD as App
from FreeCAD import Vector, Units, Console, Rotation, Matrix, Sketcher
import Part
import numpy as np
from itertools import product, combinations
from collections import defaultdict


def make_corner_fillet_points(v1: Vector, v2: Vector, v3: Vector, radius: float):
    """Given 3 points and a radius return 3 points that describe the arc which is tangent to lines
    (v1,v2) and (v2, v3) with the specified radius

    The function will fail if the points are too close to colinear

    Args:
        v1, v2, v3 (vector): the corner fillet points

        radius (float): the radius of the fillet
    """

    nearly_one = 1.0 - 100 * App.Base.Precision.confusion()
    v21 = Vector(v1 - v2).normalize()
    v23 = Vector(v3 - v2).normalize()

    dp = v21.dot(v23)
    if dp >= nearly_one or dp <= -nearly_one:
        raise ValueError("Points are colinear. Can't make a fillet")

    # the lines could be pointed in any orientation. Start by finding a vector perpendicular to both lines
    up_vec = Vector(v21.cross(v23)).normalize()
    v21Norm = Vector(up_vec.cross(v21)).normalize()
    v23Norm = Vector(up_vec.cross(v23)).normalize()
    if v21Norm.dot(v23) < 0:
        v21Norm = -v21Norm

    if v23Norm.dot(v21) < 0:
        v23Norm = -v23Norm

    offset21 = v2 + v21Norm * radius
    offset23 = v2 + v23Norm * radius
    l1 = Part.Line(offset21, offset21 + v21)
    l2 = Part.Line(offset23, offset23 + v23)
    center = l1.intersect(l2)[0]
    center = Vector(center.X, center.Y, center.Z)
    start_point = center - v21Norm * radius
    end_point = center - v23Norm * radius
    other_norm = Vector(v2 - center).normalize()
    middle_point = center + other_norm * radius

    return [start_point, middle_point, end_point]


def make_dovetail_shape(
    middle_point: Vector,
    middle_to_corner: Vector,
    middle_to_end: Vector,
    height: float,
    tail_angle_degrees: float = 10.0,
    inner_corner_radius: float = 1.0,
    tail_corner_radius: float = 0.0,
    up_vector: Vector = None,
    extra_base: float = 0.0,
):
    """Create a dovetail shaped feature. These features are used to join separate parts together.

    Args:
        middle_point (Vector): the point in the center of the dovetail at the point where the two features join.
            This will typically be a point on the edge of an object

        middle_to_corner (Vector): a vector that extends along the edge of the object between the middle_point
            and the narrow part of the dovetail. This determines both the orientation and width of the dovetail.

        middle_to_end (Vector): a vector that extends along the length of the dovetail. This should be perpendicular
            to middle_to_corner

        height (float): how thick the dovetail is. The dovetail will be extruded this distance along the up_vector

        tail_angle_degrees (float): the angle the side of the dovetail makes with middle_to_end. Must be be in range
            of 0 to 80 degrees. 0 degrees is a straight peg. 10 degrees is a typical value.

        inner_corner_radius (float): a radius placed at the narrow end of the dovetail. The hole for the dovetail
            should have a larger radius than the positive dovetail so that pieces mesh easily

        tail_corner_radius (float): a radius placed at the wide end of the dovetail. The positive part of the dovetail
            should have a larger radius than the hole for the dovetail so that pieces mesh easily

        up_vector (optional Vector): if provided this value will be used for the dovetail's up vector. If not provided
            the up vector will be computed from middle_to_corner and middle_to_end. This does not typically need to
            be provided.

        extra_base (float): an additional offset at the narrow end of the dovetail. Intended to handle cases where
            unwanted thin walls are being left after cut operations

    Returns:
        Solid shape
    """
    if tail_angle_degrees > 80 or tail_angle_degrees < 0:
        raise ValueError(f"Tail angle {tail_angle_degrees} out of range. Expected range of 0 to 80 degrees")

    # make a dovetail shaped object
    # middlePoint is the center of the notch
    # middleToCorner is the vector from middlePoint to the inner corner if cornerRadius is 0
    # middleToEnd is the vector from the middlePoint to the rear of the notch

    x = Vector(middle_to_corner).normalize()
    y = Vector(middle_to_end).normalize()
    if up_vector:
        z = Vector(up_vector).normalize()
    else:
        z = Vector(x.cross(y)).normalize()

    edges = []
    inner_corner = middle_point + middle_to_corner
    tail_middle = middle_point + middle_to_end

    # we're looking at the x positive side of the dovetail
    # an angle of 0 means that the dovetail side is perpendicular to the edge
    # an angle of 45 is out and +x
    # to get that subtract the specified angle from 90 degrees and convert to radians
    # so that the usual mechanics can be applied
    angle_rad = np.deg2rad(90 - tail_angle_degrees)
    angle_x_part = np.cos(angle_rad)
    angle_y_part = np.sin(angle_rad)

    # if the dovetail angle is 0 then ypart is sin(90-0) = 1 and the length of the vector is middelToEnd.Length
    # if the dovetail angle is 45 then the vector is longer due to the angle
    # the angle is restricted, so ypart is always a positive nonzero number
    angle_length = middle_to_end.Length / angle_y_part
    outer_corner = inner_corner + (x * angle_x_part + y * angle_y_part) * angle_length

    # need to track the current end in order to append features
    current_end = None

    base_offset = -y * extra_base

    if inner_corner_radius > 0:
        newCorner = inner_corner + x * (inner_corner_radius * 10)

        arcPts = make_corner_fillet_points(newCorner, inner_corner, outer_corner, inner_corner_radius)
        if extra_base > 0:
            edges.append(Part.LineSegment(middle_point + base_offset, arcPts[0] + base_offset))
            edges.append(Part.LineSegment(arcPts[0] + base_offset, arcPts[0]))
        else:
            edges.append(Part.LineSegment(middle_point, arcPts[0]))
        edges.append(Part.Arc(arcPts[0], arcPts[1], arcPts[2]))
        current_end = arcPts[2]
    else:
        edges.append(Part.LineSegment(middle_point, inner_corner))
        current_end = inner_corner

    if tail_corner_radius > 0:
        arcPts = make_corner_fillet_points(current_end, outer_corner, tail_middle, tail_corner_radius)
        edges.append(Part.LineSegment(current_end, arcPts[0]))
        edges.append(Part.Arc(arcPts[0], arcPts[1], arcPts[2]))
        current_end = arcPts[2]
    else:
        edges.append(Part.LineSegment(current_end, outer_corner))
        current_end = outer_corner

    edges.append(Part.LineSegment(current_end, tail_middle))

    if extra_base > 0:
        edges.append(Part.LineSegment(tail_middle, middle_point + base_offset))
    else:
        edges.append(Part.LineSegment(tail_middle, middle_point))

    # jump through the hoops to turn the edges into a face
    s = Part.Shape(edges)
    w = Part.Wire(s.Edges)
    f = Part.Face(w)

    # make a solid
    solid = f.extrude(z * height)

    # make the other half
    mirrored_solid = solid.mirror(middle_point, x)

    # fuse the parts and clean up the splitter
    return mirrored_solid.fuse(solid).removeSplitter()
