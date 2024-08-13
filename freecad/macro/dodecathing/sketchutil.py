from FreeCAD import Vector, Sketcher
import Part
from enum import IntEnum


# itertools pairwise is available starting in 3.10
# this macro was written for FreeCAD 0.22.x which uses python 3.8
def pairwise(iterable):
    # pairwise('ABCDEFG') → AB BC CD DE EF FG
    iterator = iter(iterable)
    a = next(iterator, None)
    for b in iterator:
        yield a, b
        a = b


class CDir(IntEnum):
    """An enum for compass directions.
    This is intentionally limited to 8 directions
    """

    North = 0
    NorthEast = 1
    East = 2
    SouthEast = 3
    South = 4
    SouthWest = 5
    West = 6
    NorthWest = 7


CDir2Vec = {
    CDir.North: Vector(0, 1, 0),
    CDir.NorthEast: Vector(1, 1, 0).normalize(),
    CDir.East: Vector(1, 0, 0),
    CDir.SouthEast: Vector(1, -1, 0).normalize(),
    CDir.South: Vector(0, -1, 0),
    CDir.SouthWest: Vector(-1, -1, 0).normalize(),
    CDir.West: Vector(-1, 0, 0),
    CDir.NorthWest: Vector(-1, 1, 0).normalize(),
}


def get_perpendicular_direction(direction: CDir) -> CDir:
    return CDir((direction + 2) % len(CDir))


class SketchUtil:
    """A collection of utility functions to simplify creating sketches in FreeCAD

    Note: Unless specified the Z coordinate will be ignored from arguments
    """

    def __init__(self, sketch):
        self.sketch = sketch

    def add_fixed_point(self, position: Vector) -> int:
        """Add a point to the sketch and constrain the point to the provided position

        Args:
          position (vector): the location for the point. Only the x and y values are used

        Return:
          the id of the created point
        """
        id = self.sketch.addGeometry(Part.Point(position))
        self.sketch.addConstraint(Sketcher.Constraint("DistanceX", id, 1, position.x))
        self.sketch.addConstraint(Sketcher.Constraint("DistanceY", id, 1, position.y))
        return id

    def add_fixed_line(self, line_start: Vector, line_end: Vector, is_construction: bool = False) -> int:
        """Add a line to the sketch and constrain the point to the provided positions

        Args:
            line_start (vector): the start of the line. Only the x and y values are used
            line_start (vector): the start of the line. Only the x and y values are used
            is_construction (bool): if true the line will be created as construction geometry
        Return:
          the id of the created line
        """
        id = self.sketch.addGeometry(Part.LineSegment(line_start, line_end), is_construction)
        self.sketch.addConstraint(Sketcher.Constraint("DistanceX", id, 1, line_start.x))
        self.sketch.addConstraint(Sketcher.Constraint("DistanceY", id, 1, line_start.y))
        self.sketch.addConstraint(Sketcher.Constraint("DistanceX", id, 2, line_end.x))
        self.sketch.addConstraint(Sketcher.Constraint("DistanceY", id, 2, line_end.y))
        return id

    def add_line_between_points(self, start_point: int, end_point: int, is_construction: bool = False) -> int:
        """Add a line to the sketch and constrain it to the provided points

        Args:
            start_point (int): id of starting point
            end_point (int): id of ending point

        Return:
            the id of the created line
        """
        vA = self.sketch.getPoint(start_point, 1)
        vB = self.sketch.getPoint(end_point, 1)
        lineId = self.sketch.addGeometry(Part.LineSegment(vA, vB), is_construction)
        self.sketch.addConstraint(Sketcher.Constraint("Coincident", lineId, 1, start_point, 1))
        self.sketch.addConstraint(Sketcher.Constraint("Coincident", lineId, 2, end_point, 1))
        return lineId

    def connect_points_with_lines(
        self, point_list, add_line_from_end_to_start: bool = False, is_construction: bool = False
    ):
        """Given a list of points add lines to the sketch and constrain those lines to the points

        Args:
            point_list (list of Vector): the points to connect
            add_line_from_end_to_start (bool): if true then connect the first and last points to make a line loop
            is_construction (bool): if true then add the lines as construction geometry
        """
        if len(point_list) < 2:
            return

        lines = [self.add_line_between_points(p[0], p[1]) for p in pairwise(point_list)]

        if add_line_from_end_to_start and len(point_list) > 2:
            # if there's only 2 points then they were already connected
            lines.append(self.add_line_between_points(point_list[-1], point_list[0], is_construction))

        return lines

    def add_radius_circle_at_point(self, point_id: int, radius: float, is_construction: bool = False):
        """Add a circle of specified radius to the sketch. Constrain the circle to be coincident to the provided point.
        Args:
            point_id (int): id of the point that will be at the center of the circle
            radius(float): radius of the circle

        Return:
            id of newly created circle
        """

        v = self.sketch.getPoint(point_id, 1)
        circle_id = self.sketch.addGeometry(Part.Circle(v, Vector(0, 0, 1), radius), is_construction)
        self.sketch.addConstraint(Sketcher.Constraint("Coincident", circle_id, 3, point_id, 1))
        self.sketch.addConstraint(Sketcher.Constraint("Radius", circle_id, radius))
        return circle_id

    def add_diameter_circle_at_point(self, point_id: int, diameter: float, is_construction: bool = False) -> int:
        """Add a circle of specified diameter to the sketch. Constrain the circle to be coincident to the provided point.
        Args:
            point_id (int): id of the point that will be at the center of the circle
            diameter(float): diameter of the circle

        Return:
            id of newly created circle
        """
        v = self.sketch.getPoint(point_id, 1)
        circle_id = self.sketch.addGeometry(Part.Circle(v, Vector(0, 0, 1), diameter), is_construction)
        self.sketch.addConstraint(Sketcher.Constraint("Coincident", circle_id, 3, point_id, 1))
        self.sketch.addConstraint(Sketcher.Constraint("Diameter", circle_id, diameter))
        return circle_id

    def add_point_on_line(self, line_id: int) -> int:
        """Add a point to the sketch and constrain it to the line indicated by line_id. The point will be added to the midpoint of the line.
        Args:
            line_id(int): id of the line used to constrain the point

        Return:
            id of newly added point
        """

        vA = self.sketch.getPoint(line_id, 1)
        vB = self.sketch.getPoint(line_id, 2)
        point_id = self.sketch.addGeometry(Part.Point(vA * 0.5 + vB * 0.5))
        self.sketch.addConstraint(Sketcher.Constraint("PointOnObject", point_id, 1, line_id))
        return point_id

    def constrain_point_distance(self, point_a_id: int, point_b_id: int, distance: float) -> int:
        """Add a distance constraint between two points
        Args:
            point_a_id (int): id of first point
            point_b_id (int): id of second point
            distance(float): value to use for the distance constraint

        Return:
            the id of the newly created constraint
        """
        return self.sketch.addConstraint(Sketcher.Constraint("Distance", point_a_id, 1, point_b_id, 1, distance))

    def add_point_on_circle(self, circle_id: int, direction: CDir = CDir.North) -> int:
        """Add a point to the sketch and constrain it to circle circle_id.

        Args:
            circle_id (int): id of the circle

            direction(CDir): the position to create the point relative to the center of the circle.
               This is the starting location for the point the point is not constrained to this position

        Return: id of the new point

        Notes:
            It might be useful to add an argument to constrain the point to the starting position
        """
        circle = self.sketch.Geometry[circle_id]
        if not circle.isDerivedFrom("Part::GeomCircle"):
            raise ValueError(f"Id {circle_id} was not a circle")

        if not direction in CDir2Vec:
            raise ValueError("Direction not found in lookup table")

        point = circle.Center + circle.Radius * CDir2Vec[direction]
        point_id = self.sketch.addGeometry(Part.Point(point))
        self.sketch.addConstraint(Sketcher.Constraint("PointOnObject", point_id, 1, circle_id))

        return point_id

    def add_arc_between_points(self, start_point: int, end_point: int, offset_direction: CDir) -> int:
        """Add an arc to the sketch between start_point and end_point. The endpoints of the arc are
        constrained to the provided points and middle of the arc is pushed in offset_direction.

        Args:
            start_point (int): id of the point to start the arc

            end_point (int): id of the point to end the arc

            offset_direction(CDir): direction to offset the midpoint of the arc relative to the
                midpoint between the start and end points.

        Return: id of new arc
        """
        vA = self.sketch.getPoint(start_point, 1)
        vB = self.sketch.getPoint(end_point, 1)
        vAB = vB - vA

        # offsetting by half of the AB length seemed to help the solver converge correctly
        # when adding tangent constraints
        vOff = CDir2Vec[offset_direction] * vAB.Length * 0.5
        midPoint = vA * 0.5 + vB * 0.5 + vOff
        arc_id = self.sketch.addGeometry(Part.Arc(vA, midPoint, vB))

        self.sketch.addConstraint(Sketcher.Constraint("Coincident", arc_id, 2, start_point, 1))
        self.sketch.addConstraint(Sketcher.Constraint("Coincident", arc_id, 1, end_point, 1))
        return arc_id

    def add_relative_point(self, point: int, direction: CDir, distance: float) -> int:
        """Add a point to the sketch and constrain it ot a position relative to the given point.

        Args:
            point (int): id of the point that will be used as the starting point

            direction(CDir): direction relative to the point

            distance(float): distance from the starting point to the created point

        Return:
            id of new point
        """
        vA = self.sketch.getPoint(point, 1)
        vDir = CDir2Vec[direction] * distance
        vNew = vA + vDir
        point_id = self.sketch.addGeometry(Part.Point(vNew))
        if vDir.x == 0:
            self.sketch.addConstraint(Sketcher.Constraint("Vertical", point, 1, point_id, 1))
        else:
            self.sketch.addConstraint(Sketcher.Constraint("DistanceX", point, 1, point_id, 1, vDir.x))

        if vDir.y == 0:
            self.sketch.addConstraint(Sketcher.Constraint("Horizontal", point, 1, point_id, 1))
        else:
            self.sketch.addConstraint(Sketcher.Constraint("DistanceY", point, 1, point_id, 1, vDir.y))

        return point_id

    def add_box_around_point(
        self, pointA: int, pointB: int, enclosedPoint: int, direction: CDir, closed=False, border=1
    ):
        """
        Create a box with corners at pointA and pointB that extends to enclose enclosedPoint.
        The sides will be parallel to the direction. If enclosedPoint is not between pointA and pointB
        with respect to direction then the box will not enclose the point

        borderExtends the box slightly past the point

        if closed is True the a line will be created between pointA and pointB

        default values are intended to support building cutters to modify geometry
        """
        vA = self.sketch.getPoint(pointA, 1)
        vB = self.sketch.getPoint(pointB, 1)
        vP = self.sketch.getPoint(enclosedPoint, 1)
        vAP = vP - vA
        vBP = vP - vB
        vAB = vB - vA

        vDir = CDir2Vec[direction]
        abDist = vDir.dot(vAB)

        # handle cases where the enclosed point is already contained by the box defined by the given points + direction
        aDist = max(vDir.dot(vAP), abDist, 0)
        bDist = max(vDir.dot(vBP), -abDist, 0)

        print(f"adist {aDist} bDist {bDist}")

        offA = self.add_relative_point(pointA, direction, aDist + border)
        offB = self.add_relative_point(pointB, direction, bDist + border)

        return [
            self.add_line_between_points(pointA, offA),
            self.add_line_between_points(pointB, offB),
            self.add_line_between_points(offA, offB),
            offA,
            offB,
        ]
