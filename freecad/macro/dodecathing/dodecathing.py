import FreeCAD as App
from FreeCAD import Vector, Units, Console, Rotation, Matrix, Sketcher
import Part
import numpy as np

from dodecathing.sketchutil import SketchUtil, CDir, pairwise
from dodecathing.dodecahedron import make_dodecahedron_faces, get_opposing_faces

from dodecathing.dovetail import make_dovetail_shape

otherParams = {"SpanThick"}
sketchParams = {"Radius", "TanArcDist", "OtherArcDist", "SpanCenterSpace", "PyramidHeight"}


def find_corner_angles_radians():
    phi = 1.618033988749895  # (1 + 5**0.5) / 2

    deca_faces = make_dodecahedron_faces(1)
    fcs = get_opposing_faces(deca_faces)
    m, n = fcs[0]

    # given m as the starting face n is one of 5 faces that are adjacent to the directly opposite face
    # v12 is the vector between those faces and norm_m points directly at the face opposing m
    v1 = deca_faces[m].CenterOfMass
    v2 = deca_faces[n].CenterOfMass
    norm_m = deca_faces[m].normalAt(0, 0)
    v12 = v2 - v1
    v12norm = Vector(v12).normalize()
    base_to_face = abs(norm_m.dot(v12norm))

    # for unit vectors the dot product is the cosine of the angle between the vectors
    face_to_base_rad = np.arccos(base_to_face)

    # now calculate the angle of the diamond
    #
    # by law of cosines tan(theta) = opposite / adjacent
    # length = v12.Length
    # ratio = phi + 1
    # side_adjacent = length
    # side_opposite = length / ratio
    # tan_theta2 = side_opposite/side_adjacent
    # (length/ratio) / length = (length/ratio) * (1/length) = 1/ratio

    ratio = phi + 1

    half_corner_angle_rad = np.arctan(1 / ratio)
    corner_angle_rad = 2 * half_corner_angle_rad

    # v12 describes the vector along the span face. I also need the angle between the face's edge and
    # the base angle (norm_m)
    vSide = v12norm.cross(norm_m).normalize()

    # for every ratio units along v12norm the edge moves 1 unit to the side

    vEdge = Vector(v12norm * ratio + vSide).normalize()

    edge_angle_rad = np.arccos(norm_m.dot(vEdge))

    return face_to_base_rad, corner_angle_rad, edge_angle_rad


# constants computed by findPyramidAnglesRadians
kFaceToBaseAngleRad = 0.5535743588970453
kCornerAngleRad = 0.7297276562269663
kEdgeToBaseAngleRad = 0.6523581397843684


def transform_matrix_from_xy_basis(xAxis: Vector, yAxis: Vector, xyzTranslate: Vector = Vector(0, 0, 0)):
    """
    reminder: if xAxis is forward then yAxis is to the LEFT
    """
    verySmallNumber = 10 * App.Base.Precision.confusion()
    xAxis = Vector(xAxis).normalize()
    yAxis = Vector(yAxis).normalize()
    zAxis = xAxis.cross(yAxis).normalize()

    if abs(xAxis.dot(yAxis)) > verySmallNumber:
        xAxis = yAxis.cross(zAxis)
        print("Provided axes are not orthogonal. Regenerating x axis as {xAxis}")

    return Matrix(xAxis, yAxis, zAxis, xyzTranslate)


def transform_matrix_from_yz_basis(yAxis: Vector, zAxis: Vector, xyzTranslate: Vector = Vector(0, 0, 0)):
    """
    reminder: if xAxis is forward then yAxis is to the LEFT
    """
    verySmallNumber = 10 * App.Base.Precision.confusion()

    yAxis = Vector(yAxis).normalize()
    zAxis = Vector(zAxis).normalize()
    xAxis = -zAxis.cross(yAxis).normalize()

    if abs(zAxis.dot(yAxis)) > verySmallNumber:
        zAxis = -yAxis.cross(xAxis)
        print("Provided axes are not orthogonal. Regenerating z axis as {zAxis}")

    return Matrix(xAxis, yAxis, zAxis, xyzTranslate)


def get_span_dimensions(radius: float, pyramidHeight: float):
    deca_faces = make_dodecahedron_faces(radius)
    fcs = get_opposing_faces(deca_faces)
    m, n = fcs[0]
    v1 = deca_faces[m].CenterOfMass
    v2 = deca_faces[n].CenterOfMass
    norm_m = deca_faces[m].normalAt(0, 0)

    # the face is a diamond of X:Y ratio 2.618 : 1
    phi = 1.618033988749895  # (1 + 5**0.5) / 2
    x_to_y_ratio = 1 / (phi + 1)  # convert X offsets to Y offsets

    v12 = v2 - v1
    v12norm = Vector(v12).normalize()

    # overall length of the face
    length = v12.Length

    # the height of the corner pyramid is specified in terms of norm_m
    # this face is an an angle to that vector, get the ratio between the vectors
    face_to_pyramid_ratio = abs(norm_m.dot(v12norm))

    edgeXOffset = pyramidHeight / face_to_pyramid_ratio
    edgeYOffset = edgeXOffset * x_to_y_ratio
    middleYOffset = 0.5 * length * x_to_y_ratio

    return {"length": length, "edgeXOffset": edgeXOffset, "edgeYOffset": edgeYOffset, "middleYOffset": middleYOffset}


def make_face_cutter(v0, vSide, vDown, cutterLength, cutterTolerance):
    pts = [v0, v0 + cutterLength * vSide, v0 + cutterLength * (vSide + vDown), v0 + cutterLength * vDown, v0]
    p = Part.makePolygon(pts)
    f = Part.Face(p)

    n = f.normalAt(0, 0)
    if n.z > 0:
        # print("Flipped")
        n = -n

    f2 = f.translated(-n * cutterTolerance)
    return Part.Solid(f2.extrude(n * cutterLength))


def make_cutter_set(fp):
    dims = get_span_dimensions(float(fp.Radius), float(fp.PyramidHeight))
    cutterLength = float(dims["length"] * 0.5)
    cutterTolerance = 0.1

    v1Base = Vector(np.cos(kFaceToBaseAngleRad), 0, -np.sin(kFaceToBaseAngleRad))
    v2Base = Vector(-np.cos(kFaceToBaseAngleRad), 0, -np.sin(kFaceToBaseAngleRad))
    v1L = Vector(np.cos(kCornerAngleRad * 0.5), np.sin(kCornerAngleRad * 0.5), 0)
    v2L = Vector(-np.cos(kCornerAngleRad * 0.5), np.sin(kCornerAngleRad * 0.5), 0)
    v1R = Vector(np.cos(kCornerAngleRad * 0.5), -np.sin(kCornerAngleRad * 0.5), 0)
    v2R = Vector(-np.cos(kCornerAngleRad * 0.5), -np.sin(kCornerAngleRad * 0.5), 0)
    vOrigin = Vector(0, 0, 0)
    vEnd = Vector(dims["length"], 0, 0)

    return [
        make_face_cutter(vOrigin, v1L, v1Base, cutterLength, cutterTolerance),
        make_face_cutter(vOrigin, v1R, v1Base, cutterLength, cutterTolerance),
        make_face_cutter(vEnd, v2L, v2Base, cutterLength, cutterTolerance),
        make_face_cutter(vEnd, v2R, v2Base, cutterLength, cutterTolerance),
    ]


def make_dovetail_connectors(fp):
    dims = get_span_dimensions(float(fp.Radius), float(fp.PyramidHeight))
    widthVec = Vector(0, float(fp.DoveWidth) * 0.5, 0)
    lengthVec = Vector(float(fp.DoveLength - fp.Tolerance), 0, 0)

    return [
        make_dovetail_shape(
            Vector(dims["edgeXOffset"], 0, 0),
            widthVec,
            -lengthVec,
            -float(fp.SpanThick),
            inner_corner_radius=0,
            tail_corner_radius=1,
        ),
        make_dovetail_shape(
            Vector(dims["length"] - dims["edgeXOffset"], 0, 0),
            -widthVec,
            lengthVec,
            -float(fp.SpanThick),
            inner_corner_radius=0,
            tail_corner_radius=1,
        ),
    ]


def make_span_at_origin(fp, sketch):
    """ """

    sketch.deleteAllGeometry()
    dims = get_span_dimensions(float(fp.Radius), float(fp.PyramidHeight))
    print(dims)
    tanArcDist = fp.TanArcDist
    deepArcDist = fp.OtherArcDist

    xpts = [dims["edgeXOffset"], dims["length"] / 2, dims["length"] - dims["edgeXOffset"]]
    ypts = [dims["edgeYOffset"], dims["middleYOffset"], dims["edgeYOffset"]]

    l_side = [Vector(x, y, 0) for x, y in zip(xpts, ypts)]
    r_side = [Vector(x, -y, 0) for x, y in zip(xpts, ypts)]

    su = SketchUtil(sketch)
    l_pts = [su.add_fixed_point(v) for v in l_side]
    r_pts = [su.add_fixed_point(v) for v in r_side]
    l_lines = [su.add_line_between_points(p1, p2, True) for p1, p2 in pairwise(l_pts)]
    r_lines = [su.add_line_between_points(p1, p2, True) for p1, p2 in pairwise(r_pts)]
    center = su.add_fixed_point(Vector(dims["length"] / 2, 0, 0))
    circle_id = su.add_diameter_circle_at_point(center, fp.SpanCenterSpace, True)

    # create arcs on the +y side

    arc_1_start = su.add_point_on_line(l_lines[0])
    su.constrain_point_distance(l_pts[0], arc_1_start, tanArcDist)
    arc_1_end = su.add_point_on_circle(circle_id, CDir.East)
    arc_id_1 = su.add_arc_between_points(arc_1_start, arc_1_end, CDir.NorthEast)
    sketch.addConstraint(Sketcher.Constraint("Tangent", arc_id_1, circle_id))
    sketch.addConstraint(Sketcher.Constraint("Tangent", arc_id_1, l_lines[0]))

    arc_2_start = su.add_point_on_line(l_lines[1])
    su.constrain_point_distance(arc_2_start, l_pts[2], deepArcDist)
    arc_id_2 = su.add_arc_between_points(arc_2_start, arc_1_end, CDir.SouthEast)
    sketch.addConstraint(Sketcher.Constraint("Tangent", arc_id_2, arc_id_1))

    # create arcs on the -y side
    arc_3_start = su.add_point_on_line(r_lines[1])
    su.constrain_point_distance(arc_3_start, r_pts[2], tanArcDist)
    arc_3_end = su.add_point_on_circle(circle_id, CDir.West)
    arc_id_3 = su.add_arc_between_points(arc_3_start, arc_3_end, CDir.SouthWest)
    sketch.addConstraint(Sketcher.Constraint("Tangent", arc_id_3, circle_id))
    sketch.addConstraint(Sketcher.Constraint("Tangent", arc_id_3, r_lines[1]))

    arc_4_start = su.add_point_on_line(r_lines[0])
    su.constrain_point_distance(arc_4_start, r_pts[0], deepArcDist)
    arc_id_4 = su.add_arc_between_points(arc_4_start, arc_3_end, CDir.NorthWest)
    sketch.addConstraint(Sketcher.Constraint("Tangent", arc_id_4, arc_id_3))

    # and now connect the dots
    su.connect_points_with_lines([arc_4_start, r_pts[0], l_pts[0], arc_1_start])
    su.connect_points_with_lines([arc_2_start, l_pts[2], r_pts[2], arc_3_start])

    sketch.recompute()
    sketch.Visibility = False
    return sketch


class DodecaThing:

    def __init__(self, obj):
        obj.Proxy = self
        obj.addProperty(
            "App::PropertyBool", "MakeAllSpans", "Control", "If true then all spans will be created"
        ).MakeAllSpans = True
        obj.addProperty(
            "App::PropertyBool", "MakeAllCorners", "Control", "If true then all corners will be created"
        ).MakeAllCorners = True
        obj.addProperty(
            "App::PropertyBool",
            "MakeDodecahedron",
            "Control",
            "If true then the enclosing dodecahedron will be created",
        ).MakeDodecahedron = False

        obj.addProperty("App::PropertyLength", "Radius", "Dims", "Radius of the dodecahedron").Radius = 100.0
        obj.addProperty("App::PropertyLength", "PyramidHeight", "Dims", "Height of the pyramids").PyramidHeight = 25
        obj.addProperty("App::PropertyLength", "SpanThick", "SpanDims", "Thickness of the spans").SpanThick = 4.0
        obj.addProperty(
            "App::PropertyLength",
            "TanArcDist",
            "SpanDims",
            "Distance between the edge and the start of the tangent arc",
        ).TanArcDist = 12.0
        obj.addProperty(
            "App::PropertyLength",
            "OtherArcDist",
            "SpanDims",
            "Distnace between the edge and the start of the other arc",
        ).OtherArcDist = 12.0
        obj.addProperty(
            "App::PropertyLength", "SpanCenterSpace", "SpanDims", "Thickness of the span's center"
        ).SpanCenterSpace = 4.0

        obj.addProperty("App::PropertyLength", "DoveLength", "Dovetail", "Length of the dovetail").DoveLength = 6.0
        obj.addProperty(
            "App::PropertyLength", "DoveWidth", "Dovetail", "Width of the dovetail at the smallest point"
        ).DoveWidth = 6.0

        obj.addProperty(
            "App::PropertyLength", "Tolerance", "Printing", "Width of the dovetail at the smallest point"
        ).Tolerance = 0.2

        obj.addExtension("App::GroupExtensionPython")

        sketchObj = obj.newObject("Sketcher::SketchObjectPython", "pysk")
        sketchObj.Visibility = False
        self.update_sketch(obj)

    def onDocumentRestored(self, obj):
        if hasattr(obj, "Group"):
            for g in obj.Group:
                if g.isDerivedFrom("Sketcher::SketchObjectPython"):
                    g.Visibility = False

    def onChanged(self, fp, prop):
        if prop in sketchParams:
            self.update_sketch(fp)

    def execute(self, fp):
        # updating the sketch here produces a loop where the scripted object and sketch oscillate as needing an update
        # that probably indicates that the code needs to be restructured

        spans = self.make_spans(fp)
        corners = self.make_corners(fp)
        dodecahedron = [Part.Shell(make_dodecahedron_faces(fp.Radius))] if fp.MakeDodecahedron else []
        fp.Shape = Part.Compound(spans + corners + dodecahedron)

    def update_sketch(self, fp):
        sketch = self.get_sketch(fp)
        if sketch:
            make_span_at_origin(fp, sketch)

    def get_sketch(self, fp):
        if hasattr(fp, "Group"):
            for g in fp.Group:
                if g.isDerivedFrom("Sketcher::SketchObjectPython"):
                    return g
        return None

    def make_spans(self, fp):
        sketchObj = self.get_sketch(fp)
        shapes = []

        if sketchObj:
            for w in sketchObj.Shape.Wires:
                f = Part.Face(w)
                shapes.append(f.extrude(Vector(0, 0, -fp.SpanThick)))

        if shapes:
            if len(shapes) == 1:
                working_shape = shapes[0]
            else:
                working_shape = Part.Compound(shapes)
        else:
            working_shape = Part.Shape()

        tails = make_dovetail_connectors(fp)
        working_shape = working_shape.fuse(tails)

        cutters = make_cutter_set(fp)
        working_shape = working_shape.cut(cutters)
        working_shape = working_shape.removeSplitter()

        if not fp.MakeAllSpans:
            return [working_shape]

        decaFaces = make_dodecahedron_faces(float(fp.Radius))
        faces = get_opposing_faces(decaFaces)
        allShapes = []

        for face in faces:
            m, n = face
            v1 = decaFaces[m].CenterOfMass
            v2 = decaFaces[n].CenterOfMass
            norm_m = decaFaces[m].normalAt(0, 0)

            v12 = v2 - v1
            v12norm = Vector(v12).normalize()

            vLeft = v12norm.cross(norm_m).normalize()
            m = transform_matrix_from_xy_basis(v12norm, vLeft, v1)
            allShapes.append(working_shape.transformed(m))

        return allShapes

    def make_corners(self, fp):
        peak = Vector(0, 0, 0)

        # edge angle is defined by
        # edgeAngle = Vector(np.sin(kEdgeToBaseAngleRad), 0, np.cos(kEdgeToBaseAngleRad))
        # to travel the height the vector must be height / cos(kEdgeToBaseAngleRad)

        edgeLength = fp.PyramidHeight / np.cos(kEdgeToBaseAngleRad)
        pointRadius = edgeLength * np.sin(kEdgeToBaseAngleRad)

        startPt = Vector(pointRadius, 0, -fp.PyramidHeight)
        base_points = [
            Rotation(Vector(0, 0, 1), Radian=theta) * startPt for theta in np.linspace(0, 2 * np.pi, 5, False)
        ]
        base_points.append(base_points[0])

        # build the top of the corner
        polyList = [Part.makePolygon([v1, v2, peak], True) for v1, v2 in pairwise(base_points)]
        polyList.append(Part.makePolygon(base_points))

        faceList = [Part.Face(p) for p in polyList]
        s = Part.Shell(faceList)
        shape = Part.Solid(s)

        # need material to support the dovetail
        # find vector perpendicular to a face

        face1_midpoint = base_points[0] * 0.5 + base_points[1] * 0.5
        face1_right_vector = (base_points[1] - base_points[0]).normalize()
        face1_up_vector = (peak - face1_midpoint).normalize()
        face1_normal_vector = face1_right_vector.cross(face1_up_vector)

        # the dovetail joins perpendicular to the face
        # create a point offset by the span thickness along the normal vector
        pt2 = face1_midpoint - float(fp.SpanThick) * face1_normal_vector

        # make a 5 element polar array using the right vector and the new point
        pts2 = [Rotation(Vector(0, 0, 1), Radian=theta) * pt2 for theta in np.linspace(0, 2 * np.pi, 5, False)]
        rts2 = [
            Rotation(Vector(0, 0, 1), Radian=theta) * face1_right_vector
            for theta in np.linspace(0, 2 * np.pi, 5, False)
        ]

        # duplicate endpoints so that loops catch the start-end pair
        pts2.append(pts2[0])
        rts2.append(rts2[0])

        # use the points to make a list of lines
        lines = [Part.Line(v, v + r) for v, r in zip(pts2, rts2)]
        bottom_points = []

        # and the points where these lines intersect form the bottom of our shape
        for l1, l2 in pairwise(lines):
            p = l1.intersect(l2)[0]
            bottom_points.append(Vector(p.X, p.Y, p.Z))

        polyA = Part.makePolygon(base_points)
        polyB = Part.makePolygon(bottom_points, True)
        loft = Part.makeLoft([polyA, polyB], True)

        # fuse in the aditional material
        shape = shape.fuse(loft)

        # and now dovetails
        widthVec = face1_right_vector * (float(fp.DoveWidth) * 0.5)
        lengthVec = face1_up_vector * float(fp.DoveLength)
        dt_shape = make_dovetail_shape(face1_midpoint, widthVec, lengthVec, -float(fp.SpanThick), extra_base=1)

        # one cutter for each side
        cutters = [
            dt_shape.transformed(Rotation(Vector(0, 0, 1), Radian=theta).toMatrix())
            for theta in np.linspace(0, 2 * np.pi, 5, False)
        ]
        shape = shape.cut(cutters)

        # shape's all done at this point
        # the code here replicates the shape and moves it to the correct locations
        if not fp.MakeAllCorners:
            return [shape]

        deca_faces = make_dodecahedron_faces(float(fp.Radius))
        faces = get_opposing_faces(deca_faces)
        allShapes = []

        done_faces = set()

        for face in faces:
            m, n = face
            if m not in done_faces:
                done_faces.add(m)
                norm = deca_faces[m].normalAt(0, 0)
                v1 = deca_faces[m].CenterOfMass
                v2 = deca_faces[n].CenterOfMass
                v12norm = Vector(v2 - v1).normalize()

                v_left = v12norm.cross(norm).normalize()

                xfrm = transform_matrix_from_yz_basis(-v_left, -norm, v1)
                allShapes.append(shape.transformed(xfrm))

            if n not in done_faces:
                done_faces.add(n)
                norm = deca_faces[n].normalAt(0, 0)
                v1 = deca_faces[n].CenterOfMass
                v2 = deca_faces[m].CenterOfMass
                v12norm = Vector(v2 - v1).normalize()

                v_left = v12norm.cross(norm).normalize()

                xfrm = transform_matrix_from_yz_basis(-v_left, -norm, v1)
                allShapes.append(shape.transformed(xfrm))

        return allShapes


class ViewThing:
    def __init__(self, viewObject):
        """Set this object to the proxy object of the actual view provider"""
        viewObject.Proxy = self

    def attach(self, viewObject):
        """Setup the scene sub-graph of the view provider, this method is mandatory"""
        print(f"\nAttach {self} -> {viewObject}")
        self.viewObject = viewObject
        self.object = viewObject.Object
        return

    def claimChildren(self):

        if hasattr(self, "object") and hasattr(self.object, "Group"):
            # print(f"Claim children {self} {self.object} {self.object.Group}")
            return self.object.Group
        print(f"ClaimChildren no kids {self}")
        return []

    def dumps(self):
        return None

    def loads(self, state):
        return None

    def updateData(self, fp, prop):
        """If a property of the handled feature has changed we have the chance to handle this here"""
        # App.Console.PrintMessage(f"updateData {self} {fp}")
        return

    def getDisplayModes(self, obj):
        """Return a list of display modes."""
        modes = []
        return modes

    def getDefaultDisplayMode(self):
        """Return the name of the default display mode. It must be defined in getDisplayModes."""
        return "Shaded"

    def setDisplayMode(self, mode):
        """Map the display mode defined in attach with those defined in getDisplayModes.
        Since they have the same names nothing needs to be done. This method is optional.
        """
        return mode

    def onChanged(self, vp, prop):
        """Print the name of the property that has changed"""
        App.Console.PrintMessage("Change property: " + str(prop) + "\n")

    def getIcon(self):
        """Return the icon in XMP format which will appear in the tree view. This method is optional
        and if not defined a default icon is shown.
        """
        return """
            /* XPM */
            static const char * ViewProviderBox_xpm[] = {
            "16 16 6 1",
            "   c None",
            ".  c #141010",
            "+  c #615BD2",
            "@  c #C39D55",
            "#  c #000000",
            "$  c #57C355",
            "        ........",
            "   ......++..+..",
            "   .@@@@.++..++.",
            "   .@@@@.++..++.",
            "   .@@  .++++++.",
            "  ..@@  .++..++.",
            "###@@@@ .++..++.",
            "##$.@@$#.++++++.",
            "#$#$.$$$........",
            "#$$#######      ",
            "#$$#$$$$$#      ",
            "#$$#$$$$$#      ",
            "#$$#$$$$$#      ",
            " #$#$$$$$#      ",
            "  ##$$$$$#      ",
            "   #######      "};
            """
