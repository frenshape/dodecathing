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
    very_small_number = 10 * App.Base.Precision.confusion()
    xAxis = Vector(xAxis).normalize()
    yAxis = Vector(yAxis).normalize()
    zAxis = xAxis.cross(yAxis).normalize()

    if abs(xAxis.dot(yAxis)) > very_small_number:
        xAxis = yAxis.cross(zAxis)
        print("Provided axes are not orthogonal. Regenerating x axis as {xAxis}")

    return Matrix(xAxis, yAxis, zAxis, xyzTranslate)


def transform_matrix_from_yz_basis(yAxis: Vector, zAxis: Vector, xyzTranslate: Vector = Vector(0, 0, 0)):
    """
    reminder: if xAxis is forward then yAxis is to the LEFT
    """
    very_small_number = 10 * App.Base.Precision.confusion()

    yAxis = Vector(yAxis).normalize()
    zAxis = Vector(zAxis).normalize()
    xAxis = -zAxis.cross(yAxis).normalize()

    if abs(zAxis.dot(yAxis)) > very_small_number:
        zAxis = -yAxis.cross(xAxis)
        print("Provided axes are not orthogonal. Regenerating z axis as {zAxis}")

    return Matrix(xAxis, yAxis, zAxis, xyzTranslate)


def get_span_dimensions(radius: float, pyramid_height: float):
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

    edgeXOffset = pyramid_height / face_to_pyramid_ratio
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
    cutterTolerance = fp.Tolerance * 0.5

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
    # print(dims)
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

        obj.addProperty(
            "App::PropertyLength", "SpanEdge1", "Decorative", "First edge thickness for the span"
        ).SpanEdge1 = 0.4
        obj.addProperty(
            "App::PropertyLength", "SpanEdge2", "Decorative", "Second edge thickness for the span"
        ).SpanEdge2 = 3

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
            "Distance between the edge and the start of the other arc",
        ).OtherArcDist = 12.0
        obj.addProperty(
            "App::PropertyLength", "SpanCenterSpace", "SpanDims", "Thickness of the span's center"
        ).SpanCenterSpace = 4.0

        obj.addProperty("App::PropertyLength", "DoveLength", "Dovetail", "Length of the dovetail").DoveLength = 6.0
        obj.addProperty(
            "App::PropertyLength", "DoveWidth", "Dovetail", "Width of the dovetail at the smallest point"
        ).DoveWidth = 8.0

        obj.addProperty(
            "App::PropertyLength", "Tolerance", "Printing", "Width of the dovetail at the smallest point"
        ).Tolerance = 0.2

        sk2 = App.ActiveDocument.addObject("Sketcher::SketchObjectPython", "DoDecaThingSketch")

        # This is slightly counterintuitive
        #
        # If you add App::GroupExtensionPython to obj and add new objects to the group you're
        # telling FreeCAD that obj is being built out of the objects in the group rather than
        # the objects in the group being built out of obj
        #
        # Instead, add the PropertyLink property to the sketch object and then add obj to the sketch's
        # link. That causes the sketch to be in obj's InList and when DodecaThing execute runs
        # and updates the sketch the document's dependency chains evaluate correctly and prevents the
        # "still touched after recompute" issue
        #
        # However, having a lot of ungrouped generated objects is annoying so the editor display is
        # tidied up by the View claiming any children which have a SourceDodecaThing link which is
        # pointing to obj.
        #
        # This general pattern is used for all generated objects

        sk2.addProperty("App::PropertyLink", "SourceDodecaThing").SourceDodecaThing = obj

        # tag the new sketch so that it can be found obj's InList
        sk2.addProperty("App::PropertyBool", "DodecaThingSketchObject").DodecaThingSketchObject = True
        sk2.setEditorMode("DodecaThingSketchObject", ("ReadOnly", "Hidden"))

        self.update_sketch(obj)

    def loads(self, state):
        # nothing should get serialized here
        return None

    def dumps(self):
        # nothing in this class should get serialized
        return None

    def onDocumentRestored(self, obj):
        pass

    def onChanged(self, fp, prop):
        pass

    def execute(self, fp):
        self.update_sketch(fp)

        if fp.MakeAllSpans:
            DodecaThing.place_all_spans(fp)
        else:
            DodecaThing.place_single_span(fp)

        if fp.MakeAllCorners:
            DodecaThing.place_all_corners(fp)
        else:
            DodecaThing.place_single_corner(fp)

        if fp.MakeDodecahedron:
            ddo = DodecaThing.get_dodecahedron_feature(fp)
            ddo.Shape = Part.Solid(Part.Shell(make_dodecahedron_faces(fp.Radius)))
        else:
            DodecaThing.remove_dodecahedron_feature(fp)

    @staticmethod
    def update_sketch(fp):
        sketch = DodecaThing.get_sketch(fp)
        if sketch:
            make_span_at_origin(fp, sketch)

    @staticmethod
    def get_sketch(fp):
        if hasattr(fp, "InList"):
            for obj in fp.InList:
                if hasattr(obj, "DodecaThingSketchObject"):
                    return obj

        return None

    @staticmethod
    def get_dodecahedron_feature(fp):
        if hasattr(fp, "InList"):
            for obj in fp.InList:
                if hasattr(obj, "DodecaThingEnclosingDDG"):
                    if obj.SourceDodecaThing == fp:
                        return obj

        obj = fp.Document.addObject("Part::Feature", "Dodecahedron")
        obj.addProperty("App::PropertyLink", "SourceDodecaThing").SourceDodecaThing = fp
        obj.addProperty("App::PropertyBool", "DodecaThingEnclosingDDG").DodecaThingEnclosingDDG = True
        obj.setEditorMode("DodecaThingEnclosingDDG", ("ReadOnly", "Hidden"))

        # the hierarchy fails to update sometimes if fp isn't touched after adding a new feature
        fp.touch()
        return obj

    @staticmethod
    def remove_dodecahedron_feature(fp):
        if hasattr(fp, "InList"):
            for obj in fp.InList:
                if hasattr(obj, "DodecaThingEnclosingDDG"):
                    obj.Document.removeObject(obj.Name)

    @staticmethod
    def get_corner_feature(fp, corner_index: int):
        if corner_index < 0:
            raise ValueError("Expected a non negative integer for span index")

        if hasattr(fp, "InList"):
            for obj in fp.InList:
                if hasattr(obj, "DodecaThingCornerIndex"):
                    if obj.DodecaThingCornerIndex == corner_index:
                        return obj

        obj = fp.Document.addObject("Part::Feature", "Corner000")
        obj.addProperty("App::PropertyLink", "SourceDodecaThing").SourceDodecaThing = fp
        obj.addProperty("App::PropertyInteger", "DodecaThingCornerIndex").DodecaThingCornerIndex = corner_index
        obj.setEditorMode("DodecaThingCornerIndex", ("ReadOnly", "Hidden"))

        # the hierarchy fails to update sometimes if fp isn't touched after adding a new feature
        fp.touch()
        return obj

    @staticmethod
    def remove_corners_above_index(fp, corner_index: int):
        objsToRemove = []
        if hasattr(fp, "InList"):
            for obj in fp.InList:
                if hasattr(obj, "DodecaThingCornerIndex"):
                    if obj.DodecaThingCornerIndex > corner_index:
                        objsToRemove.append(obj.Name)

        for objName in objsToRemove:
            fp.Document.removeObject(objName)

    @staticmethod
    def get_span_feature(fp, span_index: int):
        if span_index < 0:
            raise ValueError("Expected a non negative integer for span index")

        if hasattr(fp, "InList"):
            for obj in fp.InList:
                if hasattr(obj, "DodecaThingSpanIndex"):
                    if obj.DodecaThingSpanIndex == span_index:
                        return obj

        obj = fp.Document.addObject("Part::Feature", "Span000")
        obj.addProperty("App::PropertyLink", "SourceDodecaThing").SourceDodecaThing = fp
        obj.addProperty("App::PropertyInteger", "DodecaThingSpanIndex").DodecaThingSpanIndex = span_index
        obj.setEditorMode("DodecaThingSpanIndex", ("ReadOnly", "Hidden"))

        # the hierarchy fails to update sometimes if fp isn't touched after adding a new feature
        fp.touch()
        return obj

    @staticmethod
    def remove_spans_above_index(fp, span_index: int):
        objsToRemove = []
        if hasattr(fp, "InList"):
            for obj in fp.InList:
                if hasattr(obj, "DodecaThingSpanIndex"):
                    if obj.DodecaThingSpanIndex > span_index:
                        objsToRemove.append(obj.Name)

        for objName in objsToRemove:
            fp.Document.removeObject(objName)

    @staticmethod
    def make_span(fp):
        shapes = []

        if sketch_obj := DodecaThing.get_sketch(fp):
            for w in sketch_obj.Shape.Wires:
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

        up_faces = [f for f in working_shape.Faces if f.normalAt(0, 0).z > 0.99]
        if up_faces:
            if len(up_faces) > 1:
                print("Unexpected number of upward facing faces. Insetting may not work as expected")
            f = up_faces[0]
            w = f.OuterWire

            w.fix(1e-4, 1e-4, 1e-4)

            # make the decorative edge. This makes it easy to outline the shape with a different color
            if fp.SpanEdge1 > 0:
                valid_cutters = []
                edge_thickness = float(fp.SpanEdge1)
                offset1 = (-edge_thickness) - 0.1
                offset2 = -edge_thickness
                inset = w.makeOffset2D(offset1)
                outset = w.makeOffset2D(offset2)

                edge_dist = Vector(0, 0, -0.1)
                cutter = Part.makeFace([outset, inset], "Part::FaceMakerBullseye").extrude(2 * edge_dist)
                cutter.translate(-edge_dist)

                if cutter.isValid():
                    working_shape = working_shape.cut(cutter)
                else:
                    Console.PrintWarning("Failed to make decorative edge cutter")

            # make the interior cut
            if fp.SpanEdge2 > 0:
                valid_cutters = []
                inset = w.makeOffset2D(-fp.SpanEdge2)
                if inset.isValid():
                    cutters = [Part.Face(wire).extrude(Vector(0, 0, -fp.SpanThick * 2.0)) for wire in inset.Wires]
                    for c in cutters:
                        if not c.isValid():
                            Console.PrintWarning(
                                "Skipping invalid cutter produced by SpanEdge2. Try adjusting this value by a small amount"
                            )
                        else:
                            valid_cutters.append(c)
                    if valid_cutters:
                        working_shape = working_shape.cut(valid_cutters)
                else:
                    print("Edge not valid")
                    Console.PrintWarning(
                        "SpanEdge2 produced an invalid face. No span cut will occur. Try adjusting the number by a small amount"
                    )

        return Part.Solid(working_shape)

    @staticmethod
    def place_single_span(fp):
        shape = DodecaThing.make_span(fp)
        span = DodecaThing.get_span_feature(fp, 0)
        span.Shape = shape
        DodecaThing.remove_spans_above_index(fp, 0)

    @staticmethod
    def place_all_spans(fp):
        shape = DodecaThing.make_span(fp)
        decaFaces = make_dodecahedron_faces(float(fp.Radius))
        faces = get_opposing_faces(decaFaces)

        for span_index, face in enumerate(faces):
            m, n = face
            v1 = decaFaces[m].CenterOfMass
            v2 = decaFaces[n].CenterOfMass
            norm_m = decaFaces[m].normalAt(0, 0)

            v12 = v2 - v1
            v12norm = Vector(v12).normalize()

            vLeft = v12norm.cross(norm_m).normalize()
            m = transform_matrix_from_xy_basis(v12norm, vLeft, v1)
            span = DodecaThing.get_span_feature(fp, span_index)
            span.Shape = shape.transformed(m)

    @staticmethod
    def make_corner(fp):
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

        # fuse in the additional material
        shape = shape.fuse(loft)

        # and now dovetails
        width_vector = face1_right_vector * (float(fp.DoveWidth) * 0.5)
        length_vector = face1_up_vector * float(fp.DoveLength)
        dt_shape = make_dovetail_shape(face1_midpoint, width_vector, length_vector, -float(fp.SpanThick), extra_base=1)

        # one cutter for each side
        cutters = [
            dt_shape.transformed(Rotation(Vector(0, 0, 1), Radian=theta).toMatrix())
            for theta in np.linspace(0, 2 * np.pi, 5, False)
        ]
        shape = shape.cut(cutters)
        return Part.Solid(shape)

    @staticmethod
    def place_single_corner(fp):
        shape = DodecaThing.make_corner(fp)
        corner = DodecaThing.get_corner_feature(fp, 0)
        corner.Shape = shape
        DodecaThing.remove_corners_above_index(fp, 0)

    @staticmethod
    def place_all_corners(fp):
        shape = DodecaThing.make_corner(fp)
        deca_faces = make_dodecahedron_faces(float(fp.Radius))
        faces = get_opposing_faces(deca_faces)

        done_faces = set()
        for face in faces:
            m, n = face
            if m not in done_faces:
                corner_index = len(done_faces)
                done_faces.add(m)
                norm = deca_faces[m].normalAt(0, 0)
                v1 = deca_faces[m].CenterOfMass
                v2 = deca_faces[n].CenterOfMass
                v12norm = Vector(v2 - v1).normalize()

                v_left = v12norm.cross(norm).normalize()

                xfrm = transform_matrix_from_yz_basis(-v_left, -norm, v1)
                corner = DodecaThing.get_corner_feature(fp, corner_index)
                corner.Shape = shape.transformed(xfrm)

            if n not in done_faces:
                corner_index = len(done_faces)
                done_faces.add(n)
                norm = deca_faces[n].normalAt(0, 0)
                v1 = deca_faces[n].CenterOfMass
                v2 = deca_faces[m].CenterOfMass
                v12norm = Vector(v2 - v1).normalize()

                v_left = v12norm.cross(norm).normalize()

                xfrm = transform_matrix_from_yz_basis(-v_left, -norm, v1)
                corner = DodecaThing.get_corner_feature(fp, corner_index)
                corner.Shape = shape.transformed(xfrm)


class ViewThing:
    def __init__(self, viewObject):
        """Set this object to the proxy object of the actual view provider"""
        viewObject.Proxy = self

    def attach(self, viewObject):
        """Setup the scene sub-graph of the view provider, this method is mandatory"""
        # print(f"\nAttach {self} -> {viewObject}")
        self.viewObject = viewObject
        self.object = viewObject.Object
        return

    def claimChildren(self):
        controlled_objects = []
        for obj in self.object.InList:
            if hasattr(obj, "SourceDodecaThing") and obj.SourceDodecaThing == self.object:
                controlled_objects.append(obj)
            else:
                print(f"claim children rejecting object {obj}")

        # also consider: return controlled_objects + self.object.OutList
        return controlled_objects

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

    def setDisplayMode(self, mode):
        """Map the display mode defined in attach with those defined in getDisplayModes.
        Since they have the same names nothing needs to be done. This method is optional.
        """
        return mode

    def onChanged(self, vp, prop):
        """Print the name of the property that has changed"""
        # App.Console.PrintMessage("Change property: " + str(prop) + "\n")
        pass

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
