using Elements.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using rg = Rhino.Geometry;
using Rhino;
using Elements.Geometry.Interfaces;
using Rhino.Compute;

namespace HyRhi
{
    public struct HyparPlane
    {
        public HyparPlane(Vector3 origin, double rotation)
        {
            this.origin = origin;
            this.rotation = rotation;
        }
        public Vector3 origin { get; }
        public double rotation { get; }
    }

    public static class Conversion
    {

        public static Transform IdentityTransform => new Transform(0.0, 0.0, 0.0);

        public static Polygon ToPolygon(this rg.Polyline pl)
        {
            if (pl.IsClosed)
            {
                pl.RemoveAt(pl.Count - 1);
            }

            var vertices = new List<Vector3>();
            var lastVertex = rg.Point3d.Unset;
            foreach (var vertex in pl)
            {
                if (lastVertex == rg.Point3d.Unset || lastVertex.DistanceTo(vertex) > 0.0001)
                {
                    vertices.Add(vertex.ToVector3());
                }
                lastVertex = vertex;
            }

            var polygon = new Polygon(vertices);
            return polygon;
        }

        public static Polygon ToPolygon(this rg.Curve curve)
        {
            if (curve.IsPolyline() && curve is rg.PolylineCurve pcrv)
            {
                return pcrv.ToPolyline().ToPolygon();
            }
            return curve.ToPolyline(0.01, 0.1, 0.1, 100).ToPolyline().ToPolygon();
            // throw new ArgumentException("This curve is not a polygon, conversion will not continue");
        }

        public static HyparPlane ToHyparPlane(this rg.Plane p)
        {
            return new HyparPlane(p.Origin.ToVector3(), RhinoMath.ToDegrees(rg.Vector3d.VectorAngle(rg.Vector3d.XAxis, p.XAxis, rg.Plane.WorldXY)));
        }

        public static Line ToLine(this rg.Line line)
        {
            return new Line(line.From.ToVector3(), line.To.ToVector3());
        }

        public static Vector3 ToVector3(this rg.Vector3d v)
        {
            return new Vector3(v.X, v.Y, v.Z);
        }

        public static Vector3 ToVector3(this rg.Point3d p)
        {
            return new Vector3(p.X, p.Y, p.Z);
        }

        public static Vector3 Unitize(this Vector3 v)
        {
            var length = v.Length();
            if (length == 0) return v;
            return v / length;
        }

        public static Profile ToProfile(this rg.Polyline pl)
        {
            var polygon = pl.ToPolygon();
            var profile = new Profile(polygon);
            return profile;
        }

        public static rg.Point3d ToRgPoint(this Vector3 v)
        {
            return new rg.Point3d(v.X, v.Y, v.Z);
        }

        public static rg.Point3d ToRgPoint(this Elements.GeoJSON.Position p)
        {
            return new rg.Point3d(p.Longitude, p.Latitude, 0);
        }

        public static Elements.Material ToMaterial(this Rhino.DocObjects.Material mat)
        {
            var color = mat.DiffuseColor.ToColor();
            color.Alpha = 1.0 - mat.Transparency;
            var specularFactor = 0.1;
            var glossinessFactor = mat.Shine / Rhino.DocObjects.Material.MaxShine;
            var name = mat.Name;
            return new Elements.Material(color, specularFactor, glossinessFactor, false, null, true, Guid.NewGuid(), name);
        }
        
        private static System.Drawing.Color ToSystemColor(this Color color)
        {
            return System.Drawing.Color.FromArgb((int)(color.Alpha * 255),
                                                (int)(color.Red * 255),
                                                (int)(color.Green * 255),
                                                (int)(color.Blue * 255));
        }

        public static rg.Vector3d ToRgVector(this Vector3 v)
        {
            return new rg.Vector3d(v.X, v.Y, v.Z);
        }

        public static rg.Brep ToRgBrep(this Elements.Geometry.Solids.Extrude extrude)
        {
            var profile = extrude.Profile.ToSurface();
            var dir = extrude.Direction.ToRgVector();
            dir.Unitize();
            dir *= extrude.Height;
            return ExtrudeBrep(profile, dir);
        }

        public static rg.Brep ToRgBrep(this Elements.Geometry.Solids.Face face)
        {
            var outer = face.Outer.ToPolygon().ToRgPolyline();
            var inners = face.Inner?.Select(i => i.ToPolygon().ToRgPolyline());
            var curves = new List<rg.Curve>();
            curves.Add(new rg.PolylineCurve(outer));
            if (inners != null) curves.AddRange(inners.Select(i => new rg.PolylineCurve(i)));
            return Rhino.Compute.BrepCompute.CreatePlanarBreps(curves, 0.01).First();
        }

        public static rg.GeometryBase[] ApplyTransform(this rg.GeometryBase[] geometries, rg.Transform xform)
        {
            foreach (var b in geometries)
            {
                b.Transform(xform);
            }
            return geometries;
        }

        public static rg.BoundingBox GetBoundingBox(this Elements.GeometricElement elem)
        {
            if (elem.Representation != null)
            {
                return elem.Representation.ToBoundingBox();
            }
            else if (elem is ITessellate mesh)
            {
                return mesh.ToRgMesh().GetBoundingBox(false);
            }
            else
            {
                return rg.BoundingBox.Unset;
            }
        }

        public static rg.BoundingBox ToBoundingBox(this Representation r)
        {
            if (r == null) return rg.BoundingBox.Empty;
            var bbox = rg.BoundingBox.Empty;
            var ops = r.SolidOperations;
            foreach (var solid in ops)
            {
                if (!solid.IsVoid) // currently ignoring voids
                {
                    foreach (var vertex in solid.Solid.Vertices.Values)
                    {
                        bbox.Union(vertex.Point.ToRgPoint());
                    }
                }
            }

            return bbox;
        }

        public static rg.Brep[] ToRgBreps(this Representation rep)
        {
            List<rg.Brep> voids = new List<rg.Brep>();
            List<rg.Brep> solids = new List<rg.Brep>();
            foreach (var solidOp in rep.SolidOperations)
            {
                var faceSrfs = new List<rg.Brep>();
                var faces = solidOp.Solid.Faces;
                foreach (var face in faces.Values)
                {
                    faceSrfs.Add(face.ToRgBrep());
                }

                var brep = Rhino.Compute.BrepCompute.JoinBreps(faceSrfs, 0.01);
                if (solidOp.IsVoid)
                {
                    voids.AddRange(brep);
                }
                else
                {
                    solids.AddRange(brep);
                }
            }

            if (voids.Count == 0)
            {
                return solids.ToArray();
            }
            else
            {
                return Rhino.Compute.BrepCompute.CreateBooleanDifference(solids, voids, 0.01);
            }
        }

        private static rg.Brep ExtrudeBrep(rg.Brep brep, rg.Vector3d dir)
        {
            if (dir.IsZero)
            {
                return brep;
            }
            List<rg.Curve> edgeCurves = new List<rg.Curve>();
            List<rg.Brep> breps;
            checked
            {
                int num = brep.Edges.Count - 1;
                for (int i = 0; i <= num; i++)
                {
                    if (brep.Edges[i].TrimCount == 1)
                    {
                        rg.Curve item = brep.Edges[i].DuplicateCurve();
                        edgeCurves.Add(item);
                    }
                }
                if (edgeCurves.Count == 0)
                {
                    return brep;
                }
                breps = new List<rg.Brep>()
                {
                    brep
                };
                int num2 = edgeCurves.Count - 1;
                for (int j = 0; j <= num2; j++)
                {
                    rg.Surface extrusion = Rhino.Compute.SurfaceCompute.CreateExtrusion(edgeCurves[j], dir);
                    if (extrusion != null)
                    {
                        rg.Brep val2 = extrusion.ToBrep();
                        // val2.Faces.SplitKinkyFaces();
                        breps.Add(val2);
                    }
                }
                rg.Brep topFace = brep.DuplicateBrep();
                topFace.Translate(dir);
                breps.Add(topFace);
            }
            rg.Brep[] array = Rhino.Compute.BrepCompute.JoinBreps(breps, 0.0001);
            if (array == null)
            {
                return brep;
            }
            return array[0];
        }

        public static rg.Plane ToRgPlane(this Plane plane)
        {
            var origin = plane.Origin.ToRgPoint();
            var zAxis = plane.Normal.ToRgVector();
            return new rg.Plane(origin, zAxis);
        }

        public static rg.Transform ToRgTransform(this Transform transform)
        {
            rg.Transform t = rg.Transform.Identity;
            if (transform == null)
            {
                return t;
            }
            var mtx = transform.Matrix;

            t.M00 = mtx.m11;
            t.M01 = mtx.m21;
            t.M02 = mtx.m31;
            t.M03 = mtx.tx;
            t.M10 = mtx.m12;
            t.M11 = mtx.m22;
            t.M12 = mtx.m32;
            t.M13 = mtx.ty;
            t.M20 = mtx.m13;
            t.M21 = mtx.m23;
            t.M22 = mtx.m33;
            t.M23 = mtx.tz;
            t.M30 = 0;
            t.M31 = 0;
            t.M32 = 0;
            t.M33 = 1;

            return t;
        }

        public static rg.Line ToRgLine(this Line line)
        {
            return new rg.Line(line.Start.ToRgPoint(), line.End.ToRgPoint());
        }

        public static rg.Polyline ToRgPolyline(this Polygon polygon)
        {
            var vertices = polygon.Vertices.Select(v => v.ToRgPoint()).ToList();
            vertices.Add(vertices[0]);
            return new rg.Polyline(vertices);
        }

        public static rg.Polyline ToRgPolyline(this Polyline polyline)
        {
            var vertices = polyline.Vertices.Select(v => v.ToRgPoint()).ToList();
            return new rg.Polyline(vertices);
        }

        public static rg.Mesh ToRgMesh(this ITessellate tessellate)
        {
            var meshRep = new Mesh();
            tessellate.Tessellate(ref meshRep);
            var rhinoMeshes = meshRep.ToRgMesh();
            return rhinoMeshes;
        }

        public static rg.Mesh ToRgMesh(this Mesh mesh)
        {
            var meshOut = new rg.Mesh();
            foreach (var vertex in mesh.Vertices)
            {
                meshOut.Vertices.Add(vertex.Position.ToRgPoint());
                meshOut.Normals.Add(vertex.Normal.ToRgVector());
                if (!vertex.Color.Equals(default(Color)))
                {
                    meshOut.VertexColors.Add(vertex.Color.ToSystemColor());
                }
            }
            foreach (var face in mesh.Triangles)
            {
                meshOut.Faces.AddFace(face.Vertices[0].Index, face.Vertices[1].Index, face.Vertices[2].Index);
            }
            return meshOut;
        }

        internal static rg.Brep ToSurface(this Profile profile)
        {
            var boundary = profile.Perimeter.ToRgPolyline();
            var voids = profile.Voids?.Select(v => v.ToRgPolyline());
            var curves = new List<rg.Curve>();
            curves.Add(new rg.PolylineCurve(boundary));
            if (voids != null) curves.AddRange(voids.Select(v => new rg.PolylineCurve(v)));
            return Rhino.Compute.BrepCompute.CreatePlanarBreps(curves, 0.01).FirstOrDefault();
        }


        public static Mesh ToMesh(this rg.Brep brep)
        {
            var rgMesh = MeshCompute.CreateFromBrep(brep);
            var union = new rg.Mesh();
            union.Append(rgMesh);
            return union.ToMesh();
        }
        public static Mesh ToMesh(this rg.Mesh mesh)
        {
            List<Vertex> vertexCache = new List<Vertex>();
            var meshOut = new Mesh();
            var hasVertexColors = mesh.VertexColors.Count > 0;
            var hasVertexUVs = mesh.TextureCoordinates.Count > 0;
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                var vertex = mesh.Vertices[i];
                var vtxNormal = mesh.Normals[i];
                var vtxUV = hasVertexUVs ? mesh.TextureCoordinates[i].ToUV() : default(UV);
                var color = hasVertexColors ? mesh.VertexColors[i].ToColor() : default(Color);
                var newVertex = meshOut.AddVertex(vertex.ToVector3(), vtxUV, vtxNormal.ToVector3(), color);
                vertexCache.Add(newVertex);
            }
            foreach (var face in mesh.Faces)
            {
                if (face.IsQuad)
                {
                    var t1 = new Triangle(vertexCache[face.A], vertexCache[face.B], vertexCache[face.C]);
                    var t2 = new Triangle(vertexCache[face.C], vertexCache[face.D], vertexCache[face.A]);
                    meshOut.AddTriangle(t1);
                    meshOut.AddTriangle(t2);
                }
                else
                {
                    var triangle = new Triangle(vertexCache[face.A], vertexCache[face.B], vertexCache[face.C]);
                    meshOut.AddTriangle(triangle);
                }
            }
            return meshOut;
        }

        public static Color ToColor(this System.Drawing.Color c)
        {
            return new Color(c.R / 255.0, c.G / 255.0, c.B / 255.0, c.A / 255.0);
        }
        public static Vector3 ToVector3(this rg.Vector3f v)
        {
            return new Vector3(v.X, v.Y, v.Z);
        }


        public static Vector3 ToVector3(this rg.Point3f pt)
        {
            return new Vector3(pt.X, pt.Y, pt.Z);
        }

        public static UV ToUV(this rg.Point2f uv)
        {
            return new UV(uv.X, uv.Y);
        }

        public static Line ToLine(this rg.Curve line)
        {
            return new Line(line.PointAtStart.ToVector3(), line.PointAtEnd.ToVector3());
        }

        public static Polyline ToPolyline(this rg.Polyline pl)
        {
            var vertices = new List<Vector3>();
            var lastVertex = rg.Point3d.Unset;
            foreach (var vertex in pl)
            {
                if (lastVertex == rg.Point3d.Unset || lastVertex.DistanceTo(vertex) > 0.0001)
                {
                    vertices.Add(vertex.ToVector3());
                }
                lastVertex = vertex;
            }
            return new Polyline(vertices);
        }

        public static Polyline ToPolyline(this rg.Curve curve)
        {
            if (curve.IsPolyline() && curve is rg.PolylineCurve pcrv)
            {
                return pcrv.ToPolyline().ToPolyline();
            }

            if (curve.TryGetPolyline(out rg.Polyline polyline))
            {
                return polyline.ToPolyline();
            }
            return curve.ToPolyline(0.01, 0.1, 0.1, 100).ToPolyline().ToPolyline();
        }

        public static Profile ToProfile(this rg.Brep brep)
        {
            if (brep.Faces.Count > 1) return null;
            var face = brep.Faces[0];
            var outer = face.OuterLoop.To3dCurve().ToPolygon();
            var inner = face.Loops.Where(l => l.LoopType == rg.BrepLoopType.Inner).Select(l => l.To3dCurve().ToPolygon()).ToList();
            return new Profile(outer, inner, default(Guid), "");
        }

        public static Profile ToProfile(this rg.Curve crv)
        {
            var polygon = crv.ToPolygon();
            return new Profile(polygon);
        }

        public static Transform ToTransform(this rg.Transform xform)
        {
            var mtx = new Matrix();
            mtx.m11 = xform.M00;
            mtx.m12 = xform.M10;
            mtx.m13 = xform.M20;
            mtx.m21 = xform.M01;
            mtx.m22 = xform.M11;
            mtx.m23 = xform.M21;
            mtx.m31 = xform.M02;
            mtx.m32 = xform.M12;
            mtx.m33 = xform.M22;
            mtx.tx = xform.M03;
            mtx.ty = xform.M13;
            mtx.tz = xform.M23;

            return new Transform(mtx);
        }

        internal static string Capitalize(this string s)
        {
            return char.ToUpper(s[0]) + s.Substring(1);
        }
    }
}
