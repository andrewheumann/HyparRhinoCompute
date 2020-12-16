using Elements;
using Elements.Geometry;
using Elements.Geometry.Solids;
using HyRhi;
using System.Collections.Generic;
using System.Linq;
using rg = Rhino.Geometry;

namespace RhinoComputeExample
{
    public static class RhinoComputeExample
    {
        /// <summary>
        /// The RhinoComputeExample function.
        /// </summary>
        /// <param name="model">The input model.</param>
        /// <param name="input">The arguments to the execution.</param>
        /// <returns>A RhinoComputeExampleOutputs instance containing computed results and the model with any new elements.</returns>
        public static RhinoComputeExampleOutputs Execute(Dictionary<string, Model> inputModels, RhinoComputeExampleInputs input)
        {
            // set up a model to hold elements
            var model = new Model();

            // Call rhino methods with regular rhino calls, or Rhino.Compute calls if required methods are unavailable 
            var box1 = rg.Brep.CreateFromBox(new rg.BoundingBox(new rg.Point3d(0, 0, 0), new rg.Point3d(input.Radius, input.Radius, input.Radius)));
            var box2 = rg.Brep.CreateFromBox(new rg.BoundingBox(new rg.Point3d(input.Radius * 0.5, input.Radius * 0.5, input.Radius * 0.5), new rg.Point3d(input.Radius * 1.5, input.Radius * 1.5, input.Radius * 1.5)));
            var booleanDiff = Rhino.Compute.BrepCompute.CreateBooleanDifference(box1, box2, 0.1).First();

            var boundaryCurves = booleanDiff.Faces.Select(f => f.OuterLoop.To3dCurve());

            // Use conversion extension methods from HyRhi / Conversion.cs
            var elementsSphereMesh = booleanDiff.ToMesh();
            var elementsCrvs = boundaryCurves.Select(c => c.ToPolygon());

            // create Hypar Elements from resulting geometry
            var meshElement = new MeshElement(elementsSphereMesh, BuiltInMaterials.Glass);
            var curveElements = elementsCrvs.Select(c => new ModelCurve(c));

            // add elements to model
            model.AddElement(meshElement);
            model.AddElements(curveElements);

            // construct output object
            var output = new RhinoComputeExampleOutputs();

            // add model to output
            output.Model = model;

            // return output
            return output;
        }
    }
}