﻿using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Shared;
using VelcroPhysics.Templates;
using VelcroPhysics.Tools.Triangulation.TriangulationBase;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Factories
{
    /// <summary>
    /// An easy to use factory for creating bodies
    /// </summary>
    public static class FixtureFactory
    {
        public static Fixture AttachEdge(FVector2 start, FVector2 end, Body body, object userData = null)
        {
            EdgeShape edgeShape = new EdgeShape(start, end);
            return body.CreateFixture(edgeShape, userData);
        }

        public static Fixture AttachChainShape(Vertices vertices, Body body, object userData = null)
        {
            ChainShape shape = new ChainShape(vertices);
            return body.CreateFixture(shape, userData);
        }

        public static Fixture AttachLoopShape(Vertices vertices, Body body, object userData = null)
        {
            ChainShape shape = new ChainShape(vertices, true);
            return body.CreateFixture(shape, userData);
        }

        public static Fixture AttachRectangle(float width, float height, float density, FVector2 offset, Body body, object userData = null)
        {
            Vertices rectangleVertices = PolygonUtils.CreateRectangle(width / 2, height / 2);
            rectangleVertices.Translate(ref offset);
            PolygonShape rectangleShape = new PolygonShape(rectangleVertices, density);
            return body.CreateFixture(rectangleShape, userData);
        }

        public static Fixture AttachCircle(float radius, float density, Body body, object userData = null)
        {
            if (radius <= 0)
                throw new ArgumentOutOfRangeException(nameof(radius), "Radius must be more than 0 meters");

            CircleShape circleShape = new CircleShape(radius, density);
            return body.CreateFixture(circleShape, userData);
        }

        public static Fixture AttachCircle(float radius, float density, Body body, FVector2 offset, object userData = null)
        {
            if (radius <= 0)
                throw new ArgumentOutOfRangeException(nameof(radius), "Radius must be more than 0 meters");

            CircleShape circleShape = new CircleShape(radius, density);
            circleShape.Position = offset;
            return body.CreateFixture(circleShape, userData);
        }

        public static Fixture AttachPolygon(Vertices vertices, float density, Body body, object userData = null)
        {
            if (vertices.Count <= 1)
                throw new ArgumentOutOfRangeException(nameof(vertices), "Too few points to be a polygon");

            PolygonShape polygon = new PolygonShape(vertices, density);
            return body.CreateFixture(polygon, userData);
        }

        public static Fixture AttachEllipse(float xRadius, float yRadius, int edges, float density, Body body, object userData = null)
        {
            if (xRadius <= 0)
                throw new ArgumentOutOfRangeException(nameof(xRadius), "X-radius must be more than 0");

            if (yRadius <= 0)
                throw new ArgumentOutOfRangeException(nameof(yRadius), "Y-radius must be more than 0");

            Vertices ellipseVertices = PolygonUtils.CreateEllipse(xRadius, yRadius, edges);
            PolygonShape polygonShape = new PolygonShape(ellipseVertices, density);
            return body.CreateFixture(polygonShape, userData);
        }

        public static List<Fixture> AttachCompoundPolygon(List<Vertices> list, float density, Body body, object userData = null)
        {
            List<Fixture> res = new List<Fixture>(list.Count);

            //Then we create several fixtures using the body
            foreach (Vertices vertices in list)
            {
                if (vertices.Count == 2)
                {
                    EdgeShape shape = new EdgeShape(vertices[0], vertices[1]);
                    res.Add(body.CreateFixture(shape, userData));
                }
                else
                {
                    PolygonShape shape = new PolygonShape(vertices, density);
                    res.Add(body.CreateFixture(shape, userData));
                }
            }

            return res;
        }

        public static Fixture AttachLineArc(float radians, int sides, float radius, bool closed, Body body)
        {
            Vertices arc = PolygonUtils.CreateArc(radians, sides, radius);
            arc.Rotate((MathHelper.Pi - radians) / 2);
            return closed ? AttachLoopShape(arc, body) : AttachChainShape(arc, body);
        }

        public static List<Fixture> AttachSolidArc(float density, float radians, int sides, float radius, Body body)
        {
            Vertices arc = PolygonUtils.CreateArc(radians, sides, radius);
            arc.Rotate((MathHelper.Pi - radians) / 2);

            //Close the arc
            arc.Add(arc[0]);

            List<Vertices> triangles = Triangulate.ConvexPartition(arc, TriangulationAlgorithm.Earclip);

            return AttachCompoundPolygon(triangles, density, body);
        }

        public static Fixture CreateFromTemplate(Body body, FixtureTemplate f1)
        {
            return body.CreateFixture(f1);
        }
    }
}