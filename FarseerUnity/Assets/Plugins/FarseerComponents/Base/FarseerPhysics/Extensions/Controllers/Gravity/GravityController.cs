using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Extensions.Controllers.ControllerBase;

namespace VelcroPhysics.Extensions.Controllers.Gravity
{
    public class GravityController : Controller
    {
        public GravityController(float strength)
            : base(ControllerType.GravityController)
        {
            Strength = strength;
            MaxRadius = float.MaxValue;
            GravityType = GravityType.DistanceSquared;
            Points = new List<FVector2>();
            Bodies = new List<Body>();
        }

        public GravityController(float strength, float maxRadius, float minRadius)
            : base(ControllerType.GravityController)
        {
            MinRadius = minRadius;
            MaxRadius = maxRadius;
            Strength = strength;
            GravityType = GravityType.DistanceSquared;
            Points = new List<FVector2>();
            Bodies = new List<Body>();
        }

        public float MinRadius { get; set; }
        public float MaxRadius { get; set; }
        public float Strength { get; set; }
        public GravityType GravityType { get; set; }
        public List<Body> Bodies { get; set; }
        public List<FVector2> Points { get; set; }

        public override void Update(float dt)
        {
            FVector2 f = FVector2.Zero;

            foreach (Body worldBody in World.BodyList)
            {
                if (!IsActiveOn(worldBody))
                    continue;

                foreach (Body controllerBody in Bodies)
                {
                    if (worldBody == controllerBody || (worldBody.IsStatic && controllerBody.IsStatic) || !controllerBody.Enabled)
                        continue;

                    FVector2 d = controllerBody.Position - worldBody.Position;
                    float r2 = d.LengthSquared();

                    if (r2 <= Settings.Epsilon || r2 > MaxRadius * MaxRadius || r2 < MinRadius * MinRadius)
                        continue;

                    switch (GravityType)
                    {
                        case GravityType.DistanceSquared:
                            f = Strength / r2 * worldBody.Mass * controllerBody.Mass * d;
                            break;
                        case GravityType.Linear:
                            f = Strength / (float)Math.Sqrt(r2) * worldBody.Mass * controllerBody.Mass * d;
                            break;
                    }

                    worldBody.ApplyForce(ref f);
                }

                foreach (FVector2 point in Points)
                {
                    FVector2 d = point - worldBody.Position;
                    float r2 = d.LengthSquared();

                    if (r2 <= Settings.Epsilon || r2 > MaxRadius * MaxRadius || r2 < MinRadius * MinRadius)
                        continue;

                    switch (GravityType)
                    {
                        case GravityType.DistanceSquared:
                            f = Strength / r2 * worldBody.Mass * d;
                            break;
                        case GravityType.Linear:
                            f = Strength / (float)Math.Sqrt(r2) * worldBody.Mass * d;
                            break;
                    }

                    worldBody.ApplyForce(ref f);
                }
            }
        }

        public void AddBody(Body body)
        {
            Bodies.Add(body);
        }

        public void AddPoint(FVector2 point)
        {
            Points.Add(point);
        }
    }
}