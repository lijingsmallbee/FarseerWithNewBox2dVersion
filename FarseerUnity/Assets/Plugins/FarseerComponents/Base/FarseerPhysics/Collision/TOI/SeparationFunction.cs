using System.Diagnostics;
using Microsoft.Xna.Framework;
using VelcroPhysics.Collision.Distance;
using VelcroPhysics.Collision.Narrowphase;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.TOI
{
    public static class SeparationFunction
    {
        public static void Initialize(ref SimplexCache cache, DistanceProxy proxyA, ref Sweep sweepA, DistanceProxy proxyB, ref Sweep sweepB, float t1, out FVector2 axis, out FVector2 localPoint, out SeparationFunctionType type)
        {
            int count = cache.Count;
            Debug.Assert(0 < count && count < 3);

            Transform xfA, xfB;
            sweepA.GetTransform(out xfA, t1);
            sweepB.GetTransform(out xfB, t1);

            if (count == 1)
            {
                localPoint = FVector2.Zero;
                type = SeparationFunctionType.Points;
                FVector2 localPointA = proxyA.Vertices[cache.IndexA[0]];
                FVector2 localPointB = proxyB.Vertices[cache.IndexB[0]];
                FVector2 pointA = MathUtils.Mul(ref xfA, localPointA);
                FVector2 pointB = MathUtils.Mul(ref xfB, localPointB);
                axis = pointB - pointA;
                axis.Normalize();
            }
            else if (cache.IndexA[0] == cache.IndexA[1])
            {
                // Two points on B and one on A.
                type = SeparationFunctionType.FaceB;
                FVector2 localPointB1 = proxyB.Vertices[cache.IndexB[0]];
                FVector2 localPointB2 = proxyB.Vertices[cache.IndexB[1]];

                FVector2 a = localPointB2 - localPointB1;
                axis = new FVector2(a.Y, -a.X);
                axis.Normalize();
                FVector2 normal = MathUtils.Mul(ref xfB.q, axis);

                localPoint = 0.5f * (localPointB1 + localPointB2);
                FVector2 pointB = MathUtils.Mul(ref xfB, localPoint);

                FVector2 localPointA = proxyA.Vertices[cache.IndexA[0]];
                FVector2 pointA = MathUtils.Mul(ref xfA, localPointA);

                float s = FVector2.Dot(pointA - pointB, normal);
                if (s < 0.0f)
                {
                    axis = -axis;
                }
            }
            else
            {
                // Two points on A and one or two points on B.
                type = SeparationFunctionType.FaceA;
                FVector2 localPointA1 = proxyA.Vertices[cache.IndexA[0]];
                FVector2 localPointA2 = proxyA.Vertices[cache.IndexA[1]];

                FVector2 a = localPointA2 - localPointA1;
                axis = new FVector2(a.Y, -a.X);
                axis.Normalize();
                FVector2 normal = MathUtils.Mul(ref xfA.q, axis);

                localPoint = 0.5f * (localPointA1 + localPointA2);
                FVector2 pointA = MathUtils.Mul(ref xfA, localPoint);

                FVector2 localPointB = proxyB.Vertices[cache.IndexB[0]];
                FVector2 pointB = MathUtils.Mul(ref xfB, localPointB);

                float s = FVector2.Dot(pointB - pointA, normal);
                if (s < 0.0f)
                {
                    axis = -axis;
                }
            }

            //Velcro note: the returned value that used to be here has been removed, as it was not used.
        }

        public static float FindMinSeparation(out int indexA, out int indexB, float t, DistanceProxy proxyA, ref Sweep sweepA, DistanceProxy proxyB, ref Sweep sweepB, ref FVector2 axis, ref FVector2 localPoint, SeparationFunctionType type)
        {
            Transform xfA, xfB;
            sweepA.GetTransform(out xfA, t);
            sweepB.GetTransform(out xfB, t);

            switch (type)
            {
                case SeparationFunctionType.Points:
                    {
                        FVector2 axisA = MathUtils.MulT(ref xfA.q, axis);
                        FVector2 axisB = MathUtils.MulT(ref xfB.q, -axis);

                        indexA = proxyA.GetSupport(axisA);
                        indexB = proxyB.GetSupport(axisB);

                        FVector2 localPointA = proxyA.Vertices[indexA];
                        FVector2 localPointB = proxyB.Vertices[indexB];

                        FVector2 pointA = MathUtils.Mul(ref xfA, localPointA);
                        FVector2 pointB = MathUtils.Mul(ref xfB, localPointB);

                        float separation = FVector2.Dot(pointB - pointA, axis);
                        return separation;
                    }

                case SeparationFunctionType.FaceA:
                    {
                        FVector2 normal = MathUtils.Mul(ref xfA.q, axis);
                        FVector2 pointA = MathUtils.Mul(ref xfA, localPoint);

                        FVector2 axisB = MathUtils.MulT(ref xfB.q, -normal);

                        indexA = -1;
                        indexB = proxyB.GetSupport(axisB);

                        FVector2 localPointB = proxyB.Vertices[indexB];
                        FVector2 pointB = MathUtils.Mul(ref xfB, localPointB);

                        float separation = FVector2.Dot(pointB - pointA, normal);
                        return separation;
                    }

                case SeparationFunctionType.FaceB:
                    {
                        FVector2 normal = MathUtils.Mul(ref xfB.q, axis);
                        FVector2 pointB = MathUtils.Mul(ref xfB, localPoint);

                        FVector2 axisA = MathUtils.MulT(ref xfA.q, -normal);

                        indexB = -1;
                        indexA = proxyA.GetSupport(axisA);

                        FVector2 localPointA = proxyA.Vertices[indexA];
                        FVector2 pointA = MathUtils.Mul(ref xfA, localPointA);

                        float separation = FVector2.Dot(pointA - pointB, normal);
                        return separation;
                    }

                default:
                    Debug.Assert(false);
                    indexA = -1;
                    indexB = -1;
                    return 0.0f;
            }
        }

        public static float Evaluate(int indexA, int indexB, float t, DistanceProxy proxyA, ref Sweep sweepA, DistanceProxy proxyB, ref Sweep sweepB, ref FVector2 axis, ref FVector2 localPoint, SeparationFunctionType type)
        {
            Transform xfA, xfB;
            sweepA.GetTransform(out xfA, t);
            sweepB.GetTransform(out xfB, t);

            switch (type)
            {
                case SeparationFunctionType.Points:
                    {
                        FVector2 localPointA = proxyA.Vertices[indexA];
                        FVector2 localPointB = proxyB.Vertices[indexB];

                        FVector2 pointA = MathUtils.Mul(ref xfA, localPointA);
                        FVector2 pointB = MathUtils.Mul(ref xfB, localPointB);
                        float separation = FVector2.Dot(pointB - pointA, axis);

                        return separation;
                    }
                case SeparationFunctionType.FaceA:
                    {
                        FVector2 normal = MathUtils.Mul(ref xfA.q, axis);
                        FVector2 pointA = MathUtils.Mul(ref xfA, localPoint);

                        FVector2 localPointB = proxyB.Vertices[indexB];
                        FVector2 pointB = MathUtils.Mul(ref xfB, localPointB);

                        float separation = FVector2.Dot(pointB - pointA, normal);
                        return separation;
                    }
                case SeparationFunctionType.FaceB:
                    {
                        FVector2 normal = MathUtils.Mul(ref xfB.q, axis);
                        FVector2 pointB = MathUtils.Mul(ref xfB, localPoint);

                        FVector2 localPointA = proxyA.Vertices[indexA];
                        FVector2 pointA = MathUtils.Mul(ref xfA, localPointA);

                        float separation = FVector2.Dot(pointA - pointB, normal);
                        return separation;
                    }
                default:
                    Debug.Assert(false);
                    return 0.0f;
            }
        }
    }
}