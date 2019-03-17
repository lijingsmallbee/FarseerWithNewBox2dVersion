using Microsoft.Xna.Framework;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision
{
    public static class AABBHelper
    {
        public static void ComputeEdgeAABB(ref FVector2 start, ref FVector2 end, ref Transform transform, out AABB aabb)
        {
            FVector2 v1 = MathUtils.Mul(ref transform, ref start);
            FVector2 v2 = MathUtils.Mul(ref transform, ref end);

            aabb.LowerBound = FVector2.Min(v1, v2);
            aabb.UpperBound = FVector2.Max(v1, v2);

            FVector2 r = new FVector2(Settings.PolygonRadius, Settings.PolygonRadius);
            aabb.LowerBound = aabb.LowerBound - r;
            aabb.UpperBound = aabb.UpperBound + r;
        }

        public static void ComputeCircleAABB(ref FVector2 pos, float radius, ref Transform transform, out AABB aabb)
        {
            FVector2 p = transform.p + MathUtils.Mul(transform.q, pos);
            aabb.LowerBound = new FVector2(p.X - radius, p.Y - radius);
            aabb.UpperBound = new FVector2(p.X + radius, p.Y + radius);
        }

        public static void ComputePolygonAABB(Vertices vertices, ref Transform transform, out AABB aabb)
        {
            FVector2 lower = MathUtils.Mul(ref transform, vertices[0]);
            FVector2 upper = lower;

            for (int i = 1; i < vertices.Count; ++i)
            {
                FVector2 v = MathUtils.Mul(ref transform, vertices[i]);
                lower = FVector2.Min(lower, v);
                upper = FVector2.Max(upper, v);
            }

            FVector2 r = new FVector2(Settings.PolygonRadius, Settings.PolygonRadius);
            aabb.LowerBound = lower - r;
            aabb.UpperBound = upper + r;
        }
    }
}
