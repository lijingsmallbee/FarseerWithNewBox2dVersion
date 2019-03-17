using Microsoft.Xna.Framework;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision
{
    public static class TestPointHelper
    {
        public static bool TestPointCircle(ref FVector2 pos, float radius, ref FVector2 point, ref Transform transform)
        {
            FVector2 center = transform.p + MathUtils.Mul(transform.q, pos);
            FVector2 d = point - center;
            return FVector2.Dot(d, d) <= radius * radius;
        }

        public static bool TestPointPolygon(Vertices vertices, Vertices normals, ref FVector2 point, ref Transform transform)
        {
            FVector2 pLocal = MathUtils.MulT(transform.q, point - transform.p);

            for (int i = 0; i < vertices.Count; ++i)
            {
                float dot = FVector2.Dot(normals[i], pLocal - vertices[i]);
                if (dot > 0.0f)
                {
                    return false;
                }
            }

            return true;
        }
    }
}
