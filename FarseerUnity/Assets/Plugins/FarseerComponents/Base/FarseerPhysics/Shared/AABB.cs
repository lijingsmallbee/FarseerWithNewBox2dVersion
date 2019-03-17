using System;
using Microsoft.Xna.Framework;
using VelcroPhysics.Collision.RayCast;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Shared
{
    /// <summary>
    /// An axis aligned bounding box.
    /// </summary>
    public struct AABB
    {
        /// <summary>
        /// The lower vertex
        /// </summary>
        public FVector2 LowerBound;

        /// <summary>
        /// The upper vertex
        /// </summary>
        public FVector2 UpperBound;

        public AABB(FVector2 min, FVector2 max)
            : this(ref min, ref max) { }

        public AABB(FVector2 center, float width, float height)
            : this(center - new FVector2(width / 2, height / 2), center + new FVector2(width / 2, height / 2))
        {
        }

        public AABB(ref FVector2 min, ref FVector2 max)
        {
            LowerBound = new FVector2(Math.Min(min.X, max.X), Math.Min(min.Y, max.Y));
            UpperBound = new FVector2(Math.Max(min.X, max.X), Math.Max(min.Y, max.Y));
        }

        public float Width => UpperBound.X - LowerBound.X;

        public float Height => UpperBound.Y - LowerBound.Y;

        /// <summary>
        /// Get the center of the AABB.
        /// </summary>
        public FVector2 Center => 0.5f * (LowerBound + UpperBound);

        /// <summary>
        /// Get the extents of the AABB (half-widths).
        /// </summary>
        public FVector2 Extents => 0.5f * (UpperBound - LowerBound);

        /// <summary>
        /// Get the perimeter length
        /// </summary>
        public float Perimeter
        {
            get
            {
                float wx = UpperBound.X - LowerBound.X;
                float wy = UpperBound.Y - LowerBound.Y;
                return 2.0f * (wx + wy);
            }
        }

        /// <summary>
        /// Gets the vertices of the AABB.
        /// </summary>
        /// <value>The corners of the AABB</value>
        public Vertices Vertices
        {
            get
            {
                Vertices vertices = new Vertices(4);
                vertices.Add(UpperBound);
                vertices.Add(new FVector2(UpperBound.X, LowerBound.Y));
                vertices.Add(LowerBound);
                vertices.Add(new FVector2(LowerBound.X, UpperBound.Y));
                return vertices;
            }
        }

        /// <summary>
        /// First quadrant
        /// </summary>
        public AABB Q1 => new AABB(Center, UpperBound);

        /// <summary>
        /// Second quadrant
        /// </summary>
        public AABB Q2 => new AABB(new FVector2(LowerBound.X, Center.Y), new FVector2(Center.X, UpperBound.Y));

        /// <summary>
        /// Third quadrant
        /// </summary>
        public AABB Q3 => new AABB(LowerBound, Center);

        /// <summary>
        /// Forth quadrant
        /// </summary>
        public AABB Q4 => new AABB(new FVector2(Center.X, LowerBound.Y), new FVector2(UpperBound.X, Center.Y));

        /// <summary>
        /// Verify that the bounds are sorted. And the bounds are valid numbers (not NaN).
        /// </summary>
        /// <returns>
        /// <c>true</c> if this instance is valid; otherwise, <c>false</c>.
        /// </returns>
        public bool IsValid()
        {
            FVector2 d = UpperBound - LowerBound;
            bool valid = d.X >= 0.0f && d.Y >= 0.0f;
            return valid && LowerBound.IsValid() && UpperBound.IsValid();
        }

        /// <summary>
        /// Combine an AABB into this one.
        /// </summary>
        /// <param name="aabb">The AABB.</param>
        public void Combine(ref AABB aabb)
        {
            LowerBound = FVector2.Min(LowerBound, aabb.LowerBound);
            UpperBound = FVector2.Max(UpperBound, aabb.UpperBound);
        }

        /// <summary>
        /// Combine two AABBs into this one.
        /// </summary>
        /// <param name="aabb1">The aabb1.</param>
        /// <param name="aabb2">The aabb2.</param>
        public void Combine(ref AABB aabb1, ref AABB aabb2)
        {
            LowerBound = FVector2.Min(aabb1.LowerBound, aabb2.LowerBound);
            UpperBound = FVector2.Max(aabb1.UpperBound, aabb2.UpperBound);
        }

        /// <summary>
        /// Does this AABB contain the provided AABB.
        /// </summary>
        /// <param name="aabb">The AABB.</param>
        /// <returns>
        /// <c>true</c> if it contains the specified AABB; otherwise, <c>false</c>.
        /// </returns>
        public bool Contains(ref AABB aabb)
        {
            bool result = LowerBound.X <= aabb.LowerBound.X;
            result = result && LowerBound.Y <= aabb.LowerBound.Y;
            result = result && aabb.UpperBound.X <= UpperBound.X;
            result = result && aabb.UpperBound.Y <= UpperBound.Y;
            return result;
        }

        /// <summary>
        /// Determines whether the AABB contains the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns>
        /// <c>true</c> if it contains the specified point; otherwise, <c>false</c>.
        /// </returns>
        public bool Contains(ref FVector2 point)
        {
            //using epsilon to try and guard against float rounding errors.
            return (point.X > (LowerBound.X + float.Epsilon) && point.X < (UpperBound.X - float.Epsilon) &&
                    (point.Y > (LowerBound.Y + float.Epsilon) && point.Y < (UpperBound.Y - float.Epsilon)));
        }

        /// <summary>
        /// Test if the two AABBs overlap.
        /// </summary>
        /// <param name="a">The first AABB.</param>
        /// <param name="b">The second AABB.</param>
        /// <returns>True if they are overlapping.</returns>
        public static bool TestOverlap(ref AABB a, ref AABB b)
        {
            FVector2 d1 = b.LowerBound - a.UpperBound;
            FVector2 d2 = a.LowerBound - b.UpperBound;

            return (d1.X <= 0) && (d1.Y <= 0) && (d2.X <= 0) && (d2.Y <= 0);
        }

        /// <summary>
        /// Raycast against this AABB using the specified points and maxfraction (found in input)
        /// </summary>
        /// <param name="output">The results of the raycast.</param>
        /// <param name="input">The parameters for the raycast.</param>
        /// <returns>True if the ray intersects the AABB</returns>
        public bool RayCast(out RayCastOutput output, ref RayCastInput input, bool doInteriorCheck = true)
        {
            // From Real-time Collision Detection, p179.

            output = new RayCastOutput();

            float tmin = -Settings.MaxFloat;
            float tmax = Settings.MaxFloat;

            FVector2 p = input.Point1;
            FVector2 d = input.Point2 - input.Point1;
            FVector2 absD = MathUtils.Abs(d);

            FVector2 normal = FVector2.Zero;

            for (int i = 0; i < 2; ++i)
            {
                float absD_i = i == 0 ? absD.X : absD.Y;
                float lowerBound_i = i == 0 ? LowerBound.X : LowerBound.Y;
                float upperBound_i = i == 0 ? UpperBound.X : UpperBound.Y;
                float p_i = i == 0 ? p.X : p.Y;

                if (absD_i < Settings.Epsilon)
                {
                    // Parallel.
                    if (p_i < lowerBound_i || upperBound_i < p_i)
                    {
                        return false;
                    }
                }
                else
                {
                    float d_i = i == 0 ? d.X : d.Y;

                    float inv_d = 1.0f / d_i;
                    float t1 = (lowerBound_i - p_i) * inv_d;
                    float t2 = (upperBound_i - p_i) * inv_d;

                    // Sign of the normal vector.
                    float s = -1.0f;

                    if (t1 > t2)
                    {
                        MathUtils.Swap(ref t1, ref t2);
                        s = 1.0f;
                    }

                    // Push the min up
                    if (t1 > tmin)
                    {
                        if (i == 0)
                        {
                            normal.X = s;
                        }
                        else
                        {
                            normal.Y = s;
                        }

                        tmin = t1;
                    }

                    // Pull the max down
                    tmax = Math.Min(tmax, t2);

                    if (tmin > tmax)
                    {
                        return false;
                    }
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (doInteriorCheck && (tmin < 0.0f || input.MaxFraction < tmin))
            {
                return false;
            }

            // Intersection.
            output.Fraction = tmin;
            output.Normal = normal;
            return true;
        }
    }
}