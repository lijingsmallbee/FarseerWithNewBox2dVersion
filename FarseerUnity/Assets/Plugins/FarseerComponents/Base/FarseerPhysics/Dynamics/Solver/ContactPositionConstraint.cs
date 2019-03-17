using Microsoft.Xna.Framework;
using VelcroPhysics.Collision.Narrowphase;

namespace VelcroPhysics.Dynamics.Solver
{
    public sealed class ContactPositionConstraint
    {
        public int IndexA;
        public int IndexB;
        public float InvIA, InvIB;
        public float InvMassA, InvMassB;
        public FVector2 LocalCenterA, LocalCenterB;
        public FVector2 LocalNormal;
        public FVector2 LocalPoint;
        public FVector2[] LocalPoints = new FVector2[Settings.MaxManifoldPoints];
        public int PointCount;
        public float RadiusA, RadiusB;
        public ManifoldType Type;
    }
}