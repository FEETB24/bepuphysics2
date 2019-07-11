using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Trees;

namespace Demos.Port.CollisionGroups
{
    public struct RayHitGroupHandler : IRayHitHandler
    {
        public float T;
        public CollidableReference HitCollidable;

        public int filter;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowTest(CollidableReference collidable)
        {
            //check the filter
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal, CollidableReference collidable)
        {
            //We are only interested in the earliest hit. This callback is executing within the traversal, so modifying maximumT informs the traversal
            //that it can skip any AABBs which are more distant than the new maximumT.
            if (t < maximumT)
                maximumT = t;
            if (t < T)
            {
                //Cache the earliest impact.
                T = t;
                HitCollidable = collidable;
            }
        }
    }
}