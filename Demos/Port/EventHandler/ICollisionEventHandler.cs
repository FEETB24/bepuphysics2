using System.Numerics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;

namespace Demos.Port.EventHandler
{
    public interface ICollisionEventHandler
    {
        void OnContactAdded<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold,
            in Vector3 contactOffset, in Vector3 contactNormal, float depth, int featureId, int contactIndex, int workerIndex) where TManifold : IContactManifold;
    }
}