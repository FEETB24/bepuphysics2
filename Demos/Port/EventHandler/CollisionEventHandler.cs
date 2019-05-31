using System.Numerics;
using System.Threading;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities.Collections;

namespace Demos.Port.EventHandler
{
    struct CollisionEventHandler : ICollisionEventHandler
    {
        public Simulation Simulation;
        public QuickList<CollidablePair> Pairs;

        public void OnContactAdded<TManifold>(CollidableReference eventSource, CollidablePair pair, ref TManifold contactManifold,
            in Vector3 contactOffset, in Vector3 contactNormal, float depth, int featureId, int contactIndex, int workerIndex) where TManifold : IContactManifold
        {
            //var other = pair.A.Packed == eventSource.Packed ? pair.B : pair.A;
            //Console.WriteLine($"Added contact: ({eventSource}, {other}): {featureId}");
            //Simply ignore any particles beyond the allocated space.
            var index = Interlocked.Increment(ref Pairs.Count) - 1;
            if (index < Pairs.Span.Length)
            {
                ref var particle = ref Pairs[index];

                particle.A = pair.A;
                particle.B = pair.B;
            }
        }
    }
}