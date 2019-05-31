using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using Demos.Demos;
using Demos.Port.EventHandler;

namespace Demos.Port.CollisionGroups
{
    public struct FeeSimNarrowPhaseCallbacks<TEventHandler> : INarrowPhaseCallbacks where TEventHandler : ICollisionEventHandler
    {
        public BodyProperty<DropCircleTest.BodyProperty> CollisionGroups;
        public CollisionEvents<TEventHandler> Events;

        public void Initialize(Simulation simulation)
        {
            CollisionGroups.Initialize(simulation.Bodies);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            //It's impossible for two statics to collide, and pairs are sorted such that bodies always come before statics.
            if (b.Mobility != CollidableMobility.Static)
            {
                return CollisionGroup.AllowDetection(CollisionGroups[a.Handle].Filter, CollisionGroups[b.Handle].Filter);
            }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return CollisionGroup.AllowDetection(CollisionGroups[childIndexA].Filter, CollisionGroups[childIndexB].Filter);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold,
            out PairMaterialProperties pairMaterial)
        {
            CreateMaterial(out pairMaterial);
            return CollisionGroup.AllowCollision(CollisionGroups[pair.A.Handle].Filter, CollisionGroups[pair.B.Handle].Filter);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold,
            out PairMaterialProperties pairMaterial)
        {
            CreateMaterial(out pairMaterial);
            return CollisionGroup.AllowCollision(CollisionGroups[pair.A.Handle].Filter, CollisionGroups[pair.B.Handle].Filter);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB,
            ConvexContactManifold* manifold)
        {
            return CollisionGroup.AllowCollision(CollisionGroups[pair.A.Handle].Filter, CollisionGroups[pair.B.Handle].Filter);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void CreateMaterial(out PairMaterialProperties pairMaterial)
        {
            pairMaterial.FrictionCoefficient = 1;
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose()
        {
            CollisionGroups.Dispose();
        }
    }
}