using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using Demos.Demos;

namespace Demos.Port.CollisionGroups
{
    public struct CollisionGroupcallbacks: INarrowPhaseCallbacks
    {
        public BodyProperty<DropCircleTest.BodyProperty> CollisionGroups;


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
                return CollisionGroup.AllowCollision(CollisionGroups[a.Handle].Filter, CollisionGroups[b.Handle].Filter);
            }
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold,
            out PairMaterialProperties pairMaterial)
        {
            CreateMaterial(out pairMaterial);
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold,
            out PairMaterialProperties pairMaterial)
        {
            CreateMaterial(out pairMaterial);
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB,
            ConvexContactManifold* manifold)
        {
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Flush()
        {

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