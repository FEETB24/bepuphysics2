using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace BepuPhysics
{

    public struct DefaultPoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        public Vector3 Gravity;
        public float LinearDamping;
        public float AngularDamping;
        Vector3 gravityDt;
        float linearDampingDt;
        float angularDampingDt;
        private float _dt;

        private QuickDictionary<int, float, IntComparer> _angularDampingDictionary;
        private QuickDictionary<int, float, IntComparer> _linearDampingDictionary;

        public struct IntComparer : IEqualityComparerRef<int>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Equals(ref int a, ref int b)
            {
                return a == b;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Hash(ref int item)
            {
                return item;
            }
        }
        public void CustomAngularDamping(int bodyIndex, float angularDamping, IUnmanagedMemoryPool pool)
        {
            if (angularDamping == AngularDamping)
            {
                _angularDampingDictionary.FastRemove(bodyIndex);
                return;
            }

            _angularDampingDictionary.Add(bodyIndex, angularDamping, pool);
        }

        public void CustomLinearDamping(int bodyIndex, float linearDamping, IUnmanagedMemoryPool pool)
        {
            if (linearDamping == LinearDamping)
            {
                _linearDampingDictionary.FastRemove(bodyIndex);
                return;
            }

            _linearDampingDictionary.Add(bodyIndex, linearDamping, pool);
        }

        public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
        
        public DefaultPoseIntegratorCallbacks(IUnmanagedMemoryPool pool, Vector3? gravity = null, float linearDamping = .03f, float angularDamping = .03f) : this()
        {
            Gravity = gravity ?? new Vector3(0, -9.81f, 0);
            LinearDamping = linearDamping;
            AngularDamping = angularDamping;
            _angularDampingDictionary = new QuickDictionary<int, float, IntComparer>(1, pool);
            _linearDampingDictionary = new QuickDictionary<int, float, IntComparer>(1, pool);
            _linearDampingDictionary.Add(-1, LinearDamping, pool);
            _angularDampingDictionary.Add(-1, AngularDamping, pool);
        }

        public void PrepareForIntegration(float dt)
        {
            _dt = dt;
            //No reason to recalculate gravity * dt for every body; just cache it ahead of time.
            gravityDt = Gravity * dt;
            //Since this doesn't use per-body damping, we can precalculate everything.
            linearDampingDt = (float)Math.Pow(MathHelper.Clamp(1 - LinearDamping, 0, 1), dt);
            angularDampingDt = (float)Math.Pow(MathHelper.Clamp(1 - AngularDamping, 0, 1), dt);

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IntegrateVelocity(int bodyIndex, in RigidPose pose, in BodyInertia localInertia, int workerIndex, ref BodyVelocity velocity)
        {
            //Note that we avoid accelerating kinematics. Kinematics are any body with an inverse mass of zero (so a mass of ~infinity). No force can move them.
            if (localInertia.InverseMass > 0)
            {
                // ref var handle = ref Simulation.Bodies.ActiveSet.IndexToHandle[0];
                // ref var index = ref Unsafe.Add(ref handle, bodyIndex);

                if (_linearDampingDictionary.TryGetValue(bodyIndex, out var linearDamping))
                {
                    linearDamping = (float)Math.Pow(MathHelper.Clamp(1 - linearDamping, 0, 1), _dt);
                    velocity.Linear = (velocity.Linear + gravityDt) * linearDamping;
                }
                else
                {
                    velocity.Linear = (velocity.Linear + gravityDt) * linearDampingDt;
                }

                if (_angularDampingDictionary.TryGetValue(bodyIndex, out var angularDamping))
                {
                    angularDamping = (float)Math.Pow(MathHelper.Clamp(1 - angularDamping, 0, 1), _dt);
                    velocity.Angular = velocity.Angular * angularDamping;
                }
                else
                {
                    velocity.Angular = velocity.Angular * angularDampingDt;
                }
            }
            //Implementation sidenote: Why aren't kinematics all bundled together separately from dynamics to avoid this per-body condition?
            //Because kinematics can have a velocity- that is what distinguishes them from a static object. The solver must read velocities of all bodies involved in a constraint.
            //Under ideal conditions, those bodies will be near in memory to increase the chances of a cache hit. If kinematics are separately bundled, the the number of cache
            //misses necessarily increases. Slowing down the solver in order to speed up the pose integrator is a really, really bad trade, especially when the benefit is a few ALU ops.

            //Note that you CAN technically modify the pose in IntegrateVelocity. The PoseIntegrator has already integrated the previous velocity into the position, but you can modify it again
            //if you really wanted to.
            //This is also a handy spot to implement things like position dependent gravity or per-body damping.
        }

        public void Dispose(IUnmanagedMemoryPool pool)
        {
            _angularDampingDictionary.Dispose(pool);
            _linearDampingDictionary.Dispose(pool);
        }

    }
    public unsafe struct DefaultNarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        public SpringSettings ContactSpringiness;

        public void Initialize(Simulation simulation)
        {
            //Use a default if the springiness value wasn't initialized.
            if (ContactSpringiness.AngularFrequency == 0 && ContactSpringiness.TwiceDampingRatio == 0)
                ContactSpringiness = new SpringSettings(30, 1);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ConfigureMaterial(out PairMaterialProperties pairMaterial)
        {
            pairMaterial.FrictionCoefficient = 4;
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = ContactSpringiness;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
        {
            return true;
        }
        public void Dispose()
        {
        }

        public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : struct, IContactManifold<TManifold>
        {
            ConfigureMaterial(out pairMaterial);
            return true;
        }
    }

}