using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuUtilities;

namespace Demos.Port
{
    public class CustomPositionLastTimestepper : ITimestepper
    {
        /// <summary>
        /// Fires after the sleeper completes and before bodies are integrated.
        /// </summary>
        public event TimestepperStageHandler Slept;
        /// <summary>
        /// Fires after bodies have had their velocities and bounding boxes updated, but before collision detection begins.
        /// </summary>
        public event TimestepperStageHandler BeforeCollisionDetection;
        /// <summary>
        /// Fires after all collisions have been identified, but before constraints are solved.
        /// </summary>
        public event TimestepperStageHandler CollisionsDetected;
        /// <summary>
        /// Fires after the solver executes and before body poses are integrated.
        /// </summary>
        public event TimestepperStageHandler ConstraintsSolved;
        /// <summary>
        /// Fires after bodies have their poses integrated and before data structures are incrementally optimized.
        /// </summary>
        public event TimestepperStageHandler PosesIntegrated;
        public void Timestep(Simulation simulation, float dt, IThreadDispatcher threadDispatcher = null)
        {
            simulation.Sleep(threadDispatcher);
            Slept?.Invoke(dt, threadDispatcher);

            simulation.IntegrateVelocitiesBoundsAndInertias(dt, threadDispatcher);
            BeforeCollisionDetection?.Invoke(dt, threadDispatcher);

            simulation.CollisionDetection(dt, threadDispatcher);
            CollisionsDetected?.Invoke(dt, threadDispatcher);


            int count = simulation.Bodies.ActiveSet.Count;
            ref var baseConveyorSettings = ref simulation.Bodies.ActiveSet.ConveyorSettings[0];
            ref var baseVelocities = ref simulation.Bodies.ActiveSet.Velocities[0];
            ref var basePose = ref simulation.Bodies.ActiveSet.Poses[0];

            for (int i = 0; i < count; i++)
            {
                ref var conveyorSettings = ref Unsafe.Add(ref baseConveyorSettings, i);
                ref var velocity = ref Unsafe.Add(ref baseVelocities, i);
                if (conveyorSettings.IsLinearConveyor)
                {
                    ref var pose = ref Unsafe.Add(ref basePose, i);
                    var globalVelocity = System.Numerics.Vector3.Transform(conveyorSettings.ConveyorVelocity, pose.Orientation);
                    velocity.Linear = conveyorSettings.LinearVelocity + globalVelocity;
                }

                if (conveyorSettings.IsAngularConveyor)
                {
                    velocity.Angular = conveyorSettings.AngularVelocity + conveyorSettings.ConveyorAngularVelocity;
                }
            }
            


            simulation.Solve(dt, threadDispatcher);
            ConstraintsSolved?.Invoke(dt, threadDispatcher);


            for (int i = 0; i < count; i++)
            {
                ref var conveyorSettings = ref Unsafe.Add(ref baseConveyorSettings, i);
                ref var velocity = ref Unsafe.Add(ref baseVelocities, i);
                if (conveyorSettings.IsLinearConveyor)
                {
                    velocity.Linear = conveyorSettings.LinearVelocity;
                }
                if (conveyorSettings.IsAngularConveyor)
                {
                    velocity.Angular = conveyorSettings.AngularVelocity;
                }
            }

            simulation.IntegratePoses(dt, threadDispatcher);
            PosesIntegrated?.Invoke(dt, threadDispatcher);

            simulation.IncrementallyOptimizeDataStructures(threadDispatcher);
        }
    }
}