using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    public class CraneDemo : Demo
    {
        private BodyReference _boxReference;
        private BodyReference _baseReference;

        private bool _baseUp = true;

        private int _jointIndex;

        private float _jointDistance = 5;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DefaultNarrowPhaseCallbacks(), new DefaultPoseIntegratorCallbacks(true/*new Vector3(0, -10, 0)*/));

            var boxShape = new Box(1, 1, 1);
            var baseShape = new Box(5, 1, 5);


            boxShape.ComputeInertia(2, out var boxInertia);

            var boxShapeIndex = Simulation.Shapes.Add(boxShape);
            var baseShapeIndex = Simulation.Shapes.Add(baseShape);

            Symmetric3x3.Scale(boxInertia.InverseInertiaTensor, .5f, out boxInertia.InverseInertiaTensor);

            var boxDescription = new BodyDescription
            {
                //Make the uppermost block kinematic to hold up the rest of the chain.
                LocalInertia = boxInertia,
                Pose = new RigidPose
                {
                    Position = new Vector3(0, -2, 0),
                    Orientation = Quaternion.Identity
                },
                Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = .01f },
                Collidable = new CollidableDescription { Shape = boxShapeIndex, SpeculativeMargin = .1f },
            };


            var baseDescription = new BodyDescription
            {
                //Make the uppermost block kinematic to hold up the rest of the chain.
                LocalInertia = new BodyInertia(),
                Pose = new RigidPose
                {
                    Position = new Vector3(0, 0, 0),
                    Orientation = Quaternion.Identity
                },
                Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = .01f },
                Collidable = new CollidableDescription { Shape = baseShapeIndex, SpeculativeMargin = .1f },
            };

            var boxIndex = Simulation.Bodies.Add(boxDescription);
            var baseIndex = Simulation.Bodies.Add(baseDescription);

            _boxReference = new BodyReference(boxIndex, Simulation.Bodies);
            _baseReference = new BodyReference(baseIndex, Simulation.Bodies);


            var distanceJoint = new DistanceLimit(Vector3.Zero, new Vector3(0,.51f,0), 5f, 5f, new SpringSettings(5, 1));

            _jointIndex = Simulation.Solver.Add(baseIndex, boxIndex, distanceJoint);
            

        }

        unsafe void ChangeLenght(float lenght)
        {
            var distanceJoint = new DistanceLimit(Vector3.Zero, Vector3.Zero, lenght, lenght, new SpringSettings(2, 10f));
            Simulation.Solver.ApplyDescription(_jointIndex, ref distanceJoint);
        }


        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.Z))
            {
                _jointDistance++;
                ChangeLenght(_jointDistance);

            }

            if (input.WasPushed(OpenTK.Input.Key.X))
            {
                _jointDistance--;
                ChangeLenght(_jointDistance);

            }


            var hasVelocity = false;

            var newVelocity = Vector3.Zero;
            if (input.WasDown(OpenTK.Input.Key.Left))
            {
                hasVelocity = true;
                newVelocity.X += 10;
            }

            if (input.WasDown(OpenTK.Input.Key.Right))
            {
                hasVelocity = true;
                newVelocity.X += -10;
            }

            _baseReference.Velocity.Linear = newVelocity;

            if (hasVelocity)
                Simulation.Awakener.AwakenBody(_baseReference.Handle);



            base.Update(window, camera, input, dt);
        }

    }
}