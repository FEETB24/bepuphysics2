using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using Quaternion = System.Numerics.Quaternion;

namespace Demos.Demos
{
    public class MagicDemo: Demo
    {
        private BodyReference _boxReference;
        private BodyReference _baseReference;

        private bool _baseUp = true;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DefaultNarrowPhaseCallbacks(), new DefaultPoseIntegratorCallbacks(BufferPool));
            var boxShape = new Box(1, 1, 1);
            var baseShape = new Box(10, 1, 10);


            boxShape.ComputeInertia(1, out var boxInertia);

            var boxShapeIndex = Simulation.Shapes.Add(boxShape);
            var baseShapeIndex = Simulation.Shapes.Add(baseShape);


            var boxDescription = new BodyDescription
            {
                //Make the uppermost block kinematic to hold up the rest of the chain.
                LocalInertia = boxInertia,
                Pose = new RigidPose
                {
                    Position = new Vector3(0,2,0),
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

        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.Z))
            {
                _baseUp = !_baseUp;
                _baseReference.Pose.Position = _baseUp ? new Vector3(0, 0, 0) : new Vector3(0, -2, 0);
                Simulation.Awakener.AwakenBody(_baseReference.Handle);

            }

            if (input.WasPushed(OpenTK.Input.Key.X))
            {
                _boxReference.Pose.Position = new Vector3(0,2,0);
                Simulation.Awakener.AwakenBody(_boxReference.Handle);

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

            if(hasVelocity)
            Simulation.Awakener.AwakenBody(_baseReference.Handle);



            base.Update(window, camera, input, dt);
        }

    }
}