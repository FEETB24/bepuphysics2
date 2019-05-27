using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    public class DropCircleTest : Demo
    {
        private BodyReference _floorReference;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DefaultNarrowPhaseCallbacks(),
                new DefaultPoseIntegratorCallbacks(true /*new Vector3(0, -10, 0)*/));
            var boxShape = new Box(1, 1, 1);
            boxShape.ComputeInertia(1, out _boxInertia);
            _boxIndex = Simulation.Shapes.Add(boxShape);

            var staticShape = new Box(200, 1, 200);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);

            var staticDescription = new BodyDescription
            {
                LocalInertia = new BodyInertia(),
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings {Mode = ContinuousDetectionMode.Discrete},
                    Shape = staticShapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Activity = new BodyActivityDescription
                {
                    MinimumTimestepCountUnderThreshold = 32,
                    SleepThreshold = 0.1f,

                },
                Pose = new RigidPose
                {
                    Position = new Vector3(1, -0.5f, 1),
                    Orientation = Quaternion.Identity,
                },
                ConveyorSettings = new ConveyorSettings() {}
                
            };



            _floorIndex = Simulation.Bodies.Add(staticDescription);
            _floorReference = new BodyReference(_floorIndex, Simulation.Bodies);
        }

        private int _floorIndex;
        private TypedIndex _boxIndex;
        private BodyInertia _boxInertia;

        private void CreateCircleCube(int sampleCount, float radius, int floorCount)
        {
            var wpCnt = sampleCount;
            var increment = MathHelper.Pi * 2 / wpCnt;

            var tempPos = Vector3.Zero;
            var offset = new Vector3(0, 0, 5);
            for (var j = 0; j < floorCount; j++)
            {
                for (float i = 0; i < MathHelper.Pi * 2 - increment / 2; i += increment)
                {
                    tempPos = new Vector3((float)Math.Sin(i) * radius, j * 2, (float)Math.Cos(i) * radius);

                    var bodyDescription = new BodyDescription
                    {
                        //Make the uppermost block kinematic to hold up the rest of the chain.
                        LocalInertia = _boxInertia,
                        Pose = new RigidPose
                        {
                            Position = tempPos,
                            Orientation = Quaternion.Identity
                        },
                        Activity = new BodyActivityDescription
                        {
                            MinimumTimestepCountUnderThreshold = 32,
                            SleepThreshold = .01f
                        },
                        Collidable = new CollidableDescription { Shape = _boxIndex, SpeculativeMargin = .1f },
                    };
                    Simulation.Bodies.Add(bodyDescription);
                }
            }


            //var bodyDescription = new BodyDescription
            //{
            //    //Make the uppermost block kinematic to hold up the rest of the chain.
            //    LocalInertia = _boxInertia,
            //    Pose = new RigidPose
            //    {
            //        Position = new Vector3(0,0,10),
            //        Orientation = Quaternion.Identity
            //    },
            //    Activity = new BodyActivityDescription
            //    {
            //        MinimumTimestepCountUnderThreshold = 32,
            //        SleepThreshold = .01f
            //    },
            //    Collidable = new CollidableDescription { Shape = _boxIndex, SpeculativeMargin = .1f },
            //};
            //Simulation.Bodies.Add(bodyDescription);

        }


        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.Z))
            {
                CreateCircleCube(25, 16, 11);
                CreateCircleCube(25, 14, 10);
                CreateCircleCube(25, 12, 10);
                CreateCircleCube(25, 10, 9);
            }

            var conveyorvelocity = Vector3.Zero;
            var conveyorVelocityChanged = false;

            if (input.WasDown(OpenTK.Input.Key.Number1))
            {
                conveyorvelocity += new Vector3(2,0,0);
                conveyorVelocityChanged = true;
            }
            if (input.WasDown(OpenTK.Input.Key.Number2))
            {
                conveyorvelocity += new Vector3(-2, 0, 0);
                conveyorVelocityChanged = true;
            }
            if (input.WasDown(OpenTK.Input.Key.Number3))
            {
                conveyorvelocity += new Vector3(0, 0, 2);
                conveyorVelocityChanged = true;
            }
            if (input.WasDown(OpenTK.Input.Key.Number4))
            {
                conveyorvelocity += new Vector3(0, 0, -2);
                conveyorVelocityChanged = true;
            }

            if (conveyorVelocityChanged)
            {
                _floorReference.ConveyorSettings.ConveyorVelocity = conveyorvelocity;
                _floorReference.ConveyorSettings.IsLinearConveyor = true;
                _floorReference.Activity.SleepThreshold = -1f;
                Simulation.Awakener.AwakenBody(_floorIndex);
            }

            if (input.WasPushed(OpenTK.Input.Key.Number5))
            {
                _floorReference.ConveyorSettings.IsLinearConveyor = false;
                _floorReference.Velocity.Linear = Vector3.Zero;
                _floorReference.Activity.SleepThreshold = 0.1f;
            }
            base.Update(window, camera, input, dt);
        }
    }
}