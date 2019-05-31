using System;
using System.Collections.Generic;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using Demos.Port.CollisionGroups;
using Demos.Port.EventHandler;
using DemoUtilities;
using OpenTK.Input;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    public class DropCircleTest : Demo
    {
        private const int NumberOfFloors = 20;
        private BodyProperty<BodyProperty> _bodyProperties;


        private TypedIndex _boxIndex;
        private BodyInertia _boxInertia;
        private FeeSimNarrowPhaseCallbacks<CollisionEventHandler> _collisionGroups;


        private CollisionEvents<CollisionEventHandler> _events;

        private List<BodyReference> _floorReferences;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;
            _bodyProperties = new BodyProperty<BodyProperty>(BufferPool);

            _events = new CollisionEvents<CollisionEventHandler>(new CollisionEventHandler(), BufferPool,
                ThreadDispatcher);
            _events.EventHandler.Pairs = new QuickList<CollidablePair>(128, BufferPool);
            _events.EventHandler.Simulation = Simulation;


            _collisionGroups =
                new FeeSimNarrowPhaseCallbacks<CollisionEventHandler>
                {
                    Events = _events,
                    CollisionGroups = _bodyProperties
                };
            Simulation = Simulation.Create(BufferPool, _collisionGroups,
                new DefaultPoseIntegratorCallbacks(true /*new Vector3(0, -10, 0)*/));
            var boxShape = new Box(1, 1, 1);
            boxShape.ComputeInertia(1, out _boxInertia);
            _boxIndex = Simulation.Shapes.Add(boxShape);

            CreateFloors();
        }

        private void CreateFloors()
        {
            var floorShape = new Box(200, 1, 10);
            var floorShapeIndex = Simulation.Shapes.Add(floorShape);

            var position = new Vector3(0, -0.5f, -NumberOfFloors * floorShape.HalfLength);

            _floorReferences = new List<BodyReference>(NumberOfFloors);
            var offset = new Vector3(0, 0, floorShape.Length);
            for (var i = 0; i < NumberOfFloors; i++)
            {
                var floorIndex = CreateFloor(position, floorShapeIndex);
                position += offset;
                var floorReference = new BodyReference(floorIndex, Simulation.Bodies);
                ref var floorBodyProperties = ref _bodyProperties.Allocate(floorIndex);

                var floorCollisionGroup = new CollisionGroup(0b1);
                floorBodyProperties = new BodyProperty {Filter = floorCollisionGroup, Friction = 1f};
                _floorReferences.Add(floorReference);
            }
        }


        private int CreateFloor(in Vector3 position, in TypedIndex shapeIndex)
        {
            var floorDescription = new BodyDescription
            {
                LocalInertia = new BodyInertia(),
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings {Mode = ContinuousDetectionMode.Discrete},
                    Shape = shapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Activity = new BodyActivityDescription
                {
                    MinimumTimestepCountUnderThreshold = 32,
                    SleepThreshold = 0.1f
                },
                Pose = new RigidPose
                {
                    Position = position,
                    Orientation = Quaternion.Identity
                },
                ConveyorSettings = new ConveyorSettings()
            };

            return Simulation.Bodies.Add(floorDescription);
        }

        private void CreateCircleCube(int sampleCount, float radius, int floorCount)
        {
            var cubeCollisionGroup = new CollisionGroup(0b10);
            var wpCnt = sampleCount;
            var increment = MathHelper.Pi * 2 / wpCnt;

            var tempPos = Vector3.Zero;
            var offset = new Vector3(0, 0, 5);
            for (var j = 0; j < floorCount; j++)
            for (float i = 0; i < MathHelper.Pi * 2 - increment / 2; i += increment)
            {
                tempPos = new Vector3((float) Math.Sin(i) * radius, j * 2, (float) Math.Cos(i) * radius);
                CreateCube(tempPos);
            }
        }


        private void CreateCube(in Vector3 position)
        {
            var bodyDescription = new BodyDescription
            {
                //Make the uppermost block kinematic to hold up the rest of the chain.
                LocalInertia = _boxInertia,
                Pose = new RigidPose
                {
                    Position = position,
                    Orientation = Quaternion.Identity
                },
                Activity = new BodyActivityDescription
                {
                    MinimumTimestepCountUnderThreshold = 32,
                    SleepThreshold = .01f
                },
                Collidable = new CollidableDescription {Shape = _boxIndex, SpeculativeMargin = .1f}
            };
            var bodyIndex = Simulation.Bodies.Add(bodyDescription);
            ref var boxBodyProperty = ref _bodyProperties.Allocate(bodyIndex);
            var boxCollisionGroup = new CollisionGroup(0b10);
            boxBodyProperty = new BodyProperty {Filter = boxCollisionGroup, Friction = 1f};
        }


        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            base.Update(window, camera, input, dt);
            _events.Flush();

            #region  KeyInput

            if (input.WasPushed(Key.Z))
            {
                CreateCircleCube(25, 16, 11);
                CreateCircleCube(25, 14, 10);
                CreateCircleCube(25, 12, 10);
                CreateCircleCube(25, 10, 9);
            }

            var conveyorvelocity = Vector3.Zero;
            var conveyorVelocityChanged = false;

            if (input.WasDown(Key.Number1))
            {
                conveyorvelocity += new Vector3(2, 0, 0);
                conveyorVelocityChanged = true;
            }

            if (input.WasDown(Key.Number2))
            {
                conveyorvelocity += new Vector3(-2, 0, 0);
                conveyorVelocityChanged = true;
            }

            if (input.WasDown(Key.Number3))
            {
                conveyorvelocity += new Vector3(0, 0, 2);
                conveyorVelocityChanged = true;
            }

            if (input.WasDown(Key.Number4))
            {
                conveyorvelocity += new Vector3(0, 0, -2);
                conveyorVelocityChanged = true;
            }

            if (conveyorVelocityChanged)
                foreach (var floorReference in _floorReferences)
                {
                    floorReference.ConveyorSettings.ConveyorVelocity = conveyorvelocity;
                    floorReference.ConveyorSettings.IsLinearConveyor = true;
                    floorReference.Activity.SleepThreshold = -1f;
                    Simulation.Awakener.AwakenBody(floorReference.Handle);
                }

            if (input.WasPushed(Key.Number5))
                foreach (var floorReference in _floorReferences)
                {
                    floorReference.ConveyorSettings.IsLinearConveyor = false;
                    floorReference.Velocity.Linear = Vector3.Zero;
                    floorReference.Activity.SleepThreshold = 0.1f;
                }

            #endregion
        }

        public struct BodyProperty
        {
            public CollisionGroup Filter;
            public float Friction;
        }
    }
}