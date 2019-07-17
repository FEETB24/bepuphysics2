using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using Demos.Port;
using DemoUtilities;
using OpenTK.Input;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    public class BigConveyorDemo : Demo
    {
        private BodyDescription _boxDescription;
        private BodyDescription _horizontalConveyorDescription;
        private BodyDescription _verticalConveyorDescription;
        private BodyDescription _rampConveyorDescription;
        private BodyDescription _movingObject;

        private const float ConveyorSpeed = 2f;

        private readonly Quaternion[] _orientations = new Quaternion[4];

        private QuickList<int> _createdObjects;
        private QuickList<Movable> _movingObjects;

        private const float _conveyorHalfWidth = 1.5f;
        private const float _horizontalHalfLenght = 20;
        private const float _verticalHalfLenght = 5;

        private const float _rampDegrees = 10f;
        private const float _floorHeight = 3f;


        private const int _numberOfLoops = 10;
        private const int _numberOfLevels = 10;

        private const float SleepTime = 5;
        private Vector3 MovingOffset = new Vector3(0,0, _verticalHalfLenght * 4);


        private struct Movable
        {
            public Vector3 StartPosition;
            public bool IsMovingToOffset;
            public bool IsMoving;
            public float ToggleSleepTimer;
            public int BodyIndex;
        }


        public override void Initialize(ContentArchive content, Camera camera)
        {

            _orientations[0] = Quaternion.Identity;
            Quaternion.CreateFromYawPitchRoll(-(float)Math.PI * 0.5f, 0, 0, out _orientations[1]);
            Quaternion.CreateFromYawPitchRoll(-(float)Math.PI * 1.0f, 0, 0, out _orientations[2]);
            Quaternion.CreateFromYawPitchRoll(-(float)Math.PI * 1.5f, 0, 0, out _orientations[3]);

            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0f,-9.81f,0f))
               , new CustomPositionLastTimestepper()
                );

            _createdObjects = new QuickList<int>(_numberOfLevels * _numberOfLevels * 10, BufferPool);
            _movingObjects = new QuickList<Movable>(_numberOfLevels * _numberOfLevels, BufferPool);

            _boxDescription = CreateBoxDescription(1, 1, 1);
            _horizontalConveyorDescription = CreateConveyorDescription(_horizontalHalfLenght);
            _verticalConveyorDescription = CreateConveyorDescription(_verticalHalfLenght);
            _movingObject = CreateBoxDescription(4, 3, 2);
            _movingObject.LocalInertia = new BodyInertia();


            var angle = _rampDegrees * (float)(Math.PI / 180f);
            var rampLenght = _floorHeight / (float)Math.Sin(angle);

            _rampConveyorDescription = CreateConveyorDescription(rampLenght);
            _rampConveyorDescription.Pose.Orientation = Quaternion.CreateFromYawPitchRoll((float)Math.PI * 0.5f, 0, angle);
            _rampConveyorDescription.ConveyorSettings.ConveyorVelocity *= 2f;
            
          
            for (int i = 0; i < _numberOfLevels; i++)
            {
               

                var position = new Vector3(0, _floorHeight * i,0/* _verticalHalfLenght * i*/);

                for (int j = 0; j < _numberOfLoops; j++)
                {
                    CreateLoop(ref position);
                }

               // if (i < _numberOfLevels - 1)
                {
                    CreateRamp(ref position);
                }
            }

        }


        private BodyDescription CreateConveyorDescription(float halfLenght)
        {
            var description =  CreateBoxDescription(halfLenght * 2, 1f, _conveyorHalfWidth * 2);
            description.ConveyorSettings.IsLinearConveyor = true;
            description.ConveyorSettings.ConveyorVelocity = new Vector3(ConveyorSpeed, 0, 0);
            description.LocalInertia = new BodyInertia();
            description.Activity.SleepThreshold = -1;
            return description;
        }
        private BodyDescription CreateBoxDescription(float width, float height, float lenght )
        {
            var boxShape = new Box(width, height, lenght);
            var boxShapeIndex = Simulation.Shapes.Add(boxShape);
            boxShape.ComputeInertia(1, out var boxInertia);
            Symmetric3x3.Scale(boxInertia.InverseInertiaTensor, .5f, out boxInertia.InverseInertiaTensor);
            var boxDescription = new BodyDescription()
            {
                LocalInertia = boxInertia,
                Pose = new RigidPose
                {
                    Position = Vector3.Zero,
                    Orientation = Quaternion.Identity
                },
                Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = .01f },
                Collidable = new CollidableDescription { Shape = boxShapeIndex, SpeculativeMargin = .1f },
            };

            return boxDescription;
        }

        private void CreateLoop(ref Vector3 position)
        {

            //var movingPosition = position +new Vector3(0,1.5f, -2f);

            //var movingBodyIndex =  Instantiate(_movingObject,ref movingPosition, ref _orientations[0]);
            //var movable = new Movable()
            //{
            //    BodyIndex = movingBodyIndex,
            //    IsMoving = false,
            //    IsMovingToOffset = true,
            //    StartPosition = movingPosition,
            //    ToggleSleepTimer = 0,
            //};
            //_movingObjects.Add(movable, BufferPool);


            CreateHorizontalConveyor(ref position, ref _orientations[0]);
            CreateBoxes(position);

            position += new Vector3(_horizontalHalfLenght + _conveyorHalfWidth, 0, _verticalHalfLenght - _conveyorHalfWidth);
            CreateVerticalConveyor(ref position, ref _orientations[1]);
            position += new Vector3(-_horizontalHalfLenght + _conveyorHalfWidth, 0, _verticalHalfLenght + _conveyorHalfWidth);
            CreateHorizontalConveyor(ref position, ref _orientations[2]);

            CreateBoxes(position);

            position += new Vector3(-_horizontalHalfLenght - _conveyorHalfWidth, 0, _verticalHalfLenght - _conveyorHalfWidth);
            CreateVerticalConveyor(ref position, ref _orientations[1]);
            position += new Vector3(_horizontalHalfLenght - _conveyorHalfWidth, 0, _verticalHalfLenght + _conveyorHalfWidth);

        }

        private void CreateRamp(ref Vector3 position)
        {
            position.X -= _horizontalHalfLenght + _verticalHalfLenght - (_conveyorHalfWidth * 2);
            CreateVerticalConveyor(ref position, ref _orientations[2]);
            position.X -= _verticalHalfLenght + _conveyorHalfWidth;

            CreateVerticalConveyor(ref position, ref _orientations[3]);
            
            var angle = _rampDegrees * (float)(Math.PI / 180f);
            var rampX = _floorHeight / (float) Math.Tan(angle);
            position.Z -= _verticalHalfLenght + (rampX );
            position.Y += _floorHeight * 0.5f;
            Instantiate(_rampConveyorDescription, ref position, ref _rampConveyorDescription.Pose.Orientation);

            position.Z -= _verticalHalfLenght + (rampX);
            position.Y += _floorHeight * 0.5f;


            CreateVerticalConveyor(ref position, ref _orientations[3]);


        }

        private void CreateBoxes(Vector3 position)
        {
            position.Y += 2;
            for (var i = -(_horizontalHalfLenght - 1); i <= (_horizontalHalfLenght - 1); i += 2)
            {
                position.X = i;
                Instantiate(_boxDescription, ref position, ref _orientations[0]);
            }
        }

        private void CreateHorizontalConveyor(ref Vector3 position, ref Quaternion orientation)
        {
            Instantiate(_horizontalConveyorDescription, ref position, ref orientation);
        }

        private void CreateVerticalConveyor(ref Vector3 position, ref Quaternion orientation)
        {
            Instantiate(_verticalConveyorDescription, ref position, ref orientation);
        }

        private int Instantiate(BodyDescription bodyReference, ref Vector3 position, ref Quaternion orientation)
        {
            bodyReference.Pose.Position = position;
            bodyReference.Pose.Orientation = orientation;
            var index = Simulation.Bodies.Add(bodyReference);
            _createdObjects.Add(index, BufferPool);
            return index;
        }

        private float _spawnCount = 0;
        private float _spawnTime = 2f;

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            base.Update(window, camera, input, dt);
            #region  KeyInput

            //Delete all boxes;
            if (input.WasPushed(Key.Z))
            {
                for (var i = _createdObjects.Count - 1; i > 0; --i)
                {
                    var bodiIndex = _createdObjects[i];
                    var reference = new BodyReference(bodiIndex, Simulation.Bodies);
                    if (reference.LocalInertia.InverseMass > 0)
                    {
                        Simulation.Bodies.Remove(bodiIndex);
                        _createdObjects.RemoveAt(i);
                    }
                }
            }

            if (input.WasPushed(Key.X))
            {


                for (var j = 0; j < _numberOfLevels; j++)
                {
                    var startPosition = new Vector3(0, _floorHeight * j, _verticalHalfLenght * j);
                    for (int i = 0; i < _numberOfLoops * 2; i++)
                    {
                        CreateBoxes(startPosition);
                        startPosition.Z += _verticalHalfLenght * 2;
                    }
                }


            }
            var random = new Random(Guid.NewGuid().GetHashCode());


            //Randomly destroy
            if (input.WasPushed(Key.C))
            {
             



                for (var i = _createdObjects.Count - 1; i > 0; --i)
                {
                    var bodiIndex = _createdObjects[i];
                    var reference = new BodyReference(bodiIndex, Simulation.Bodies);
                    if ((reference.LocalInertia.InverseMass > 0) && random.Next(100) > 50)
                    {
                        Simulation.Bodies.Remove(bodiIndex);
                        _createdObjects.RemoveAt(i);
                    }
                }
            }

            _spawnCount += dt;
            if (_spawnCount >= _spawnTime)
            {
                _spawnCount -= _spawnTime;
                var boxPosition = new Vector3(-_horizontalHalfLenght + 1 , 2, 0);
                var identity = Quaternion.Identity;
                Instantiate(_boxDescription, ref boxPosition, ref identity);



                var deleteObject = random.Next(_createdObjects.Count-1);
                var reference = new BodyReference(deleteObject, Simulation.Bodies);
                if ((reference.LocalInertia.InverseMass > 0))
                {
                    Simulation.Bodies.Remove(deleteObject);
                    _createdObjects.Remove(deleteObject);
                }
     
            }


            for (var i = 0; i < _movingObjects.Count; i++)
            {
                ref var movingObject = ref _movingObjects[i];
               

                if (movingObject.IsMoving)
                {
                    var bodyReference = new BodyReference(movingObject.BodyIndex, Simulation.Bodies);
                    var targetPosition = movingObject.StartPosition;
                    if (movingObject.IsMovingToOffset)
                    {
                        targetPosition += MovingOffset;
                    }

                    var linearError = targetPosition - bodyReference.Pose.Position;

                    if (linearError.LengthSquared() < .0001f)
                    {
                        bodyReference.Velocity.Linear = Vector3.Zero;
                        movingObject.IsMoving = false;
                        movingObject.IsMovingToOffset = !movingObject.IsMovingToOffset;
                    }
                    else
                    {
                        var linearVelocity = linearError / dt;
                        bodyReference.Velocity.Linear = linearVelocity;
                        Simulation.Awakener.AwakenBody(bodyReference.Handle);
                    }
                }
                else
                {
                    movingObject.ToggleSleepTimer += dt;
                    if (movingObject.ToggleSleepTimer > SleepTime)
                    {
                        movingObject.ToggleSleepTimer = SleepTime * (float)random.NextDouble();
                        movingObject.IsMoving = true;
                    }
                }



            }

            //I know it is unoptimized but it is the easier way to delete the falling boxes.
            for (var i = _createdObjects.Count - 1; i > 0; --i)
            {
                var bodiIndex = _createdObjects[i];
                var reference = new BodyReference(bodiIndex, Simulation.Bodies);
                if ((reference.Activity.SleepThreshold > 0) && reference.Pose.Position.Y < -50000000)
                {
                    Simulation.Bodies.Remove(bodiIndex);
                    _createdObjects.RemoveAt(i);
                }
            }




            #endregion
        }
    }
}